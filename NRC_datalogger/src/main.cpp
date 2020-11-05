/*
	NRC_datalogger
	
	Firmware for use with the WatHydra hardware
	Logger wakes from a number of inputs and records data from sensors that is logged to an SD card. There is also GPS for accurate timing.
	
	CODE by Dirk Friesen with contributions from Ilia Baranov and Sunaal Mathew
	October 2018
	
 	This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License. To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/ or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
*/
#include <Arduino.h>
#include <stdio.h>
#include <IntervalTimer.h>
#include <Metro.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Snooze.h>
#include <SPI.h>
#include <TimeLib.h>
#include "../lib/Adafruit_FONA/Adafruit_FONA.h"
#include "../lib/Adafruit_LSM9DS1/Adafruit_LSM9DS1.h"
#include "../lib/Adafruit_Sensor/Adafruit_Sensor.h"  // not used in this demo but required!
#include "../lib/CircularBuffer/CircularBuffer.h"
#include "../lib/Timezone/src/Timezone.h"

// Firmware version string
static char FirmwareVersion[] = "v2.1.6 - beta - " __DATE__ " " __TIME__ "";
#define DEBUGGING_OUT true

// Load sleep drivers
SnoozeAlarm  alarm; // Using RTC
SnoozeDigital digital;	// Wake from pin interrupt
SnoozeBlock config_teensy36(alarm, digital); // add alarm driver to main Snoozeblock 
int sleep_period_hrs = 0; 
int sleep_period_mins = 0; 
int sleep_period_secs = 0;

// FONA configuration
#define FONA_TIMEOUT 10000
const int	RX_PIN  = 10;
const int	TX_PIN  = 9;
const int	FONA_RST = 25;
const char * holo_key = "(hneFa_9";
SoftwareSerial fonaSS = SoftwareSerial(TX_PIN, RX_PIN);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

// Timezone and Time change rules
tmElements_t tm;
TimeChangeRule *tcr;	// pointer to time change rule for printing time zone abbreviation
TimeChangeRule canEDT = {"EDT", Second, Sun, Mar, 2, -240};  //UTC - 4 hours
TimeChangeRule canEST = {"EST", First, Sun, Nov, 2, -300};   //UTC - 5 hours
Timezone canEastern(canEDT, canEST);
static const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";

// LSM9DS1 setup
const int CS_AG = 15;
const int CS_M = 35;    // Necessary for initialization. Should be an otherwise unused pin
Adafruit_LSM9DS1 ExtSensor = Adafruit_LSM9DS1(CS_AG, CS_M);   // Uses hardware SPI so we only pass the chip select pins

// main board sensors
const int Temp = 14;	// requires special analog input settings to not spoil data
const int A_x = 18;
const int A_y = 17;
const int A_z = 16;
const int BATT_V = A20;			// battery voltage sense

// Wind sensor
const int WIND_SPEED_PIN = 2;
const int WIND_DIR_PIN = 20; // A6

// Main board pins
const int MCU_STAT = 22;    // Status LED
const int EN_SENSE = 21;    // Main board sensor power
const int EN_DATA = 19;     // Enable 3V6 regulator
const int NOT_C_ON = 24;    // SIM808 power
const int SD_CS = BUILTIN_SDCARD; // SD card chip select
const int MODE = 6;         // mode switch pin

// Timers
Metro serialPrintTimer = Metro(5000);	// triggers serial print every 5 s
Metro samplePeriod = Metro(50);         // collect a sample every 50 ms (20 Hz)

// Global variables
uint32_t wind_pulse_count;
uint32_t logDuration = 60;	// duration of logging in seconds
float wind_speed_threshold = 0;    // km/hr
bool useFONA = false, stayOn = false;
SDClass SD;
char lastFileSent[] = "09281613.CSV";
char filesToSend[60][13];      // simple LIFO buffer of filenames with data that needs to be sent
uint8_t file_stack_ptr = 0;

void initializeSD();    // Open SD file system
void SD_dateTime(uint16_t* date, uint16_t* time);   // callback function for setting file timestamps on SD
float readBattVolt();   // For accurate voltage measurements the battery should be in a quasi-OC state (low current draw)
void loggingFun(File *dataFile);    // Core logging loop
void sleepCheck (); // Checks if the logger should sleep and if so it goes to sleep
void wakeUp();  // Turns on peripheral power supplies/modules
bool digitalWakeEnable();   // Enables wake from mode switch if hardware supports it. Returns true if enabled.
                            // NOTE: Switch must be in 'LOW' position for test to work
void getUserTime(); // get the local time and date from serial interface
void startFONA();   // Turn on FONA and get Network time
void parseTime(char TimeStr[], bool isMacro); // convert user time input or network time into Teensy time struct
void digitalClockDisplay();
void windPulse();   // Pulse count ISR
void sendStatus();	// sends the battery voltage, and threshold value
void sendData();    // sends the most recent data files
uint8_t make_json( char *json_obj, uint32_t time_s, uint32_t time_ms, float *val, char *name);       // format a json string from the provided variables
uint8_t make_json(char *json_obj, uint32_t time_s, uint32_t time_ms, uint16_t *wind, float temperature);
void updateThreshold();   // reads a message from the hologram dashboard and updates the wind threshold for logging

void setup() {
	char c = '\0';
    char time_s[20];
	bool digitalWake = false;

    // set default time to compile time
    strcpy(time_s, __DATE__);
    strcat(time_s, " ");
    strcat(time_s, __TIME__);
    parseTime(time_s, true);

    Serial.begin(115200);
    Serial.printf("time_s: %s\n", time_s);

    // configure pins
	pinMode (CS_AG, OUTPUT);
	pinMode (MCU_STAT, OUTPUT);
	pinMode (EN_SENSE, OUTPUT);
	pinMode (EN_DATA, OUTPUT);
	pinMode (NOT_C_ON, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	
	digitalWrite (EN_DATA, LOW);	// FONA power supply off
	digitalWrite (NOT_C_ON, HIGH);	// FONA off

	delay(500);   // wait for serial
	Serial.printf("NRC Logger - %s\r\n\n", FirmwareVersion);
	Serial.println(time_s);

	digitalWake = digitalWakeEnable();	// enable wake from Mode switch if compatible with hardware
	Serial.printf("Mode Switch Wake-ups: %s\n\r", digitalWake ? "Yes" : "No");
	
	Serial.printf("Use FONA? (Y/n)\n\r");
	elapsedMillis serial_timeout = 0;
	while (!Serial.available() && (serial_timeout < 15000)) {}
	c = Serial.read();
	if ((c == 'n') || (c == 'N')) {
        getUserTime();
	}
	else  {
		useFONA = true;
		Serial.println("Starting FONA chip...");
	}
}

void loop() {
	char filename[13];

    wakeUp();	// Power on external modules
    initializeSD();
    delay(500);
    updateThreshold();  // check for an incoming SMS with a different wind threshold for logging
//    sendStatus();

	// Display test parameters
	Serial.println("\nStarting Sample");

	// Open file
	sprintf(filename, "%02i%02i%02i%02i.csv", month(), day(), hour(), minute());  // generate filename
	File dataFile;	// create file
	while (!(dataFile = SD.open(filename, FILE_WRITE))) {	// try to open file with write permission
		Serial.println("There was a problem with the SD card.");  
		sprintf(filename, "%02i%02i%02i%02i.csv", month(), day(), hour(), minute());  // generate filename
	}
	Serial.printf("File: %s\n\r", filename);

    // print headers
	Serial.println("Seconds,\tAcc_x,\tAcc_y,\tAcc_z,\tGyro_x,\tGyro_y,\tGyro_z,\tMag_x,\tMag_y,\tMay_z,\tWind_V,\tWind_D");
	dataFile.println("S,ms,Accel_x,Accel_y,Accel_z,Gyro_x,Gyro_y,Gyro_z,Mag_x,Mag_y,Mag_z,WindV,WindD,T");

	// start logging
	wind_pulse_count = 0;
	loggingFun(&dataFile);

	// close down logging
	dataFile.close();   // close file
	Serial.printf("Finished logging to %s\n\n\r", filename);

	// Add the file to the LIFO queue
    strcpy(*(filesToSend+file_stack_ptr), filename);
//    *(filesToSend+file_stack_ptr++) = filename;
    Serial.println(filesToSend[file_stack_ptr]);

	// Go to sleep now (maybe, if the time is right)
    if (!stayOn) {
        sendData();         // send the data
        sleepCheck();       // check if it's time to be sleeping
    }
    digitalClockDisplay();
}

void parseTime(char *TimeStr, bool isMacro) {
    if (!isMacro) {
        char * tok;
        uint8_t field = 0;	// which tm field we're on
        tok = strtok(TimeStr, " /,:+-");
        while (tok != nullptr) {
            switch (field) {
                case 0 :
                    tm.Year = atoi(tok);
                    break;
                case 1 :
                    tm.Month = atoi(tok);
                    break;
                case 2 :
                    tm.Day = atoi(tok);
                    break;
                case 3 :
                    tm.Hour = atoi(tok);
                    break;
                case 4 :
                    tm.Minute = atoi(tok);
                    break;
                case 5 :
                    tm.Second = atoi(tok);
                    break;
                default:
                    break;
            }
            field++;
            tok = strtok(NULL, " /,:+-");
        }
    }
    else {
        char buff[5] = "\0";
        unsigned int month, hour, minute, second;   // for some reason sscanf needs these as intermediaries
        sscanf(TimeStr, "%s %u %u %u:%u:%u", buff, &tm.Day, &tm.Year, &hour, &minute, &second);
        month = (strstr(month_names, buff)-month_names)/3+1;
        tm.Month = month;
        tm.Hour = hour;
        tm.Minute = minute;
        tm.Second = second;
    }
    setTime(tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tm.Year);
    Teensy3Clock.set(makeTime(tm));
    digitalClockDisplay();
}

bool digitalWakeEnable() {
    bool pinState = digitalRead(EN_SENSE);
    digitalWrite(EN_SENSE, HIGH);		// turn on main board 3V3 supply
    delay(10);											// allow power supply settling
    if (!digitalRead(MODE))	{				// check if MODE pin remains low (trace cut or switch in FULL position)
        pinMode(MODE, INPUT_PULLUP);	// enable pullup resistor
        delay(10);										// allow pin to be pulled up
        if ( !digitalRead(MODE) ) {			// check if MODE pin is still low
            Serial.println("Move switch to 'LOW' position");
            elapsedMillis switchTimer = 0;
            while ( (switchTimer < 10000) && !digitalRead(MODE) ) {}
        }
        if (digitalRead(MODE)) {
            digital.pinMode(MODE, INPUT_PULLUP, FALLING);	// trace is cut so we can enable the wake from switch functionality
            return true;
        }
    }
    digitalWrite(EN_SENSE, pinState);	// return 3V3 supply to previous state
    return false;
}

void wakeUp() {
    Serial.printf("Battery Voltage: %.2f V\n\r", readBattVolt());	// Measure the battery voltage

    // Turn peripherals back on
    digitalWrite (EN_SENSE, HIGH);	// Power on main board sensors

    if (useFONA)
        startFONA();

    if (!ExtSensor.begin()) {
        Serial.println("there's a problem with the offboard sensor");
    }
    ExtSensor.setupAccel(Adafruit_LSM9DS1::LSM9DS1_ACCELRANGE_2G);
    ExtSensor.setupGyro(Adafruit_LSM9DS1::LSM9DS1_GYROSCALE_245DPS);
}

void startFONA() {
    char timeBuf[21];	// holds time string returned from FONA
    // Power up sequence
    digitalWrite (EN_DATA, HIGH);	// enable fona power supply
    delay(100);	// power supply settling
    digitalWrite (NOT_C_ON, LOW);	// bring fona power key pin low
    delay(1100);			// power key must be low >1s
    digitalWrite (NOT_C_ON, HIGH);	// return fona power key pin

    fonaSerial->begin(115200);
    if (! fona.begin(*fonaSerial)) {
        Serial.printf("Couldn't find FONA\n\r");
        useFONA = false;
    } else {
        fona.enableGPRS(true);

        char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
        Serial.print("IMEI: ");
        Serial.println(fona.getIMEI(imei));
        while (fona.getRSSI() < 5) {
            Serial.printf("Signal: %i", fona.getRSSI());
            delay(100);
        }
        Serial.printf("Network Status: %i\n", fona.getNetworkStatus());
        while (fona.getNetworkStatus() != 2) {
            delay(100);
        }
        Serial.print(fona.enableNetworkTimeSync(true));
        delay(10000);   // delay to allow the 808 chip to get the time from the network
        fona.getTime(timeBuf, 25);
        parseTime(timeBuf, false);
    }
}

void initializeSD() {
    // Serial.print(F("Initializing SD card..."));
    SdFile::dateTimeCallback(SD_dateTime);	// enable the SD file system to retrieve the time
    File dummyFile;
    if (!(dummyFile = SD.open("checkSD.txt", FILE_WRITE))) {
        if (!SD.begin(SD_CS)) {
            Serial.printf("Card failed, or not present.\n\r");
            Serial.printf("Insert SD card.\n\r");
            while(!SD.begin(SD_CS)) {
                delay(500);
            }
            Serial.printf(F("Card initialized.\n\r"));
        }
    }
    else
        SD.remove("checkSD.txt");
}

void SD_dateTime(uint16_t* date_16, uint16_t* time_16) {	// function that SdFat calls to get time
    *date_16 = FAT_DATE(year(), month(), day());		// return date using FAT_DATE macro to format fields
    *time_16 = FAT_TIME(hour(), minute(), second());	// return time using FAT_TIME macro to format fields
}

float readBattVolt() {
    float avg = 0, voltage;
    Serial.print(analogRead(BATT_V));
    for (int ii = 1; ii < 12; ii++) {	// average 12 samples
        avg = avg + ((float) analogRead(BATT_V)  - avg)/(float)ii;
        delay(1);
    }
    voltage = avg*(float)0.010956 - (float)0.0493;		// empirical conversion equation (assumes 12-bit)
    return voltage;
}

void loggingFun(File *dataFile) {
    bool stop_logging = false;
    uint32_t logStartTime;
    uint16_t wind_dir;
    sensors_event_t acc, mag, gyro, temp;

    pinMode(WIND_SPEED_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(WIND_SPEED_PIN), windPulse, FALLING);

    fona.enableGPS(true);
    serialPrintTimer.reset();
    samplePeriod.reset();
    logStartTime = now();

    // logging loop
    while ( !stop_logging ) {
        if ((now() - logStartTime) >= logDuration) {
            stop_logging = true;
        }
        if (samplePeriod.check() == true) {
            ExtSensor.read();
            wind_dir = analogRead(WIND_DIR_PIN);
            ExtSensor.getEvent(&acc, &mag, &gyro, &temp);
            dataFile->printf("%i,%u,", second(), micros());
            dataFile->printf("%5f,%5f,%5f,", acc.acceleration.x, acc.acceleration.y, acc.acceleration.z);
            dataFile->printf("%5f,%5f,%5f,", gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
            dataFile->printf("%5f,%5f,%5f,", mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
            dataFile->printf("%i,%i,%3f\n", wind_pulse_count, wind_dir, temp.temperature);

            // for debugging
//            dataFile->printf("%5f,%5f,%5f,", 5.0, 4.0, 3.0);
//            dataFile->printf("%5f,%5f,%5f,", 5.0, 6.0, 7.0);
//            dataFile->printf("%5f,%5f,%5f,", 1.1, 2.2, 3.3);
//            dataFile->printf("%i,%i,%3f\n", 1, 300, 20.5);
        }

        if ((serialPrintTimer.check() == true) && (DEBUGGING_OUT)) {
            Serial.printf("%7i,", second());
            Serial.printf("\t%.3f,\t%.3f,\t%.3f,", acc.acceleration.x, acc.acceleration.y, acc.acceleration.z);
            Serial.printf("\t%.3f,\t%.3f,\t%.3f,", gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
            Serial.printf("\t%.3f,\t%.3f,\t%.3f", mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
            Serial.printf("\t%i,\t%i,\t%.2f\n", wind_pulse_count, wind_dir, temp.temperature);
        }
    }
    float avg_wind_speed = wind_pulse_count*3.620/logDuration;
    if (avg_wind_speed >= wind_speed_threshold)
        stayOn = true;
    else
        stayOn = false;
}

void sleepCheck () {//
    int who; // For snooze library

    // Check mode switch pin state
    if (!digitalRead(MODE)) return;		// Mode switch == FULL (V == 0V) -> no sleeping

    if(minute()>0) {
        sleep_period_mins = 58-minute();
        sleep_period_secs = 60-second();
    }
    else {
        sleep_period_mins = 59;
        sleep_period_secs = 60-second();
    }

    // Turn off peripherals
    Serial.println("Turning peripherals off");
    digitalWrite (EN_SENSE, LOW);	// Power off main board sensors
    digitalWrite (NOT_C_ON, LOW);	// Toggle FONA power
    delay(1000);
    digitalWrite (NOT_C_ON, HIGH);
    delay(2500);
    digitalWrite (EN_DATA, LOW);	// turn off fona power supply

    // Logging sleep period
    Serial.println("OK. Going to sleep now...");
    digitalClockDisplay();
    Serial.printf("Battery Voltage: %.2f V\n\r", readBattVolt());
    Serial.printf("Sleep Period HRS = %i\n\r", sleep_period_hrs);
    Serial.printf("Sleep Period MINS = %i\n\r", sleep_period_mins);
    Serial.printf("Sleep Period SECS = %i\n\r", sleep_period_secs);

    delay(1000);

    // Set RTC alarm wake up in (hours, minutes, seconds).
    alarm.setRtcTimer(sleep_period_hrs, sleep_period_mins, sleep_period_secs);// hour, min, sec
    // Send Teensy to deep sleep
    who = SnoozeClass::hibernate( config_teensy36 );
    setTime(teensy3_clock_class::get()); // load RTC time onto MCU
    if (who == 35) // rtc wakeup value
        Serial.println("Woke up through RTC");
    else if (who == MODE)  // switch wakeup value
        Serial.println("Woke up through mode switch");
}

void getUserTime() {
    uint8_t count = 0;
    char c;
    char userTime[18];

    // request user input time via serial connection
    Serial.clear();
    Serial.printf("Enter time (yy/MM/dd,hh:mm:ss)\n");
    while (!Serial.available()) {}    // pause for user input
    do  {
        c = Serial.read();
        userTime[count] = c;
        count++;
    } while ((c != '\n') && (c != '\r'));	// newline or carriage return character received
    parseTime(userTime, false);
}

void digitalClockDisplay() {
    Serial.printf("%2i:%02i:%02i %s %i, %i %s\n\r", hour(), minute(), second(), monthStr(month()), day(), year(), tcr -> abbrev);
}

void windPulse() {
    wind_pulse_count++;
}

void sendStatus() {		// sends the battery voltage
    char message[50] = "{";
    Serial.println("Sending status update");
    while(fona.getRSSI() < 10)	delay(200);

    sprintf(message, R"("battery":%.02f,"Thresh":%2f,"stayOn":%d)", readBattVolt(), wind_speed_threshold, stayOn);
    uint8_t sendAttempts = 0;
    while (!fona.Hologram_send(message, holo_key) && sendAttempts < 5) {
        sendAttempts++;
    }

    // This should shut down the data side of the FONA chip
    //fona.sendCheckReply(F("AT+CIPSHUT"), F("SHUT OK"), 20000);	// It might not have worked
    fona.enableGPRS(false);
}

void sendData() {
    bool sendNext = false;
    while(fona.getRSSI() < 10)	delay(200);
    Serial.println(filesToSend[file_stack_ptr]);
    File file_sd = SD.open(*(filesToSend+file_stack_ptr), FILE_READ);
    Serial.print("Sending file: ");
    Serial.println(file_sd.name());
    Serial.printf("Filename: %s\n", file_sd.name());
    char row[120];
    Serial.print("Sending data...");
    unsigned int fileSize = file_sd.size();             // Get the file size.
    Serial.printf("File: %s \t\tsize: %i bytes (or %i)\n", file_sd.name(), fileSize, file_sd.size());
    file_sd.readBytesUntil('\n', row, 120);
    while (file_sd.available()) {
//                S,ms,Accel_x,Accel_y,Accel_z,Gyro_x,Gyro_y,Gyro_z,Mag_x,Mag_y,Mag_z,WindV,WindD,T"
        uint32_t Seconds, Milliseconds;
        float Accel[3], Gyro[3], Mag[3], Temperature;
        uint16_t Wind[2];   // Speed and Direction
        char msg_data[64];
        file_sd.readBytesUntil('\n', row, 120);
        sscanf(row, "%lu,%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%hu,%hu,%f", &Seconds, &Milliseconds,
                &Accel[0], &Accel[1], &Accel[2], &Gyro[0], &Gyro[1], &Gyro[2],
                &Mag[0], &Mag[1], &Mag[2], &Wind[0], &Wind[1], &Temperature);

        // Send Acceleration
        make_json(msg_data, Seconds, Milliseconds, Accel, "Acc");
        uint8_t sendAttempts = 0;
        while (!fona.Hologram_send(msg_data, holo_key, "D_") && sendAttempts < 5) {
            sendAttempts++;
        }
        // Send Gyro
        make_json(msg_data, Seconds, Milliseconds, Gyro, "Gyr");
        sendAttempts = 0;
        while (!fona.Hologram_send(msg_data, holo_key, "D_") && sendAttempts < 5)
            sendAttempts++;

        make_json(msg_data, Seconds, Milliseconds, Mag, "Mag");
        sendAttempts = 0;
        while (!fona.Hologram_send(msg_data, holo_key, "D_") && sendAttempts < 5)
            sendAttempts++;

        make_json(msg_data, Seconds, Milliseconds, Wind, Temperature);
        sendAttempts = 0;
        while (!fona.Hologram_send(msg_data, holo_key, "D_") && sendAttempts < 5)
            sendAttempts++;

        strcpy(filesToSend[file_stack_ptr--], "\0");    // clear filename from LIFO buffer
        file_sd.close();                                // Close the file.
        file_sd = SD.open(filesToSend[file_stack_ptr], FILE_READ);
        Serial.print("sent data\n");
        // Debugging
        Serial.println("match found");
        Serial.printf("file_sd.name(): %s", file_sd.name());
    }
}

uint8_t make_json(char *json_obj, uint32_t time_s, uint32_t time_ms, float *vals, char *name) {
    uint8_t len;
    len = sprintf(json_obj, R"({"s":%lu,"ms":%lu,"%s":[%f,%f,%f]})", time_s, time_ms, name, vals[0], vals[1], vals[2]);
    json_obj[len] = '\0';
    return len;
}

uint8_t make_json(char *json_obj, uint32_t time_s, uint32_t time_ms, uint16_t *wind, float temperature) {
    uint8_t len;
    len = sprintf(json_obj, R"({"s":%lu,"ms":%lu,"W":[%hu,%hu],"T":%4f})", time_s, time_ms, wind[0], wind[1],
                  temperature);
    json_obj[len] = '\0';
    return len;
}

void updateThreshold() {
    while(fona.getRSSI() < 10)	delay(200); // Connect to network if we're not already connected
    uint16_t msg_len, msg_max_len = 250;
    char msg_in[msg_max_len];               // buffer for incoming message
    int8_t smsnum = fona.getNumSMS();

    Serial.printf("old threshold: %f\n", wind_speed_threshold);
    for (int8_t smsn = 1; smsn <= smsnum; smsn++) {
        fona.readSMS( smsn, msg_in, msg_max_len, &msg_len);
        Serial.println(msg_in);
        if (msg_len == 0) {
            smsnum++;
            continue;
        }
        if (strstr( msg_in, "Threshold:") != nullptr) {     // Check for variations of "Threshold"
            Serial.println("match");
            sscanf(msg_in, "%*s %f", &wind_speed_threshold);
        }

    }
    Serial.printf("new threshold: %f\n", wind_speed_threshold);
}
