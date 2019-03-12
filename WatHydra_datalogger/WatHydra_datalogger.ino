/*
	WatHydra_datalogger
	
	Firmware for use with the WatHydra hardware
	Logger wakes from a number of inputs and records data from sensors that is logged to an SD card. There is also GPS for accurate timing.
	
	CODE by Dirk Friesen with contributions from Ilia Baranov and Sunaal Mathew
	October 2018
	
 	This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License. To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/ or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
*/

//#include "libraries/Adafruit_FONA_Wathydra/Adafruit_FONA.h"
#include "src/FONA/Adafruit_FONA.h"
#include <circular_buffer.h>
#include "src/SD/SD.h"
#include <SPI.h>
#include "src/MAX11254_lib/MAX11254.h"
#include <Metro.h>
#include <TimeLib.h>
#include <Timezone.h>
#include <SoftwareSerial.h>
#include <Snooze.h>

// Sleep related macros
#define AWAKE_TIME_START_HRS 1	// Local
#define AWAKE_TIME_END_HRS   7	// Local

// Firmware version string
static char FirmwareVersion[] = "v2.1.6 - beta - Wake 1:00-7:00 Local Time";

// Load drivers
SnoozeAlarm  alarm; // Using RTC
SnoozeDigital digital;	// Wake from pin interrupt
SnoozeBlock config_teensy36(alarm, digital); // add alarm driver to main Snoozeblock 
int sleep_period_hrs = 0; 
int sleep_period_mins = 0; 
int sleep_period_secs = 0;

// GPS
const int	RX_PIN  = 10;
const int	TX_PIN  = 9;
const int	FONA_RST = 25;
const int	PPS_PIN = 23; // 1PPS from SIM808 module
SoftwareSerial fonaSS = SoftwareSerial(TX_PIN, RX_PIN);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

// Timezone and Time change rules
TimeChangeRule *tcr;	// pointer to time change rule for printing time zone abbreviation
TimeChangeRule canEDT = {"EDT", Second, Sun, Mar, 2, -240};  //UTC - 4 hours
TimeChangeRule canEST = {"EST", First, Sun, Nov, 2, -300};   //UTC - 5 hours
Timezone canEastern(canEDT, canEST);

// Moon board initialization
const long SPI_SPEED = 2000000;
MAX11254 moon_adc = MAX11254();
const int CS = 15;
const int EN_MOON = 20;

// moon board sensors
const int Hydrophone_TC = 0;
const int Hydrophone_SQ = 1;
const int Press = 2;
const int Temp = 3;
const int A_x = 4;
const int A_y = 5;
const char sensorName[6][10] = {"Passive H", "Active H", "Pressure", "Temp Moon", "Ax Moon", "Ay Moon"};

// Main board pins
const int MCU_STAT = 22;    // Status LED
const int EN_SENSE = 21;    // Main board sensor power
const int EN_DATA = 19;     // Enable 3V6 regulator
const int NOT_C_ON = 24;    // SIM808 power
const int SD_CS = BUILTIN_SDCARD; // SD card chip select
const int RDYB = 2;         // ADC data ready bit
const int MODE = 6;         // mode switch pin
const int BATT_V = A20;			// battery voltage sense

// Timers
Metro serialPrintTimer = Metro(10000);	//trigger serial print every 10 s

// Global variables
unsigned long logDuration[6], sec = 0;	// duration of logging in seconds, sec is incremented with received pps
int numChannels = 1, channel[6], numTestSeqs = 0, sequenceNum = 1, sequence_channel_num, data_temp[3];
uint8_t receivedPPS = 0;
float Batt_volt = 0;
elapsedMicros ss;
bool useFONA = false, sessionStarted = false, infiniteLog = false, interleaving = false;
tmElements_t tm;
Circular_Buffer<int, 4000, 3> DataBuf;

// Open SD file system
void initializeSD(SDClass *SD) {
	//Serial.print(F("Initializing SD card..."));
  if (!SD->begin(SD_CS)) {
    Serial.printf("Card failed, or not present.\n\r");
    Serial.printf("Insert SD card.\n\r");
    while(!SD->begin(SD_CS)) {
      delay(500);
    }
    Serial.printf(F("Card initialized.\n\r"));
  }
}

// Get user input for channels to log and duration
void sessionSetup() {
  char serial_val;
  int channel_settle_start = 0, ii = 0;
  
  Serial.println("\nNew Session\n");
  
  // Calibrate ADC
  moon_adc.self_cal();
  // start to eat bad values (power supply settling)
  moon_adc.select_channel(Hydrophone_SQ);
  moon_adc.set_sample_rate(15);
  channel_settle_start = now();
  
  // Multiple channels?
  Serial.clear();
  Serial.printf("Number of channels to log?\n\r");
  while (!Serial.available()) {}      // pause for user input
  serial_val = Serial.read() - '0';
  if ((serial_val > 0) && (serial_val <= 6)) numChannels = serial_val;  // check validity, defaults to 1
  Serial.printf("%i\n\r", numChannels);

  // Get channel configurations
  for ( ii = 0; ii < numChannels; ii++) {
    channelConfig(ii);
  }

  // get number of samples to log
  Serial.clear();
  Serial.printf("\nHow many test sequences would you like to run? (enter '0' for continuous logging)\r\n");
  while (!Serial.available()) {}      // pause for user input
  ii = 0;   // counter for max digits
  numTestSeqs = 0;  // reset value to zero
  while (Serial.available() && ii<5) {
    serial_val = Serial.read() - '0';
    if((serial_val >= 0) && (serial_val <= 9)) { 
      numTestSeqs = numTestSeqs*10 + serial_val;
      ii++;
    }
  }
  if (numTestSeqs == 0) {
    infiniteLog = true;
    Serial.println("Continuous logging selected\n");
  }
  else {
    Serial.printf("%i\n\r", numTestSeqs);
  }
  sequenceNum = 1;
  
  // wait for 5 seconds for ADC channel to settle from power on
  while ((now() - channel_settle_start) < 5) {} 
      
  sessionStarted = true;
}  

// Setup channel to log
void channelConfig(int channelNum) {
  int serial_val, ii;

  Serial.printf("\nSequencer Position %i of %i\n\r", channelNum+1, numChannels);
  // get the channel
  Serial.clear();
  Serial.printf("\nSelect channel to log\n\r");
  Serial.printf("Passive Hydrophone = 0\n\rActive Hydrophone = 1\n\rPressure Transducer = 2\n\r");
  while (!Serial.available()) {}      // pause for user input
  channel[channelNum] = Serial.read() - '0';
  Serial.printf("%i\n\r", channel[channelNum]);

  // get sample duration
  Serial.clear();
  Serial.printf("How many minutes would you like each sample to be?\r\n");
  while (!Serial.available()) {}      // pause for user input
  ii = 0;
  logDuration[channelNum] = 0;
  while (Serial.available() && ii<5) {
    serial_val = Serial.read() - '0';
    if((serial_val >= 0) && (serial_val <= 9)) { 
      logDuration[channelNum] = logDuration[channelNum]*10 + serial_val*60;
      ii++;
    }
  }
  Serial.printf("%i\n\r", logDuration[channelNum]/60);  // print back value
}

// get the local time and date from serial interface
void getUserTime() {
  time_t externalTime_t = 0;
  tmElements_t tm;
  int count = 0;
  char c;
  char userTime[10];

  // request user input time via serial connection
  Serial.clear();
  Serial.printf("Send 'T' followed by date (YYMMDD) and time (HHMM)\n\r");
  Serial.printf("e.g. T1806071620\n\r");
  while (!Serial.available()) {}    // pause for user input
  //while (((Serial.read() != '\n') || (Serial.read() != '\r')) && count < 10)
  while (Serial.available()  && count < 10 ) {
    c = Serial.read();
    if (c == 'T') {
      count = 0;
    }
    else if (c >= '0' && c <= '9') {
      userTime[count] = c;
      count++;
    }
  }

  tm.Year = y2kYearToTm((userTime[0] - '0') * 10 + userTime[1] - '0');
  tm.Month = (userTime[2] - '0') * 10 + userTime[3] - '0';
  tm.Day = (userTime[4] - '0') * 10 + userTime[5] - '0';
  tm.Hour = (userTime[6] - '0') * 10 + userTime[7] - '0';
  tm.Minute = (userTime[8] - '0') * 10 + userTime[9] - '0';
  tm.Second = 0;
  externalTime_t = makeTime(tm);
  setTime(tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tm.Year);
  Teensy3Clock.set(externalTime_t);
  ss = 0;
  Serial.printf("Date/Time: %s %i, %i", monthStr(tm.Month), tm.Day, tmYearToCalendar(tm.Year));
  Serial.printf(" %02i:%02i:%02i\r\n\n", tm.Hour, tm.Minute, tm.Second);
}

// Turn on GPS and wait for fix
void startGPS() {
	
	// Power up sequence
	digitalWrite (EN_DATA, HIGH);	// enable fona power supply
	delay(100);	// power supply settling
	digitalWrite (NOT_C_ON, LOW);	// bring fona power key pin low
	delay(1100);			// power key must be low >1s
  digitalWrite (NOT_C_ON, HIGH);	// return fona power key pin
  	
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.printf("Couldn't find FONA\n\r");
    useFONA = false;
  }
  else {
    Serial.printf(F("FONA is OK\n\r"));
    Serial.printf(F("Enabling GPS\n\r"));
    fona.enableGPS(true);
    attachInterrupt(digitalPinToInterrupt(PPS_PIN), ppsInterrupt, RISING);	// enable PPS interrupt for GPS timestamping
    NVIC_SET_PRIORITY( IRQ_PORTC, 48);	//	set interrupt priority of PPS to moderately high (0 = highest, 256 = lowest)
  	
    // Wait for GPS fix
    elapsedMillis Time2Fix = 0;
    while (((fona.GPSstatus() < 2) && (receivedPPS == 0)) && Time2Fix < 300000) {	// wait for fix until there's PPS or GPS fix or 5 minutes is up
			delay(500);     // wait between queries
    }

    // Set the time
    if ((Time2Fix > 300000) && !sessionStarted) {
    	Serial.println("GPS not receiving a signal");
    	getUserTime();
    }
    else if ((Time2Fix > 300000) && sessionStarted) {
    	setTime(Teensy3Clock.get());
    }
    else {
    	Serial.printf("Time to fix: %.1f\n\r", (float) Time2Fix/1000.0);
    	retrieveGPSTime(&tm);
    }
	}
}

// get the GPS time and convert to local time
void retrieveGPSTime(tmElements_t *tm) {
  uint16_t stringLen = 120;
  uint8_t arg = 32;
  char gpsbuffer[stringLen];
  char *timeString;
  time_t localTime_t = 0;

  while (fona.GPSstatus() < 2) {
    delay(500);     // wait between queries
  }
  fona.getGPS( arg, gpsbuffer, stringLen);
  char *tok = strtok(gpsbuffer, "," );
  int aa = 0;
  while ( tok ) {
    if (aa == 2)
      timeString = tok;
    tok = strtok(NULL, ",");
    aa++;
  }

  // parse from yy/mm/dd,hh:mm:ss from GPS to time structure
  tm->Year = y2kYearToTm((timeString[2] - '0') * 10 + timeString[3] - '0');
  tm->Month = (timeString[4] - '0') * 10 + timeString[5] - '0';
  tm->Day = (timeString[6] - '0') * 10 + timeString[7] - '0';
  tm->Hour = (timeString[8] - '0') * 10 + timeString[9] - '0';
  tm->Minute = (timeString[10] - '0') * 10 + timeString[11] - '0';
  tm->Second = (timeString[12] - '0') * 10 + timeString[13] - '0';

  // convert time structure to epoch seconds and correct for the timezone
  localTime_t = canEastern.toLocal(makeTime(*tm), &tcr);
  setTime(localTime_t);
  Teensy3Clock.set(localTime_t);
  ss = 0;
  digitalClockDisplay();
}

// Sub-second updating 
void ppsInterrupt() {
	noInterrupts();	// disable interrupts
  ss = 0;         // reset subseconds counter (microseconds
  sec = now();    // update seconds variable
  receivedPPS++;  // counter for monitoring GPS status
	interrupts();		// enable interrupts
}

// data producer function
void adcDataGet() {
	noInterrupts();
	data_temp[0] = ss;
	data_temp[1] = sec;
	data_temp[2] = moon_adc.readADC();
	DataBuf.write(data_temp, 3);
	interrupts();
}

// For accurate voltage measurements the battery should be in a quasi-OC state (low current draw)
float readBattVolt() {
	float avg = 0, voltage;
	analogReadResolution(12);
	for (int ii = 1; ii < 12; ii++) {
		avg = avg + ((float) analogRead(BATT_V)  - avg)/(float)ii;
		delay(1);
	}
	voltage = avg*0.002739 - 0.0493;		// empirical conversion equation
	return voltage;
}

// Enables wake from mode switch if hardware supports it. Returns true if enabled.
// NOTE: Switch must be in 'LOW' position for test to work
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

// Core logging loop
void loggingFun(File *dataFile) {
  int data[3];
  bool stop_logging = false;
  uint32_t logStartTime;

  serialPrintTimer.reset();
  logStartTime = now();
  attachInterrupt(digitalPinToInterrupt(RDYB), adcDataGet, FALLING);
	NVIC_SET_PRIORITY( IRQ_PORTD, 64);	// set interrupt priority lower than PPS but still high
  
  // logging loop
  while ( !stop_logging ) {
    while(!DataBuf.available()) {}  // wait for buffer to have a value
    DataBuf.read(data, 3);
    dataFile->print((uint32_t) data[0]);    // save microseconds (ensure that it's not type cast to int)
    dataFile->print(',');
    dataFile->print(data[1]);    // save seconds
    dataFile->print(',');
    dataFile->println(data[2]);  // save adc value

    // Serial output every 10 seconds for monitoring
    if (serialPrintTimer.check()) {
      int* sample = DataBuf.back();

      // read adc status
      detachInterrupt(RDYB);   // disable 
      Serial.printf("%6X", moon_adc.get_status());        // ADC status register
      attachInterrupt(digitalPinToInterrupt(RDYB), adcDataGet, FALLING);

      Serial.printf(", %8i", now()-logStartTime);  // seconds since start of file
      Serial.printf(", %9i", sample[2]);           // ADC value       
      Serial.printf(", %11i\n\r", receivedPPS);     // number of PPS pulses received

      // Do some status checks
      
      if ((DataBuf.capacity() - DataBuf.size()) < 100) {
        detachInterrupt(RDYB);   // stop retreiving new values from the adc
        Serial.printf("*BUFFER NEAR CAPACITY*\n\r");
        attachInterrupt(digitalPinToInterrupt(RDYB), adcDataGet, FALLING);
      }

      if ((now() - logStartTime) >= logDuration[sequence_channel_num]) {
        detachInterrupt(RDYB);   // stop retreiving new values from the adc
        stop_logging = true;
      }
      
      receivedPPS = 0;    // reset PPS count
      serialPrintTimer.reset();
    }
  }
}

// Checks if the logger should sleep and if so it goes to sleep
void sleepCheck () {
  int who; // For snooze library
  
  // Check mode switch pin state
  if (!digitalRead(MODE)) return;		// Mode switch == FULL (V == 0V) -> no sleeping
  
  // Check if Teensy needs to sleep; return if Teensy is in awake period
  if ((hour() >= AWAKE_TIME_START_HRS) && (hour()< AWAKE_TIME_END_HRS))
    return;

  // Time is between 04:00 & 24:00
  if((hour() >= AWAKE_TIME_END_HRS)) {
    if(minute()>0) {
      sleep_period_hrs  = 24-(hour()+1)+AWAKE_TIME_START_HRS;
      sleep_period_mins = 60-minute();
      sleep_period_secs = 0;
    }
    else {
      sleep_period_hrs  = 24-hour()+AWAKE_TIME_START_HRS;
      sleep_period_mins = 0;
      sleep_period_secs = 0;
    }
  } 
  // Time is between 00:00 & 02:00
  else if (hour() < AWAKE_TIME_START_HRS) {
    if(minute()>0) {
      sleep_period_hrs  = AWAKE_TIME_START_HRS - (hour()+1);
      sleep_period_mins = 60-minute();
			sleep_period_secs = 0;
  	}
  	else {
			sleep_period_hrs  = AWAKE_TIME_START_HRS - hour();
			sleep_period_mins = 0;
			sleep_period_secs = 0;
    }
  }

	// Turn off peripherals
	Serial.println("Turning peripherals off");
	digitalWrite (EN_MOON, LOW);	// Power off moon board
	digitalWrite (EN_SENSE, LOW);	// Power off main board sensors
  if (useFONA) {
  	digitalWrite (NOT_C_ON, LOW);		// Toggle FONA power
  	delay(1000);
  	digitalWrite (NOT_C_ON, HIGH);
		delay(2500);
 		digitalWrite (EN_DATA, LOW);	// turn off fona power supply
	}
	 	
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
  who = Snooze.hibernate( config_teensy36 );
  setTime(Teensy3Clock.get()); // load RTC time onto MCU
  if (who == 35) // rtc wakeup value
  	Serial.println("Woke up through RTC");
  else if (who == MODE)  // switch wakeup value
  	Serial.println("Woke up through mode switch");
  wakeUp();	// go through additional wakeup processes
}

// Turns on peripheral power supplies/modules
void wakeUp() {

	Serial.printf("Battery Voltage: %.2f V\n\r", readBattVolt());	// Measure the battery voltage
	
  // Turn peripherals back on
  digitalWrite (EN_MOON, HIGH);		// Power on moon board
	digitalWrite (EN_SENSE, HIGH);	// Power on main board sensors
	digitalWrite (CS, HIGH);				// chip select for moon ADC SPI off
	
	// Get time
	if (useFONA) startGPS();
	
  // initialize SPI:
  SPI.begin();
  delay(100); //give power supplies and ADC time to wake up
  moon_adc.start_adc(CS, SPI_SPEED);
}

void digitalClockDisplay() {
  Serial.printf("%2i:%02i:%02i %s %i, %i %s\n\r", hour(), minute(), second(), monthStr(month()), day(), year(), tcr -> abbrev);
}

void setup() {
  char c;
  bool digitalWake = false;

  Serial.begin(115200);

  // configure pins
  pinMode (CS, OUTPUT);
  pinMode (MCU_STAT, OUTPUT);
  pinMode (EN_SENSE, OUTPUT);
  pinMode (EN_MOON, OUTPUT);
  pinMode (EN_DATA, OUTPUT);
  pinMode (NOT_C_ON, OUTPUT);
  pinMode (RDYB, INPUT_PULLUP);
  digitalWrite (EN_DATA, LOW);		// FONA power supply off
  digitalWrite (NOT_C_ON, HIGH);	// FONA off

	
  delay(500);   // wait for serial
  Serial.printf("Single Channel Logger - %s\r\n\n", FirmwareVersion);

	digitalWake = digitalWakeEnable();	// enable wake from Mode switch if compatible with hardware
	Serial.printf("Mode Switch Wakeups: %s\n\r", digitalWake ? "Yes" : "No");
	
  Serial.printf("Use GPS? (y/n)\n\r");
  while (!Serial.available()) {}
  c = Serial.read();
  if ((c == 'y') || c == 'Y') {
    useFONA = true;
    Serial.println("Starting FONA chip...");
  }
  else {
    getUserTime();
  }
	
	// Power on external modules
	wakeUp();
}

void loop() {
  char filename[20];
  
  // Testing this
  SDClass SD;
  // ^ In testing ^

  // check if this test is part of a set
  if ((sequenceNum > numTestSeqs) && !infiniteLog ) {
    sessionStarted = false;
  }
  
  // get user input for session parameters
  if (!sessionStarted) sessionSetup();

  // Display test parameters
  Serial.println("\nStarting Session");
  Serial.printf("Sequence %i of %i\n\r", sequenceNum, numTestSeqs);

  for ( sequence_channel_num = 0; sequence_channel_num < numChannels; sequence_channel_num++) {
    // Open file
    sprintf(filename, "%02i%02i%02i%02i.csv", month(), day(), hour(), minute());  // generate filename
    File dataFile;	// create file
    while (!(dataFile = SD.open(filename, FILE_WRITE))) {	// try to open file with write permission
      //Serial.println("There was a problem with the SD card.");  
      initializeSD( &SD);
    	sprintf(filename, "%02i%02i%02i%02i.csv", month(), day(), hour(), minute());  // generate filename
    }
    Serial.printf("File: %s\n\r", filename);
    Serial.printf("Channel: %i\n\r", channel[sequence_channel_num]);
    Serial.printf("Duration: %i minutes\n\n\r", logDuration[sequence_channel_num]/60);

    // set channel and sample rate
    moon_adc.select_channel(channel[sequence_channel_num]);
    moon_adc.set_sample_rate(11);

    // print headers
    Serial.printf("Status, Time (s), %s [LSB], PPS received\n\r", sensorName[channel[sequence_channel_num]]);
    dataFile.printf("Microseconds, Seconds, %s [LSB]\n\r", sensorName[channel[sequence_channel_num]]);
  
    // start logging
    loggingFun(&dataFile);

    // close down logging
    dataFile.close();   // close file
    DataBuf.clear();    // clear the data buffer for the next file
    Serial.printf("Finished logging to %s\n\n\r", filename);
  }
  
  Serial.printf("Finished logging sequence %i of %i\n\n\r", sequenceNum, numTestSeqs);
  sequenceNum++;          // increment test number for multi-file sessions

  // Go to sleep now (maybe, if the time is right)
  sleepCheck();
  digitalClockDisplay();
}
