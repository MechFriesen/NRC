/*
	NRC_datalogger
	
	Firmware for use with the WatHydra hardware
	Logger wakes from a number of inputs and records data from sensors that is logged to an SD card. There is also GPS for accurate timing.
	
	CODE by Dirk Friesen with contributions from Ilia Baranov and Sunaal Mathew
	October 2018
	
 	This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License. To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/ or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
*/

#include <ADC.h>
#include "src/SDIC_FONA/Adafruit_FONA.h"
#include "src/SD/SD.h"
#include <SPI.h>
#include <Metro.h>
#include <TimeLib.h>
#include <Timezone.h>
#include <IntervalTimer.h>
#include <SoftwareSerial.h>
#include <Snooze.h>

// Sleep related macros
#define AWAKE_TIME_START_HRS 1	// Local
#define AWAKE_TIME_END_HRS   7	// Local

// Firmware version string
static char FirmwareVersion[] = "v2.1.6 - beta - Wake 1:00-7:00 Local Time";

// Load sleep drivers
SnoozeAlarm  alarm; // Using RTC
SnoozeDigital digital;	// Wake from pin interrupt
SnoozeBlock config_teensy36(alarm, digital); // add alarm driver to main Snoozeblock 
int sleep_period_hrs = 0; 
int sleep_period_mins = 0; 
int sleep_period_secs = 0;

// FONA configuration
const int	RX_PIN  = 10;
const int	TX_PIN  = 9;
const int	FONA_RST = 25;
const int	PPS_PIN = 23; // 1PPS from SIM808 module
SoftwareSerial fonaSS = SoftwareSerial(TX_PIN, RX_PIN);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

// Timezone and Time change rules
tmElements_t tm;
TimeChangeRule *tcr;	// pointer to time change rule for printing time zone abbreviation
TimeChangeRule canEDT = {"EDT", Second, Sun, Mar, 2, -240};  //UTC - 4 hours
TimeChangeRule canEST = {"EST", First, Sun, Nov, 2, -300};   //UTC - 5 hours
Timezone canEastern(canEDT, canEST);

// SPI initialization variables
const long SPI_SPEED = 2000000;
const int CS = 15;

// main board sensors
const int Temp = 14;	// requires special analog input settings to not spoil data
const int A_x = 18;
const int A_y = 17;
const int A_z = 16;
const int BATT_V = A20;			// battery voltage sense
const char sensorName[6][10] = {"Passive H", "Active H", "Pressure", "Temp Moon", "Ax Moon", "Ay Moon"};

// Main board pins
const int MCU_STAT = 22;    // Status LED
const int EN_SENSE = 21;    // Main board sensor power
const int EN_DATA = 19;     // Enable 3V6 regulator
const int NOT_C_ON = 24;    // SIM808 power
const int SD_CS = BUILTIN_SDCARD; // SD card chip select
const int MODE = 6;         // mode switch pin

// Timers
Metro serialPrintTimer = Metro(5000);	// triggers serial print every 5 s
IntervalTimer adcTrigger;				// triggers ADC data retrieval

// Onboard ADC
ADC *adc = new ADC();
ADC::Sync_result ADC_vals;

// Global variables
const uint16_t arraySize = 2048;
uint16_t numTestSeqs = 0, sequenceNum = 1, dataCount = 0, xData[arraySize], zData[arraySize];
uint32_t logDuration, time[arraySize];	// duration of logging in seconds, time of sample [us]
float Batt_volt = 0;
bool useFONA = false, sessionStarted = false, infiniteLog = false, externalSensors = false;
bool ledState = false;
SDClass SD;

// Get user input for channels to log and duration
void sessionSetup() {
  
	Serial.println("\nNew Session\n");

	// Is there an external sensor
	Serial.clear();
	Serial.printf("External sensor? (y/n)\r\n");
	while (!Serial.available()){}	// pause for user input
	if ((Serial.read() == 'y') || (Serial.read() == 'Y'))
		externalSensors = true;
	
  	// get number of samples to log
  	Serial.clear();
	Serial.printf("\nHow many samples would you like to run? (enter '0' for continuous logging)\r\n");
	while (!Serial.available()) {}	// pause for user input
	numTestSeqs = Serial.parseInt();	// load user input

  	if (numTestSeqs == 0) {
   		infiniteLog = true;
    	Serial.println("Continuous logging selected\n");
  	} else {
    	Serial.printf("%i\n\r", numTestSeqs);
  	}
  	sequenceNum = 1;
	sessionStarted = true;
}

// Open SD file system
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

void SD_dateTime(uint16_t* date, uint16_t* time) {	// function that SdFat calls to get time
  *date = FAT_DATE(year(), month(), day());		// return date using FAT_DATE macro to format fields
  *time = FAT_TIME(hour(), minute(), second());	// return time using FAT_TIME macro to format fields
}

// For accurate voltage measurements the battery should be in a quasi-OC state (low current draw)
float readBattVolt() {
	float avg = 0, voltage;
	for (int ii = 1; ii < 12; ii++) {	// average 12 samples
		avg = avg + ((float) analogRead(BATT_V)  - avg)/(float)ii;
		delay(1);
	}
	voltage = avg*0.002739 - 0.0493;		// empirical conversion equation (assumes 12-bit)
	return voltage;
}

// Core logging loop
void loggingFun(File *dataFile) {
	bool stop_logging = false;
	uint32_t logStartTime;
	uint16_t sample_rate = 50;	// samples/second
	
	// setup ADC (probably going to be moved to separate function)
	adc->setReference(ADC_REFERENCE::REF_3V3, ADC_0);
	adc->setAveraging(2); // set number of averages
	adc->setResolution(12); // set bits of resolution
	adc->setAveraging(2, ADC_1); // set number of averages
    adc->setResolution(12, ADC_1); // set bits of resolution
	
	// apparently these are necessary 
	adc->enableCompare(1.0/3.3*adc->getMaxValue(ADC_0), 0, ADC_0); // measurement will be ready if value < 1.0V
    adc->enableCompareRange(1.0*adc->getMaxValue(ADC_0)/3.3, 2.0*adc->getMaxValue(ADC_0)/3.3, 0, 1, ADC_0); // ready if value lies out of [1.0,2.0] V
	adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // change the conversion speed
	adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED); // change the sampling speed
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED, ADC_1); // change the conversion speed
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED, ADC_1); // change the sampling speed
	
	Serial.printf("Sample Rate [Hz]: %i\n", sample_rate);

	serialPrintTimer.reset();
	logStartTime = now();
	adc->startSynchronizedContinuous(A_x, A_z);		// continuously samples both input pin simultaneously
	adcTrigger.begin(adc_data_retrieve, 1000000/sample_rate);

	// logging loop
	while ( !stop_logging ) {
		if ((now() - logStartTime) >= 10)
			stop_logging = true;
		else if (dataCount > arraySize)
			stop_logging = true;
	}
	adcTrigger.end();	// stop retrieving values
	adc->stopSynchronizedContinuous();	// stop ADC
	
	// Print and save to SD
	Serial.println("ii\ttime [us]\tA_x\tA_y");
	for (int ii = 0; ii < arraySize; ii++) {
		Serial.printf("%i\t%i\t\t%i\t%i\n", ii, time[ii], xData[ii], zData[ii]);
		dataFile->printf("%i\t\t%i\t%i\n", time[ii], xData[ii], zData[ii]);
	}
}

// Checks if the logger should sleep and if so it goes to sleep
void sleepCheck () {// 
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
	digitalWrite (EN_SENSE, HIGH);	// Power on main board sensors
	digitalWrite (CS, HIGH);				// chip select for moon ADC SPI off
	
	// Get time
	if (useFONA) startFONA();
	
	// initialize SPI:
	SPI.begin();
	delay(100); //give power supplies and ADC time to wake up
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

// get the local time and date from serial interface
void getUserTime() {
  time_t externalTime_t = 0;
  tmElements_t tm;
  uint8_t count = 0;
  char c;
  char userTime[18];

  	// request user input time via serial connection
  	Serial.clear();
  	Serial.printf("Enter time (yy/MM/dd,hh:mm:ss)\n\r");
  	while (!Serial.available()) {}    // pause for user input
  	do  {
    	c = Serial.read();
		userTime[count] = c;
		count++;
	} while ((c != '\n') && (c != '\r'));	// newline or carriage return character received
	parseTime(userTime);
	digitalClockDisplay();	// print it out for the user to see
}

// Turn on FONA and get Network time
void startFONA() {
	char timeBuf[21];	// holds time string returned from FONA
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
	} else {
		Serial.print(fona.enableNetworkTimeSync(true));
		fona.getTime(timeBuf, 25);
		parseTime(timeBuf);
	}
	
}

// convert user time input or network time into Teensy time struct
void parseTime(char *TimeStr) {
	char * tok;
	uint8_t field = 0;	// which tm field we're on
	tok = strtok(TimeStr, " /,:+-");
	while (tok != NULL)  {
		switch(field) {
			case 0 : tm.Year = atoi(tok);
						break;
			case 1 : tm.Month = atoi(tok);
						break;
			case 2 : tm.Day = atoi(tok);
						break;
			case 3 : tm.Hour = atoi(tok);
						break;
			case 4 : tm.Minute = atoi(tok);
						break;
			case 5 : tm.Second = atoi(tok);
						break;
		}
		field++;
		tok = strtok(NULL, " /,:+-");
	}
	
	setTime(tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tm.Year);
	Teensy3Clock.set(makeTime(tm));
// 	digitalClockDisplay();
}

void digitalClockDisplay() {
  Serial.printf("%2i:%02i:%02i %s %i, %i %s\n\r", hour(), minute(), second(), monthStr(month()), day(), year(), tcr -> abbrev);
}

void setup() {
	char c;
	bool digitalWake = false;

	Serial.begin(128000);

	// configure pins
	pinMode (CS, OUTPUT);
	pinMode (MCU_STAT, OUTPUT);
	pinMode (EN_SENSE, OUTPUT);
	pinMode (EN_DATA, OUTPUT);
	pinMode (NOT_C_ON, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	
	digitalWrite (EN_DATA, LOW);		// FONA power supply off
	digitalWrite (NOT_C_ON, HIGH);	// FONA off

	delay(500);   // wait for serial
	Serial.printf("NRC Logger - %s\r\n\n", FirmwareVersion);

	digitalWake = digitalWakeEnable();	// enable wake from Mode switch if compatible with hardware
	Serial.printf("Mode Switch Wakeups: %s\n\r", digitalWake ? "Yes" : "No");
	
	Serial.printf("Use FONA? (y/n)\n\r");
	while (!Serial.available()) {}
	c = Serial.read();
	if ((c == 'y') || c == 'Y') {
		useFONA = true;
		Serial.println("Starting FONA chip...");
	} else {
		getUserTime();
	}
	
	wakeUp();	// Power on external modules
}

void loop() {
	char filename[20];	
	initializeSD();

	// get user input for session parameters
	if (!sessionStarted) sessionSetup();

	// Display test parameters
	Serial.println("\nStarting Session");
	Serial.printf("Sequence %i of %i\n\r", sequenceNum, numTestSeqs);

	// Open file
	sprintf(filename, "%02i%02i%02i%02i.csv", month(), day(), hour(), minute());  // generate filename
	File dataFile;	// create file
	while (!(dataFile = SD.open(filename, FILE_WRITE))) {	// try to open file with write permission
		Serial.println("There was a problem with the SD card.");  
		sprintf(filename, "%02i%02i%02i%02i.csv", month(), day(), hour(), minute());  // generate filename
	}
	Serial.printf("File: %s\n\r", filename);
	//Serial.printf("Channel: %i\n\r", channel[sequence_channel_num]);
	//Serial.printf("Duration: %i minutes\n\n\r", logDuration[sequence_channel_num]/60);

	// print headers (TODO: Update for NRC project)
	Serial.printf("Status, Time (s), %s [LSB], PPS received\n\r", "Acceleration");
	dataFile.printf("Microseconds, Seconds, %s [LSB]\n\r", "Acceleration");

	// start logging
	loggingFun(&dataFile);

	// close down logging
	dataFile.close();   // close file
	Serial.printf("Finished logging to %s\n\n\r", filename);

	Serial.printf("Finished logging sequence %i of %i\n\n\r", sequenceNum, numTestSeqs);
	sequenceNum++;          // increment test number for multi-file sessions

	// Go to sleep now (maybe, if the time is right)
	sleepCheck();
	digitalClockDisplay();
}

// This function is triggered by the 'adcTrigger' timer
// It reads the data into an array 
void adc_data_retrieve(void) {
	ADC_vals = adc->readSynchronizedContinuous();	// retrieve data from ADC register
	time[dataCount] = micros();						// log time (might not be necessary but it's nice for debugging)
	xData[dataCount] = ADC_vals.result_adc0;		// adc0 = A_x = pin 18
	zData[dataCount] = ADC_vals.result_adc1;		// adc1 = A_z = pin 16
	dataCount++;									// increment index
	
	// blink for debugging
	ledState = !ledState;						
	digitalWriteFast(LED_BUILTIN, ledState);
}
