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
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <math.h> //wind sketch
//#include <Adafruit_Sensor.h>

// Sleep related macros
#define AWAKE_TIME_START_HRS 1	// Local
#define AWAKE_TIME_END_HRS   7	// Local

// External board
#define LSM9DS1_XGCS 15

// Wind Sketch taken from cactus.io
#define PinWindSpeed (2) // location of anemometer sensor
#define PinWindDrctn (20) // pin of the wind vane sensor
#define VaneOffset 0;  // define the anemomter offset from magnetic north

// Firmware version string
static char FirmwareVersion[] = "NRC datalogger edits - v0.1.5";

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

// lsm config
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, 0); //from example sketch

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
Metro AccelTimer = Metro(200); // triggers accelerometer/gyro data reterival
Metro WindDirectionTimer = Metro(2500);
Metro WindSpeedTimer = Metro(2500);

// Anemomter volatile
volatile unsigned long Rotations; // cup rotation counter used in interrupt routine
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in interrupt routine

// Onboard ADC
ADC *adc = new ADC();
ADC::Sync_result ADC_vals;

// Global variables
const uint16_t arraySize_onboard = 494, arraySize_external = 49;
int VaneValue, Direction, CalDirection; //wind sketch variables
uint16_t numTestSeqs = 0, sequenceNum = 1, wind_time = 5000, external_period = 200, dataCount = 0, dataCount_external = 0, xData[arraySize_onboard], yData[arraySize_onboard];
uint32_t logDuration, time_onboard[arraySize_onboard], time_external[arraySize_external];	// duration of logging in seconds, time of sample [us]
float WindSpeed, Batt_volt = 0, array_ax[arraySize_external], array_ay[arraySize_external], array_az[arraySize_external], array_gx[arraySize_external], array_gy[arraySize_external], array_gz[arraySize_external];
bool useFONA = false, sessionStarted = false, infiniteLog = false, externalSensors = false;
bool ledState = false;
SDClass SD;

// Functions
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
	Serial.print(F("Initializing SD card..."));
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

// Anemomter functions
	// interrupt calls to increment the rotation count
void isr_rotation() {
	Rotations++;
}

// Convert MPH to Knots
float getKnots(float speed) {
return speed * 0.868976;
}

//Get Wind Direction
void getWindDirection() {
VaneValue = analogRead(PinWindDrctn);
// Serial.printf("Vane value = %i\n", VaneValue);
Direction = map(VaneValue, 0, adc->getMaxValue(ADC_0), 0, 359);
// Serial.printf("Direction = %i\n", Direction);
CalDirection = Direction + VaneOffset;
// Serial.printf("CalDirection = %i\n", CalDirection);
}

// This function is triggered by the 'adcTrigger' timer
// It reads the data into an array
void adc_data_retrieve(void) {
	ADC_vals = adc->readSynchronizedContinuous();	// retrieve data from ADC register
	time_onboard[dataCount] = micros();						// log time (might not be necessary but it's nice for debugging)
	xData[dataCount] = ADC_vals.result_adc0;		// adc0 = A_x = pin 18
	// zData[dataCount] = ADC_vals.result_adc1;		// adc1 = A_z = pin 16
	yData[dataCount] = ADC_vals.result_adc1;		// adc1 = A_y = pin 17
	dataCount++;									// increment index

	// blink for debugging
	ledState = !ledState;
	digitalWriteFast(LED_BUILTIN, ledState);
}

void setupADC() {
	uint16_t sample_rate = 50;	// samples/second

	// setup ADC
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

		adcTrigger.priority(0);
		adc->startSynchronizedContinuous(A_x, A_y);		// continuously samples both input pin simultaneously
		adcTrigger.begin(adc_data_retrieve, 1000000/sample_rate);


}

// example sketch from LSM9DS1 library; prints the acceleration and gyroscope data
void setupSensor() {
		// 1.) Set the accelerometer range
		lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
		//lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
		//lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
		//lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

		// 2.) Setup the gyroscope
		lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
		//lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
		//lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

// Checks if the logger should sleep and if so it goes to sleep
void sleepCheck () {//
  int who; // For snooze library

  // Check if Teensy needs to sleep; return if Teensy is in awake period
  if ((hour() >= AWAKE_TIME_START_HRS) && (hour()< AWAKE_TIME_END_HRS))
    return;
		//Serial.println("running");
		// Serial.printf("%i\t%i\t%i\t%i\n", AWAKE_TIME_START_HRS, AWAKE_TIME_END_HRS, hour(), minute());// 1 7

		// Check mode switch pin state
	  // if (!digitalRead(MODE)) return;		// Mode switch == FULL (V == 0V) -> no sleeping
		// Serial.println("off");

  // Time is between 04:00 & 24:00
  if((hour() >= AWAKE_TIME_END_HRS)) {
		if (minute() % 15 == 0) {
			// Serial.println("a");
			sleep_period_hrs = 0;
			sleep_period_mins = 1;
			sleep_period_secs = 0;
		}
		else {
			// Serial.println("b");
			sleep_period_hrs = 0;
			sleep_period_mins = 0;
			sleep_period_secs = 15;
		}

  }
  // Time is between 00:00 & 02:00
  else if (hour() < AWAKE_TIME_START_HRS) {
		if (minute() % 15 == 0) {
			// Serial.println("c");
			sleep_period_hrs = 0;
			sleep_period_mins = 1;
			sleep_period_secs = 0;
		}
		else {
			// Serial.println("d");
			sleep_period_hrs = 0;
			sleep_period_mins = 0;
			sleep_period_secs = 15;
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
	// }

	// Logging sleep period
 	Serial.println("OK. Going to sleep now...");
 	digitalClockDisplay();
 	Serial.printf("Battery Voltage: %.2f V\n\r", readBattVolt());
 	Serial.printf("Sleep Period HRS = %i\n\r", sleep_period_hrs);
 	Serial.printf("Sleep Period MINS = %i\n\r", sleep_period_mins);
 	Serial.printf("Sleep Period SECS = %i\n\r", sleep_period_secs);

	delay(1000);

	// Set RTC alarm wake up in (hours, minutes, seconds).
	// Currently the code runs so that it hits the "d" loop and then goes to the "a" loop
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
}

// Turns on peripheral power supplies/modules
void wakeUp() {

	Serial.printf("Battery Voltage: %.2f V\n\r", readBattVolt());	// Measure the battery voltage

	// Turn peripherals back on
	digitalWrite (EN_SENSE, HIGH);	// Power on main board sensors
	digitalWrite (CS, HIGH);				// chip select for moon ADC SPI off

	// Get time
	if (useFONA)
	startFONA();

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
		int rssi = 0;
		while (fona.getRSSI() < 5) {
			Serial.printf("RSSI: %i\n", rssi);
			// delay(50);
		}
		// delay(500);	// shitty hack
		Serial.print(fona.enableNetworkTimeSync(true));
		// if (!fona.enableNetworkTimeSync(true)) {
		// 	Serial.println(F("Failed to enable"));
		// }
		// delay(500); // shitty hack
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

// Displays Date & Time
void digitalClockDisplay() {
  Serial.printf("%2i:%02i:%02i %s %i, %i %s\n\r", hour(), minute(), second(), monthStr(month()), day(), year(), tcr -> abbrev);
}

// Core logging loop
void loggingFun() {
	bool stop_logging = false;
	uint32_t logStartTime;
	uint16_t sample_rate = 50;	// samples/second


	Serial.printf("Sample Rate [Hz]: %i\n", sample_rate);

	setupSensor();

	// logs at the period of the external sensor (Adafruit LSM9DS1)
	AccelTimer.interval(external_period);
	AccelTimer.reset();

	//Timers for wind speed and dirction; logs in the interval wind_time
	WindDirectionTimer.interval(wind_time);
	WindDirectionTimer.reset();

	WindSpeedTimer.interval(wind_time);
	WindSpeedTimer.reset();


	logStartTime = now(); //current time

	setupADC();

	adcTrigger.priority(0);
	adc->startSynchronizedContinuous(A_x, A_y);		// continuously samples both input pin simultaneously
	adcTrigger.begin(adc_data_retrieve, 1000000/sample_rate);

	//main board and external sensor
	dataCount = 0;
	dataCount_external = 0;

	// Anemomter setup
	VaneValue = 0; //Vane Value
	Rotations = 0; // Set Rotations to 0 ready for calculations
	Direction = 0; // Set to 0
	CalDirection = 0;

	pinMode(PinWindSpeed, INPUT_PULLUP); // sets the digital pin as output
	attachInterrupt(digitalPinToInterrupt(PinWindSpeed), isr_rotation, FALLING); // the wind interrupt here

	// logging loop
	while ( !stop_logging ) {
		//external sensor
		sensors_event_t a, g, temp;
		if (AccelTimer.check()) {
		time_external[dataCount_external] = micros();
		lsm.getEvent(&a, &g, &temp);
		array_ax[dataCount_external] = a.acceleration.x;
		array_ay[dataCount_external] = a.acceleration.y;
		array_az[dataCount_external] = a.acceleration.z;
		array_gx[dataCount_external] = g.gyro.x;
		array_gy[dataCount_external] = g.gyro.y;
		array_gz[dataCount_external] = g.gyro.z;
		dataCount_external++;
	}

		//Wind Direction
		if (WindDirectionTimer.check()) {
			getWindDirection();
			// add the adc trigger so analogRead from wind data doesn't affect adc
			adcTrigger.priority(0);
			adc->startSynchronizedContinuous(A_x, A_y);		// continuously samples both input pin simultaneously
		}
		//Wind Speed
		if(WindSpeedTimer.check()) {
			// convert to mp/h using the formula V=P(2.25/T) T = wind_time (in seconds)
			// V = P(2.25/5) = P * 0.45
			WindSpeed = Rotations * 0.45;
			// Serial.printf("Wind Speed = %i\n", WindSpeed);
			Rotations = 0; //Reset count for next sample
			adcTrigger.priority(0);
			adc->startSynchronizedContinuous(A_x, A_y);		// continuously samples both input pin simultaneously
			}

			//stop logging
		if ((now() - logStartTime) >= 10)
			stop_logging = true;
		else if (dataCount > arraySize_onboard)
			stop_logging = true;
	}
	adcTrigger.end();	// stop retrieving values
	adc->stopSynchronizedContinuous();	// stop ADC



	// Print and save to SD--> moved to loop()
}

void setup() {
	char c;
	bool digitalWake = false;

	Serial.begin(9600);

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
	char filename_on[20];
	char filename_ex[20];
	char filename_w[20];

	// check if this test is part of a set
	if ((sequenceNum > numTestSeqs) && !infiniteLog ) {
		sessionStarted = false;
	}

	initializeSD();

	// get user input for session parameters
	// if (!sessionStarted) sessionSetup();

	// Display test parameters
	Serial.println("\nStarting Session");
	Serial.printf("Sequence %i of %i\n\r", sequenceNum, numTestSeqs);


	// Open file
	/*
	sprintf(filename, "%02i%02i%02i%02i.csv", month(), day(), hour(), minute());  // generate filename
	File dataFile;	// create file
	while (!(dataFile = SD.open(filename, FILE_WRITE))) {	// try to open file with write permission
		Serial.println("There was a problem with the SD card.");
		sprintf(filename, "%02i%02i%02i%02i.csv", month(), day(), hour(), minute());  // generate filename
	}
	*/
	// create files
	sprintf(filename_on, "%02i%02i%02i%s.csv", month(), day(), hour(), "O");  // generate onboard sensor filename
	sprintf(filename_ex, "%02i%02i%02i%s.csv", month(), day(), hour(), "E");  // generate external sensor filename
	sprintf(filename_w, "%02i%02i%02i%s.csv", month(), day(), hour(), "W");	//generate anemometer filename
	// print file names on serial monitor
	Serial.printf("File: %s\n\r", filename_on);
	Serial.printf("File: %s\n\r", filename_ex);
	Serial.printf("File: %s\n\r", filename_w);

	// start logging
	loggingFun();


		// print on serial monitor and save onboard acceleration to SD
	File dataFile_onboard;
	dataFile_onboard.println("time [us],A_x,A_y");

	dataFile_onboard = SD.open(filename_on, FILE_WRITE);
	Serial.println("ii\ttime [us]\tA_x\tA_y");

	for (int ii = 0; ii < arraySize_onboard; ii++) {
		Serial.printf("%i\t%i\t\t%i\t%i\n", ii, time_onboard[ii], xData[ii], yData[ii]);
		dataFile_onboard.printf("%i,%i,%i\n", time_onboard[ii], xData[ii], yData[ii]);
	}
		// close down logging
	dataFile_onboard.close();

	// print on serial monitor and save external accel to SD
	File dataFile_external;
	dataFile_external.println("time [us], Accel_X (m/s2), Accel_Y (m/s2), Accel_Z (m/s2), Gyro_X (degrees/s), Gyro_Y (degrees/s), Gyro_Z (degrees/s)");

	dataFile_external = SD.open(filename_ex, FILE_WRITE);
  Serial.println("ii\ttime [us]\tAccel_X (m/s2)\tAccel_Y (m/s2)\tAccel_Z (m/s2)\tGyro_X (degrees/s)\tGyro_Y (m/s2)\tGyro_Z ");

	for (int ii = 0; ii < arraySize_external; ii++) {
		Serial.printf("%i\t%i\t%f\t%f\t%f\t%f\t%f\t%f\n", ii, time_external[ii],  array_ax[ii], array_ay[ii], array_az[ii], array_gx[ii], array_gy[ii], array_gz[ii]);
		dataFile_external.printf("%i,%f,%f,%f,%f,%f,%f\n", time_external[ii], array_ax[ii], array_ay[ii], array_az[ii], array_gx[ii], array_gy[ii], array_gz[ii]);
	}
	dataFile_external.close(); // close file

	//wind sketch
	File dataFile_wind;
	dataFile_wind.println("Time, Speed (MPH),Knots,Direction");

	dataFile_wind = SD.open(filename_w, FILE_WRITE);
	Serial.println("Time\tSpeed (MPH)\tKnots\tDirection");

	Serial.printf("%02i%02i%02i%02i%02i", month(), day(), hour(), minute(), second()); Serial.print("\t\t");
	Serial.print(WindSpeed); Serial.print("\t\t"); Serial.print(getKnots(WindSpeed)); Serial.print("\t");
	Serial.print(CalDirection);
	dataFile_wind.printf("%02i%02i%02i%02i%02i", month(), day(), hour(), minute(), second()); dataFile_wind.print("\t\t");
	dataFile_wind.print(WindSpeed); dataFile_wind.print("\t\t"); dataFile_wind.print(getKnots(WindSpeed)); dataFile_wind.print("\t");
	dataFile_wind.printf("%i\t", CalDirection);

	if(CalDirection < 22) {
		Serial.print(" N\n");
		dataFile_wind.printf(" N\n");
		}
	else if (CalDirection < 67) {
		Serial.print(" NE\n");
		dataFile_wind.printf(" NE\n");
		}
	else if (CalDirection < 112) {
		Serial.print(" E\n");
		dataFile_wind.printf(" E\n");
		}
	else if (CalDirection < 157) {
		Serial.print(" SE\n");
		dataFile_wind.printf(" SE\n");
		}
	else if (CalDirection < 212) {
		Serial.print(" S\n");
		dataFile_wind.printf(" S\n");
		}
	else if (CalDirection < 247) {
		Serial.print(" SW\n");
		dataFile_wind.printf(" SW\n");
		}
	else if (CalDirection < 292) {
		Serial.print(" W\n");
		dataFile_wind.printf(" W\n");
		}
	else if (CalDirection < 337) {
		Serial.print(" NW\n");
		dataFile_wind.printf(" NW\n");
		}
	else {
		Serial.print(" N\n");
		dataFile_wind.printf(" N\n");
		}

	dataFile_wind.close();

	Serial.printf("Finished logging to %s, %s and %s\n\n\r", filename_on, filename_ex, filename_w);

	Serial.printf("Finished logging sequence %i of %i\n\n\r", sequenceNum, numTestSeqs);
	sequenceNum++;          // increment test number for multi-file sessions

	// Go to sleep now (maybe, if the time is right)
	sleepCheck();

	digitalClockDisplay();
}
