#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

// // i2c
// Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
//
// #define LSM9DS1_SCK 13
// #define LSM9DS1_MISO 12
// #define LSM9DS1_MOSI 11
#define LSM9DS1_XGCS 15
//#define LSM9DS1_MCS 5
// You can also use software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS);
// Or hardware SPI! In this case, only CS pins are passed in
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, 0);

const uint16_t arraySize = 2048;
uint16_t dataCount_external = 0;
uint16_t counter = 0;
 // array_ax[arraySize], array_ay[arraySize], array_az[arraySize], array_gx[arraySize], array_gy[arraySize], array_gz[arraySize];
float array_ax[arraySize], array_ay[arraySize], array_az[arraySize], array_gx[arraySize], array_gy[arraySize], array_gz[arraySize];


void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}


void setup()
{
  Serial.begin(115200);

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }

  Serial.println("LSM9DS1 data read demo");

  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.beginAG())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();
}

void loop()
{
  Serial.clear();
  
  while(!Serial.read()){
  ;
  }

  lsm.read();  /* ask it to read in the data */

  /* Get a new sensor event */
  sensors_event_t a, g, temp;

  lsm.getEvent(&a, &g, &temp);
/*
  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

//  Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
  //Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
  //Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");

  Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
  Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
  Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" dps");

  Serial.println();
*/

  array_ax[dataCount_external] = a.acceleration.x;
  array_ay[dataCount_external] = a.acceleration.y;
  array_az[dataCount_external] = a.acceleration.z;
  array_gx[dataCount_external] = g.gyro.x;
  array_gy[dataCount_external] = g.gyro.y;
  array_gz[dataCount_external] = g.gyro.z;
  Serial.println("Accel X\t\tAccel Y\t\tAccel Z\t\tGyro X\t\tGyro Y\t\tGyro Z");
  Serial.printf("%f\t%f\t%f\t%f\t%f\t%f\n", array_ax[dataCount_external], array_ay[dataCount_external], array_az[dataCount_external], array_gx[dataCount_external], array_gy[dataCount_external], array_gz[dataCount_external]);
  dataCount_external++;
  delay(200);
}
