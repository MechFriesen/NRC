# NRC
Buoyancy testing for prototype 1: Floating Pavilion
 
## How to use the .hex files.
Step 1. Download the Teensy Loader Application (https://www.pjrc.com/teensy/loader.html)  
Step 2. Open the .hex file that you want to run. (File->Open HEX File)  
Step 3. Plug in main board to computer and follow Teensy Loader prompts to load program  
Step 4. It's logging to an SD card (if there's one in the Teensy). You can also monitor the Teensy with a serial monitor.   

Be sure to record meta-data like which moon board (eg. R2A) is being used. This is needed in order to determine the conversion from unitless ADC integers to floats with proper units.

## Teensy Setup
https://www.pjrc.com/teensy/tutorial.html

## Connections

| LSM9DS1 | Teensy | (Breadboard) | SPI Name |
|---------|--------|--------------|----------|
| VIN     | Vin    | +            |          |
| GND     | GND    | -            |          |
| SCL     | 13     | 20 (R)       | SCK0     |
| SDA     | 11     | 19 (L)       | MOSI0    |
| CSAG    | 15     | 18 (R)       | CS0      |
| SDOAG   | 12     | 20 (L)       | MISO0    |
| **Wind Speed** |
| black | 2 |
| **Wind Direction** |
| green | 20 |

## Libraries used
* Hologram backend https://github.com/benstr/hologram-SIMCOM 
* GPS for timestamps https://github.com/adafruit/Adafruit_FONA
  * please disable the verbose output (comment out line 31 in /libraries/Adafruit_FONA/includes/FONAConfig.h) prior to compiling binary for deployment
* Circular array buffer https://github.com/tonton81/Circular_Buffer

## Sensor Specifications and conversion equations

### Anemometer
[Davis 6410](https://www.davisinstruments.com/product/anemometer-for-vantage-pro2-vantage-pro/)  
[Datasheet](https://www.davisinstruments.com/product_documents/weather/spec_sheets/6410_SS.pdf)
### Temperature, main board (TMP20)
Conversion equations
* Parabolic (-55 to 125˚C, ±2.5˚C): T [˚C] = -1481.96 + sqrt(2.1962 * 10^6 + (1.8639 - V)/(3.88 * 10^-6)) 
* Linear (-40 to 85˚C, ±3.15˚C): T [˚C]= (V - 1.8583)/-0.01167
### Accelerometer, main board (ADXL337)
Conversion equation:
  a [g] = (V - 1.65)/0.33
