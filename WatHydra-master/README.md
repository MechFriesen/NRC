# WatHydra
Logger program for a Teensy 3.6 gathering data from an off-board ADC and storing to an SD card. Additionaly a GPS and cellular module will be connected so that accurate time stamps can be acquired and data can be uploaded to the cloud.

## Logging Modes
Below are rough definitions of the two logging modes. These have both been modified throughout the design process but their the jump-off point.
### Manual
This is the first mode that will be developed. When powered on the device will try and set it's clock from GPS or a connected host computer. It will then start logging data to a file on an SD card. New files will be created at set intervals (perhaps 10 minutes). The channels to be logged include hydrophone, pressure, and one axis of accelerometer at 2.5 kHz. These may be sampled at 10 kHz and downsampled in software to provide better anti-aliasing (hardware anti-aliasing filter at 10 kHz). Temperature will be logged every minute. 
In addition to logging to the SD card, every 2 minutes a 10 second standard deviation of the hydrophone signal will be uploaded to Hologram. In addition, a time stamp, battery level, and temperature data will be uploaded.
### Auto
Between 2:00-4:00 AM hydrophone, pressure, and accelerometer (1-axis) are logged to the SD card at 2.5 kHz for 5 minutes every 30 minutes. Temperature is logged every minute while it's awake. A 10 second standard deviation of the hydrophone signal will be uploaded to Hologram along with time stamp, battery level, and temperature data.

## How to use the .hex files.
Step 1. Download the Teensy Loader Application (https://www.pjrc.com/teensy/loader.html)  
Step 2. Open the .hex file that you want to run. (File->Open HEX File)  
Step 3. Plug in main board to computer and follow Teensy Loader prompts to load program  
Step 4. It's logging to an SD card (if there's one in the Teensy). You can also monitor the Teensy with a serial monitor.   

Be sure to record meta-data like which moon board (eg. R2A) is being used. This is needed in order to determine the conversion from unitless ADC integers to floats with proper units.

## Teensy Setup
https://www.pjrc.com/teensy/tutorial.html

## Libraries used
* Hologram backend https://github.com/benstr/hologram-SIMCOM 
* GPS for timestamps https://github.com/adafruit/Adafruit_FONA
  * please disable the verbose output (comment out line 31 in /libraries/Adafruit_FONA/includes/FONAConfig.h) prior to compiling binary for deployment
* Circular array buffer https://github.com/tonton81/Circular_Buffer

### Roadmap
* Manual Mode
  - [ ] GPS timestamp
    - [x] update Teensy clock from cell network
    - [ ] option to set time using serial monitor
    - [ ] \(on hold) subsecond using PPS
  - [ ] ADC communication
    - [ ] send SPI message
    - [ ] send the right SPI message
  - [ ] Hologram communication
  - [ ] Physical switch between manual and auto logging modes
  - [ ] Battery voltage measurement
* detailed power consumption testing with -> https://www.eevblog.com/projects/ucurrent/

## Sensor Specifications and conversion equations
### Temperature, main board (TMP20)
Conversion equations
* Parabolic (-55 to 125˚C, ±2.5˚C): T [˚C] = -1481.96 + sqrt(2.1962 * 10^6 + (1.8639 - V)/(3.88 * 10^-6)) 
* Linear (-40 to 85˚C, ±3.15˚C): T [˚C]= (V - 1.8583)/-0.01167
### Accelerometer, main board (ADXL337)
Conversion equation:
  a [g] = (V - 1.65)/0.33
