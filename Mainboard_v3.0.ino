
/**************************************************************************************/
// Written for Teensy 3.1/3.2
// Emanuel Brici, Cody Glascoe, Zach Moore, Rachael Campbell, Mike Hamilton, Mike Hansen
// #cougballoon
// Iridium Transmitter
// v1.0 Mar 5 2016
// v2.0 Mar 11 2016 New board
// v2.1 Mar 11 2016 Added support for SD card/ADC/GPS
// v2.2 Mar 15 2016 Added full support for ADS1148 ADC
// v2.3 Mar 17 2016 Restructured, added string concatenation
// v2.4 Mar 19 2016 Fixed ADC function errors
// v2.5 Mar 24 2016 Added calibrated thermistor formulas
// v3.0 Apr 05 2016 Added support for Adafruit 10DOF
/***************************************************************************************/


/*
    @NOTES
    -Must be set to 24MHz (non-optimized) prior to compiling/programming
    -DO NOT USE pins 7 or 8 breakouts, those are connected to GPS RX/TX
    -NOT USING LEDs, to much current draw.
*/


/*
    @TODO
    -Get every GPS string to save to SD card
    -If empty GPS string, keep going until we get a full one.
    -Add in new data to concat!!! Add new global variables for new data.

    @DONE
    -Finish adding ADC code
*/

//TESTING
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

#include <IridiumSBD.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>


// different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
// Note that these only control the rate at which the position is echoed, to actually speed up the
// position fix you must also send one of the position fix rate commands below too.
#define PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ  "$PMTK220,30000*2F" // Once every 10 seconds, 100 millihertz.
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
// Position fix update rate commands.
#define PMTK_API_SET_FIX_CTL_100_MILLIHERTZ  "$PMTK300,10000,0,0,0,0*2C" // Once every 10 seconds, 100 millihertz.
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
// Can't fix position faster than 5 times a second!
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
// turn on only the first sentence (GPGLL)
#define PMTK_SET_NMEA_OUTPUT_GLLONLY "$PMTK314,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on only the third sentence (GPVTG)
#define PMTK_SET_NMEA_OUTPUT_VTGONLY "$PMTK314,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on only the fourth sentence (GPGGA)
#define PMTK_SET_NMEA_OUTPUT_GGAONLY "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on only the fifth sentence (GPGSA)
#define PMTK_SET_NMEA_OUTPUT_GSAONLY "$PMTK314,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on only the sixth sentence (GPGSV)
#define PMTK_SET_NMEA_OUTPUT_GSVONLY "$PMTK314,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GPGGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on GPRMC and GPGGA and GPVTG
#define PMTK_SET_NMEA_OUTPUT_RMCGGAVTG "$PMTK314,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// to generate your own sentences, check out the MTK command datasheet and use a checksum calculator
// such as the awesome http://www.hhhh.org/wiml/proj/nmeaxor.html
#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"
// standby command & boot successful message
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_STANDBY_SUCCESS "$PMTK001,161,3*36"  // Not needed currently
#define PMTK_AWAKE "$PMTK010,002*2D"
// ask for the release and version
#define PMTK_Q_RELEASE "$PMTK605*31"
// request for updates on antenna status
#define PGCMD_ANTENNA "$PGCMD,33,1*6C"
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D"

#define NoOp                                            0xFF //No Operation signal for the ADS1148

#define MultiplexerControlRegister0_00h                 0x00
#define BiasVoltageRegister_01h                         0x01
#define MultiplexerControlRegister1_02h                 0x02
#define SystemControlRegister0_03h                      0x03
#define OffsetCalibrationCoefficientRegister0_04h       0x04
#define OffsetCalibrationCoefficientRegister1_05h       0x05
#define OffsetCalibrationCoefficientRegister2_06h       0x06
#define FullScaleCalibrationCoefficientRegister0_07h    0x07
#define FullScaleCalibrationCoefficientRegister1_08h    0x08
#define FullScaleCalibrationCoefficientRegister2_09h    0x09
#define IDACControlRegister0_0Ah                        0x0A
#define IDACControlRegister1_0Bh                        0x0B
#define GPIOConfigurationRegister_0Ch                   0x0C
#define GPIODirectionRegister_0Dh                       0x0D
#define GPIODataRegister_0Eh                            0x0E


#define MAX_NUMBER_DATA_POINTS_PER_FILE                 1000

#define SPI_SPEED                                       200000

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
Adafruit_10DOF                dof = Adafruit_10DOF();

int counter = 0;
File myFile;

String NMEAstring = ""; //Will hold both GGA, RMC, and VTG strings together.
String NMEAstringToSend = "";
char c;

bool newGPSdataReady = false;
bool ADCinactive = true;
bool GPSbusy = false;

int flashRate = 100;

const int shuntResistor1 = 10016; //sensor1
const int shuntResistor2 = 9993; //sensor2
const int shuntResistor3 = 10026; //sensor3

const int ADCchipSelect = 10;
const int ADCstart = 3;

const int SDchipSelect = 15;

int initialRegisterContents[15];

int16_t rawTemperatureOne;
int16_t rawTemperatureTwo;
int16_t rawTemperatureThree;
int16_t rawOzoneLevel;

float accelerometer_x;
float accelerometer_y;
float accelerometer_z;
float magnetometer_x;
float magnetometer_y;
float magnetometer_z;
float DOF_heading;
float gyroscope_x;
float gyroscope_y;
float gyroscope_z;
float DOF_pressure;
float DOF_temperature;
float AVDD;
float DVDD;



//const int LED_Main_Power = 5; // RED
//const int LED_GPS_fix = 6; // RED
//const int LED_Iridium_waiting_to_transmit = 9; // RED (callback function)
//const int LED_Iridium_transmitted_successfully = 22; // GREEN
//const int LED_SD_writing = 23; // GREEN //Eliminate

int loopCounter = 0;

SoftwareSerial ssIridium(0, 1); // RX, TX
IridiumSBD isbd(ssIridium, 10);


void displaySensorDetails(void) {
  
  sensor_t sensor;
  
  accel.getSensor(&sensor);
  Serial.println(F("----------- ACCELEROMETER ----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" m/s^2"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  gyro.getSensor(&sensor);
  Serial.println(F("------------- GYROSCOPE -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" rad/s"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  mag.getSensor(&sensor);
  Serial.println(F("----------- MAGNETOMETER -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  bmp.getSensor(&sensor);
  Serial.println(F("-------- PRESSURE/ALTITUDE ---------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" hPa"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" hPa"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" hPa"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  delay(500);
}

/*
   Set up GPS Modules
*/
void InitializeGPSModule() {

  delay(300);
  //Serial3.println(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  Serial3.println(PMTK_SET_NMEA_OUTPUT_RMCGGAVTG);
  delay(50);
  Serial3.println(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);
  delay(50);
  Serial3.println(PMTK_API_SET_FIX_CTL_100_MILLIHERTZ);
  delay(50);
  Serial3.println(PGCMD_NOANTENNA);
  delay(50);
  Serial3.println(PMTK_ENABLE_SBAS);
  delay(50);
  Serial3.println(PMTK_ENABLE_WAAS);
  delay(50);
  //Serial3.println(PMTK_Q_RELEASE);
  //delay(50);
}


/*
   Initialize serial to pin 13
*/
void serialEvent3() {

  GPSbusy = true;
  Serial.println("Incoming GPS data: ");
  NMEAstring = "";
  while (Serial3.available()) {
    c = Serial3.read();
    NMEAstring.concat(c);
    delayMicroseconds(1200);
  }
  NMEAstringToSend = NMEAstring;
  Serial.println(NMEAstringToSend);
  newGPSdataReady = true;
  GPSbusy = false;
}


/*
   Set up Writing and reading to SD card
*/
void InitializeSDCard() {

  Serial.print("Initializing SD card...");
  Serial.println();
  // see if the card is present and can be initialized:
  int test = SD.begin(SDchipSelect);
  Serial.print("SD.begin return value: ");
  Serial.println(test);
//  if (!SD.begin(SDchipSelect)) {
//    Serial.println("Card failed, or not present...");
//    // don't do anything more:
//    //return;
//  }
  Serial.println("card initialized.");
  Serial.println();

  char* name1 = "datalog.txt";
  File myFile = SD.open(name1, FILE_WRITE);
  if (myFile) {
    myFile.println("***************GPS_DATA_LOGGER******************");
    myFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }

  //READ EVERYTHING ON THE SD CARD
  File myFile2 = SD.open(name1, FILE_READ);
  if (myFile2) {
    Serial.println("####################################################################");
    Serial.println("#...................PRINTING ALL SD CARD DATA......................#");
    Serial.println("####################################################################");
    while (myFile2.available()) {
      Serial.write(myFile2.read());
    }
    myFile2.close();
    Serial.println("####################################################################");
    Serial.println("#..............DONE PRINTING ALL SD CARD DATA......................#");
    Serial.println("####################################################################");
  }  
}


/*
   Log to file all GPS data needed
*/
void writeToFile(char* dataToWrite) {

  Serial.println("Writing to file...");
  Serial.println();
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File myFile = SD.open("datalog.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (myFile) {
    myFile.print("Data ID Number: ");
    myFile.println(counter);
    myFile.println(dataToWrite);
    counter++;
    myFile.close();
    // print to the serial port too:
    Serial.print("Wrote to SD card: ");
    Serial.println(dataToWrite);
    
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
    Serial.println();
  }
}
/*
double sensorOneReading;    // I  Internal temperature
double sensorTwoReading;    // E  External temperature
double sensorThreeReading;  // W  Window temperature
float AVDD;                 // A  AVDD reading
float DVDD;                 // D  DVDD reading
float accelerometer_x;      // AX Accelerometer X-axis
float accelerometer_y;      // AY Accelerometer Y-axis
float accelerometer_z;      // AZ Accelerometer Z-axis
float magnetometer_x;       // MX Magnetometer X-axis
float magnetometer_y;       // MY Magnetometer Y-axis
float magnetometer_z;       // MZ Magnetometer Z-axis
float DOF_heading;          // H  Heading from DOF
float gyroscope_x;          // GX Gyroscope X-axis 
float gyroscope_y;          // GY Gyroscope Y-axis 
float gyroscope_z;          // GZ Gyroscope Z-axis 
float DOF_pressure;         // P  Pressure from DOF
float DOF_temperature;      // T  Temperature from DOF
 */

void concatAllData(double sensorOneReading, double sensorTwoReading, double sensorThreeReading, double ozoneLevel, double AVDD, double DVDD) {
  String temp = "";
  temp = NMEAstringToSend + "I" + String(sensorOneReading) + "E" + String(sensorTwoReading) + "W" + String(sensorThreeReading) + "O" + String(ozoneLevel) + "A" + String(AVDD) + "D" + String(DVDD) + "AX" + String(accelerometer_x) + "AY" + String(accelerometer_y) + "AZ" + String(accelerometer_z) + "MX" + String(magnetometer_x) + "MY" + String(magnetometer_y) + "MZ" + String(magnetometer_z) + "H" + String(DOF_heading) + "GX" + String(gyroscope_x) + "GY" + String(gyroscope_y) + "GZ" + String(gyroscope_z) + "P" + String(DOF_pressure) + "T" + String(DOF_temperature);
  NMEAstringToSend = (char*) temp.c_str();
}


void setup() {

  int signalQuality = -1;

//  pinMode(LED_Main_Power, OUTPUT);
//  digitalWrite(LED_Main_Power, LOW);
//  pinMode(LED_GPS_fix, OUTPUT);
//  digitalWrite(LED_GPS_fix, LOW);
//  pinMode(LED_Iridium_waiting_to_transmit, OUTPUT);
//  digitalWrite(LED_Iridium_waiting_to_transmit, LOW);
//  pinMode(LED_Iridium_transmitted_successfully, OUTPUT);
//  digitalWrite(LED_Iridium_transmitted_successfully, LOW);
//  pinMode(LED_SD_writing, OUTPUT);
//  digitalWrite(LED_SD_writing, LOW);

  pinMode(ADCchipSelect, OUTPUT);
  digitalWrite(ADCchipSelect, HIGH);

  pinMode(ADCstart, OUTPUT);
  digitalWrite(ADCstart, HIGH);

  pinMode(SDchipSelect, OUTPUT);
  digitalWrite(SDchipSelect, HIGH);

  Serial.begin(9600);

  ssIridium.begin(19200);

  /*
     These allow the host application to provide a Stream object (serial port) that can be used to
     monitor the RockBLOCK serial traffic and diagnostic messages, respectively.  The typical usage
     is to simply use the Arduino serial port to monitor both of these—assuming that it is connected
     to a PC serial console and not otherwise used:
  */
  isbd.attachConsole(Serial);
  //isbd.attachDiags(Serial); //Extra info for testing will be printed out

  /*
     1 for low-current USB power source, 0 for default power. This method defines the internal delays
     between retransmission.  Low current applications need longer delays.
  */
  //isbd.setPowerProfile(1); //Use this when connected to USB Serial Cable direct to computer
  isbd.setPowerProfile(0); //Use this when connected to Blue Battery

  /*
     begin() also serves as the way to wake a RockBLOCK that is asleep. At initial power up, this
     method make take several tens of seconds as the device charges.  When waking from sleep the
     process should be faster. If provided, the user’s ISBDCallback function is repeatedly called
     during this operation. This function should be called before any transmit/receive message.
  */
  isbd.begin();

  /*
     The minimum signal quality on a scale of 0 (nonexistent) to 5 (superb) needed before the library
     allows a transmission to begin. (default=2) (AT+CSQ)
  */
  isbd.setMinimumSignalQuality(1);

  /*
     If provided, the user’s ISBDCallback function is repeatedly called during this operation.
     This method is mostly informational.  It is not strictly necessary for the user application
     to verify that a signal exists before calling one of the transmission functions, as these
     check signal quality themselves.
  */
  int err = isbd.getSignalQuality(signalQuality);
  if (err != 0) {
    Serial.print("SignalQuality failed: error ");
    Serial.println(err);
    Serial1.println("");
    return;
  }

  Serial.print("Signal quality is ");
  Serial.println(signalQuality);
  Serial1.println("");

  Serial3.begin(9600);
  delay(500);
  InitializeGPSModule();
  delay(1000);
  Serial3.end();

  flashRate = 1000;

  SPI.begin();
  delay(1000);

  InitializeSDCard();
  delay(1000);

  ADS1148_reset();
  delay(500);

  Serial.println("ADS1148 Printing Register Values.");
  ADS1148_read_and_print_all_registers();
  Serial.println("");
  delay(520);

  Serial.println(F("Adafruit 10DOF Tester")); Serial.println("");
  
  /* Initialise the sensors */
  if(!accel.begin()){
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    //while(1);
  }
  if(!mag.begin()){
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    //while(1);
  }
  if(!bmp.begin()){
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }
  if(!gyro.begin()){
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();

  Serial.println("");
  Serial.println("");
  Serial.println("####################################################################");
  Serial.println("#...................#COUGBALLOON INITIALIZED.......................#");
  Serial.println("####################################################################");
  Serial.println("");
  Serial.println("");

}

void loop() {

  Serial3.begin(9600);

  delay(10000);//Delay 15 seconds, long enough to capture new GPS data

  Serial3.end();

  ADS1148_set_ADC_for_sensor_one();
  rawTemperatureOne = ADS1148_read_data_once();
  //writeIntegerToFile(rawTemperatureOne);
  Serial.print("Sensor 1 raw data: ");
  Serial.println(rawTemperatureOne);
  if (rawTemperatureOne < 0) {
    Serial.println("Error, rawTemperatureOne value < 0");
    ADS1148_read_and_print_all_registers();
  }
  Serial.print("Temperature Probe One: ");
  double temp1 = convertRawToFahrenheit(rawTemperatureOne, shuntResistor1);
  Serial.print(temp1);
  Serial.print("°C  ");
  Serial.print(temp1 * 9.0 / 5.0 + 32.0);
  Serial.println("°F");
  delay(500);

  ADS1148_set_ADC_for_sensor_two();
  rawTemperatureTwo = ADS1148_read_data_once();
  //writeIntegerToFile(rawTemperatureTwo);
  Serial.print("Sensor 2 raw data: ");
  Serial.println(rawTemperatureTwo);
  if (rawTemperatureTwo < 0) {
    Serial.println("Error, rawTemperatureTwo value < 0");
    ADS1148_read_and_print_all_registers();
  }
  Serial.print("Temperature Probe Two: ");
  double temp2 = convertRawToFahrenheit(rawTemperatureTwo, shuntResistor2);
  Serial.print(temp2);
  Serial.print("°C  ");
  Serial.print(temp2 * 9.0 / 5.0 + 32.0);
  Serial.println("°F");
  delay(500);

  ADS1148_set_ADC_for_sensor_three();
  rawTemperatureThree = ADS1148_read_data_once();
  //writeIntegerToFile(rawTemperatureThree);
  Serial.print("Sensor 3 raw data: ");
  Serial.println(rawTemperatureThree);
  if (rawTemperatureThree < 0) {
    Serial.println("Error, rawTemperatureThree value < 0");
    ADS1148_read_and_print_all_registers();
  }
  Serial.print("Temperature Probe Three: ");
  double temp3 = convertRawToFahrenheit(rawTemperatureThree, shuntResistor3);
  Serial.print(temp3);
  Serial.print("°C  ");
  Serial.print(temp3 * 9.0 / 5.0 + 32.0);
  Serial.println("°F");
  delay(500);

  //ADS1148_set_ADC_for_ozone_sensor(); // rawOzoneLevel
  //rawOzoneLevel = ADS1148_read_data_once();
  Serial.print("Ozone raw data: ");
  Serial.println(rawOzoneLevel);
  if (rawOzoneLevel < 0) {
    Serial.println("Error, rawOzoneLevel value < 0");
    ADS1148_read_and_print_all_registers();
  }
  double ozoneLevel = convertOzoneRawToPPB(rawOzoneLevel);
  Serial.print(ozoneLevel);
  Serial.println(" PPB");
  delay(500);

  //Measure and display the AVDD
  ADS1148_set_ADC_for_AVDD_reading();
  AVDD = ADS1148_read_data_once();
  Serial.println();
  Serial.print("AVDD: ");
  AVDD = AVDD/32767.0*2.97*4.0;
  Serial.println(AVDD);
  delay(300);

  //Measure and display the DVDD
  ADS1148_set_ADC_for_DVDD_reading();
  DVDD = ADS1148_read_data_once();
  Serial.println();
  Serial.print("DVDD: ");
  DVDD = DVDD/32767.0*2.97*4.0;
  Serial.println((DVDD));
  delay(300);
  
  Serial.println();
  Serial.println("ADC data retrieval complete...");
  Serial.println();

  /* Get a new sensor event */
  sensors_event_t event;
   
  /* Display the results (acceleration is measured in m/s^2) */
  accel.getEvent(&event);
  accelerometer_x = event.acceleration.x;
  accelerometer_y = event.acceleration.y;
  accelerometer_z = event.acceleration.z;
  Serial.print(F("Acceleration "));
  Serial.print("X: "); Serial.print(accelerometer_x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(accelerometer_y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(accelerometer_z); Serial.print("  ");Serial.println("m/s^2 ");

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  mag.getEvent(&event);
  sensors_vec_t   orientation;
  magnetometer_x = event.magnetic.x;
  magnetometer_y = event.magnetic.y;
  magnetometer_z = event.magnetic.z;
  Serial.print(F("Magnetometer "));
  Serial.print("X: "); Serial.print(magnetometer_x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(magnetometer_y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(magnetometer_z); Serial.print("  ");Serial.println("uT");
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &event, &orientation)){
    /* 'orientation' should have valid .heading data now */
    DOF_heading = orientation.heading;
    Serial.print(F("Heading: "));
    Serial.print(DOF_heading);
    Serial.println(F("; "));
  }

  /* Display the results (gyrocope values in rad/s) */
  gyro.getEvent(&event);
  gyroscope_x = event.gyro.x;
  gyroscope_y = event.gyro.y;
  gyroscope_z = event.gyro.z;
  Serial.print(F("Gyroscope "));
  Serial.print("X: "); Serial.print(gyroscope_x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(gyroscope_y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(gyroscope_z); Serial.print("  ");Serial.println("rad/s ");  

  /* Display the pressure sensor results (barometric pressure is measure in hPa) */
  bmp.getEvent(&event);
  if (event.pressure)
  {
    /* Display atmospheric pressure in hPa */
    DOF_pressure = event.pressure;
    Serial.print(F("Pressure "));
    Serial.print(DOF_pressure);
    Serial.print(F(" hPa, "));
    /* Display ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    DOF_temperature = temperature;
    Serial.print(temperature);
    Serial.println(F("°C "));

  }

  Serial.println();
  Serial.println("10DOF data retrieval complete...");
  Serial.println();
  
  //Concats NMEA string and three temperatures together //AND NEW STUFF!!
  concatAllData(temp1, temp2, temp3, ozoneLevel, AVDD, DVDD);

  char* NMEAstringToSend1 = (char*) NMEAstringToSend.c_str();

  int len = strlen(NMEAstringToSend1);
  for (int i = 0; i < len; i++) {
    if (NMEAstringToSend1[i] == '\r') {
      //Serial.println("CR!!");
      NMEAstringToSend1[i] = '#';
    }
    if (NMEAstringToSend1[i] == '\n') {
      //Serial.println("NL!!");
      NMEAstringToSend1[i] = '!';
    }
  }
  NMEAstringToSend1[len] = '\0';
  Serial.println("Sending and saving this string: ");
  Serial.println(NMEAstringToSend1);
  Serial.println();

  writeToFile(NMEAstringToSend1);         //WRITING TO FILE
  Serial.println();

  loopCounter++;
  Serial.print("Loop Counter: ");
  Serial.println(loopCounter);
  Serial.println();



  //Send GPS data if new data has arrived
  if (newGPSdataReady && (loopCounter >= 5)) {

    Serial.println("Beginning transmission...");
    Serial.println();

    int err = isbd.sendSBDTextLARGE(NMEAstringToSend1);
    if (err != 0) {
      Serial.print("sendSBDText failed: error ");
      Serial.println(err);
      return;
    }
    Serial.println("Data transmitted...");
    Serial.println();
    newGPSdataReady = false;
  }
  
  //FOR TESTING ONLY!!! REMOVE WHEN GPS IS BACK
  NMEAstringToSend = "$GPGGA,025346.000,4541.6390,N,12234.3097,W,1,06,1.30,49.8,M,-19.1,M,,*6E#!$GPRMC,025346.000,A,4541.6390,N,12234.3097,W,0.14,51.70,250316,,,A*4C#!$GPVTG,51.70,T,,M,0.14,N,0.25,K,A*0C#!";// CAN GET RID OF THIS WHEN GPS IS CONNECTED AGAIN

  if (loopCounter >= 5) {
    loopCounter = 0;
  }

}


bool ISBDCallback() {

  //In this function, put what you want to happen during RockBLOCK operations.
  //digitalWrite(LED_Iridium_waiting_to_transmit, (millis() / flashRate) % 2 == 1 ? HIGH : LOW);

  return true;
}



double convertRawToFahrenheit(int16_t rawInput, uint16_t shuntValue) {
  double newInput = (double)rawInput;
  double value = ((0.3683584) * newInput);//12.07kR / 2^15-1, Gives resulting total resistance
  //Serial.println(value);
  value = 1.0 / ((1.0 / value) - (1.0 / (double)shuntValue)); //Gives thermistor resistance
  //Serial.println(value);
  ////=1/((1/298.15)+(1/3449)*LN(RESISTANCE/11021))-273.15
  value = log(value / 11021.0) * (1.0 / 3449.0); //Get ln(), miltiply is by inverse of new beta
  //Serial.println(value);
  value = 1.0 / (value + (1.0 / 298.15)); //Add inverse of t0 temp, inverse again
  //Serial.println(value);
  value = value - 273.15;//Convert from Kelvin to Celsius
  //Serial.println(value);
  return value;
}

//NEED TO WRITE THIS FUNCTION
double convertOzoneRawToPPB(int16_t rawOzoneLevel){
  double returnValue = rawOzoneLevel;
  return returnValue; 
}

/*
   Writes to one register at a time.
*/
void ADS1148_writeRegister(byte registerNumber, byte value) {
  digitalWrite(ADCstart, HIGH);//ADCstart pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(20000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  SPI.transfer(0x40 | registerNumber);
  SPI.transfer(0x00);
  SPI.transfer(value);
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(ADCstart, LOW);
  delay(210);
}

/*
   Reads one register at a time.
*/
int ADS1148_readRegister(byte registerNumber) {
  digitalWrite(ADCstart, HIGH);//ADCstart pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  SPI.transfer(0x20 | registerNumber);
  SPI.transfer(0x00);
  int returnValue = SPI.transfer(NoOp);
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(ADCstart, LOW);
  return returnValue;
}


/*
   Wakes up ADS1148 from sleep mode.
*/
void ADS1148_wakeup() {
  digitalWrite(ADCstart, HIGH);//ADCstart pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  SPI.transfer(0x00);//Can be 0x01 also.
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(ADCstart, LOW);
}

/*
   Stop reading data continuously.
*/
void ADS1148_SDATAC() {
  digitalWrite(ADCstart, HIGH);//ADCstart pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  SPI.transfer(0x16);//Can be 0x01 also.
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(ADCstart, LOW);
}


/*
   Sets the ADS1148 to sleep mode.
*/
void ADS1148_sleep() {
  digitalWrite(ADCstart, HIGH);//ADCstart pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  SPI.transfer(0x02);//Can be 0x03 also.
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(ADCstart, LOW);
}

/*
   Synchronize the A/D conversion
*/
void ADS1148_sync() {
  digitalWrite(ADCstart, HIGH);//ADCstart pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  SPI.transfer(0x04);//Can be 0x05 also.
  SPI.transfer(0x04);//Can be 0x05 also.
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(ADCstart, LOW);
}

/*
   Wakes up ADS1148 from sleep mode.
*/
void ADS1148_reset() {
  digitalWrite(ADCstart, HIGH);//ADCstart pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  SPI.transfer(0x06);//Can be 0x01 also.
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(ADCstart, LOW);
}

/*
   Reads the current data from the ADS1148.
*/
uint16_t ADS1148_read_data_once() {
  digitalWrite(ADCstart, HIGH);//ADCstart pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);

  SPI.transfer(0x12);//Can be 0x13 also.
  uint8_t returnMSB = SPI.transfer(NoOp);
  //Serial.println(returnMSB, HEX);
  uint8_t returnLSB = SPI.transfer(NoOp);
  //Serial.println(returnLSB, HEX);
  uint16_t returnValue = (returnMSB << 8 ) | returnLSB;
  //Serial.println(returnValue, HEX);

  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(ADCstart, LOW);
  return returnValue;
}


/*
   System offset calibration.
   This command initiates a system offset calibration. For a system offset calibration, the input
   should be externally set to zero. The OFC register is updated when this operation completes.
*/
void ADS1148_system_offset_calibration() {
  //Set MultiplexerControlRegister1_02h to 0x21
  //ADC input is set to -> Inputs shorted to midsupply (AVDD + AVSS)/2
  ADS1148_writeRegister(MultiplexerControlRegister1_02h, 0x21);

  //Perform the calibration
  digitalWrite(ADCstart, HIGH);//ADCstart pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  SPI.transfer(0x60);
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(ADCstart, LOW);
  Serial.println("ADS1148 System Offset Calibration complete...");
}

/*
   System gain calibration.
   This command initiates the system gain calibration. For a system gain calibration, the input
   should be set to full-scale. The FSC register is updated after this operation.
*/
void ADS1148_system_gain_calibration() {
  //Set MultiplexerControlRegister1_02h to 0x22
  //ADC input is set to -> VREFP – VREFN (full-scale)
  ADS1148_writeRegister(MultiplexerControlRegister1_02h, 0x22);

  //Perform the calibration
  digitalWrite(ADCstart, HIGH);//ADCstart pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  SPI.transfer(0x61);
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(ADCstart, LOW);
  Serial.println("ADS1148 System Gain Calibration complete...");
}

/*
   Self offset calibration.
   This command initiates a self-calibration for offset. The device internally shorts the inputs
   and performs the calibration. The OFC register is updated after this operation.
*/
void ADS1148_self_offset_calibration() {
  Serial.println("Beginning ADS1148 Self Offset Calibration");
  //Set MultiplexerControlRegister1_02h to 0x21
  //ADC input is set to -> Inputs shorted to midsupply (AVDD + AVSS)/2
  ADS1148_writeRegister(MultiplexerControlRegister1_02h, 0x21);
  ADS1148_read_and_print_all_registers();
  //Perform the calibration
  digitalWrite(ADCstart, HIGH);//ADCstart pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  SPI.transfer(0x62);
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(ADCstart, LOW);
  delay(5000);
  Serial.println("ADS1148 Self Offset Calibration complete...");
}

/*
   Function to read and return value from Sensor 1
   Positive Input - AIN7 (IDAC source)
   Negative Input - AIN0
*/
void ADS1148_set_ADC_for_sensor_one() {
  ADS1148_writeRegister(MultiplexerControlRegister0_00h, 0x38); //0b00 11_1 000
  ADS1148_writeRegister(BiasVoltageRegister_01h, 0x00); //0b0000_0000
  ADS1148_writeRegister(MultiplexerControlRegister1_02h, 0x20); //0b0 01 0_0 000
  ADS1148_writeRegister(SystemControlRegister0_03h, 0x00); //0b0000_0000
  //4,5,6 are OFC
  //7,8,9 are FSC
  ADS1148_writeRegister(IDACControlRegister0_0Ah, 0x93); //0b1001 _0 111 250uA
  ADS1148_writeRegister(IDACControlRegister1_0Bh, 0x7F); //0b0111_ 1111
  //C,D,E are GPIO
}

/*
   Function to read and return value from Sensor 2
   Positive Input - AIN5 (IDAC source)
   Negative Input - AIN0
*/
void ADS1148_set_ADC_for_sensor_two() {
  ADS1148_writeRegister(MultiplexerControlRegister0_00h, 0x28); //0b00 10_1 000
  ADS1148_writeRegister(BiasVoltageRegister_01h, 0x00); //0b0000_0000
  ADS1148_writeRegister(MultiplexerControlRegister1_02h, 0x20); //0b0 01 0_0 000
  ADS1148_writeRegister(SystemControlRegister0_03h, 0x00); //0b0000_0000
  //4,5,6 are OFC
  //7,8,9 are FSC
  ADS1148_writeRegister(IDACControlRegister0_0Ah, 0x93); //0b1001 _0 111 250uA
  ADS1148_writeRegister(IDACControlRegister1_0Bh, 0x5F); //0b0101_ 1111
  //C,D,E are GPIO
}

/*
   Function to read and return value from Sensor 3
   Positive Input - AIN3 (IDAC source)
   Negative Input - AIN0
*/
void ADS1148_set_ADC_for_sensor_three() {
  ADS1148_writeRegister(MultiplexerControlRegister0_00h, 0x18); //0b00 01_1 000
  ADS1148_writeRegister(BiasVoltageRegister_01h, 0x00); //0b0000_0000
  ADS1148_writeRegister(MultiplexerControlRegister1_02h, 0x20); //0b0 01 0_0 000
  ADS1148_writeRegister(SystemControlRegister0_03h, 0x00); //0b0000_0000
  //4,5,6 are OFC
  //7,8,9 are FSC
  ADS1148_writeRegister(IDACControlRegister0_0Ah, 0x93); //0b1001_ 0 011 250uA
  ADS1148_writeRegister(IDACControlRegister1_0Bh, 0x3F); //0b0011_ 1111
  //C,D,E are GPIO
}

/*
   Function to read and return value from Ozone Sensor
   Positive Input - AIN6 (No IDAC source needed for this measurement)
   Negative Input - AIN0
*/
void ADS1148_set_ADC_for_ozone_sensor() {
  ADS1148_writeRegister(MultiplexerControlRegister0_00h, 0x30); //0b00 11_0 000
  ADS1148_writeRegister(BiasVoltageRegister_01h, 0x00); //0b0000_0000
  ADS1148_writeRegister(MultiplexerControlRegister1_02h, 0x20); //0b0 01 0_0 000
  ADS1148_writeRegister(SystemControlRegister0_03h, 0x00); //0b0000_0000
  //4,5,6 are OFC
  //7,8,9 are FSC
  ADS1148_writeRegister(IDACControlRegister0_0Ah, 0x90); //0b1001_ 0 000 IDAC off
  ADS1148_writeRegister(IDACControlRegister1_0Bh, 0xFF); //0b1111_ 1111
  //C,D,E are GPIO
}


/*
   Function to set registers to read AVDD
*/
void ADS1148_set_ADC_for_AVDD_reading() {
  ADS1148_writeRegister(MultiplexerControlRegister0_00h, 0x30); //0b00 11_0 000
  ADS1148_writeRegister(BiasVoltageRegister_01h, 0x00); //0b0000_0000
  ADS1148_writeRegister(MultiplexerControlRegister1_02h, 0x26); //0b0 01 0_0 000
  ADS1148_writeRegister(SystemControlRegister0_03h, 0x00); //0b0000_0000
  //4,5,6 are OFC
  //7,8,9 are FSC
  //ADS1148_writeRegister(IDACControlRegister0_0Ah, 0x90); //0b1001_ 0 000 IDAC off
  //ADS1148_writeRegister(IDACControlRegister1_0Bh, 0xFF); //0b1111_ 1111
  //C,D,E are GPIO
}


/*
   Function to set registers to read AVDD
*/
void ADS1148_set_ADC_for_DVDD_reading() {
  ADS1148_writeRegister(MultiplexerControlRegister0_00h, 0x30); //0b00 11_0 000
  ADS1148_writeRegister(BiasVoltageRegister_01h, 0x00); //0b0000_0000
  ADS1148_writeRegister(MultiplexerControlRegister1_02h, 0x27); //0b0 01 0_0 000
  ADS1148_writeRegister(SystemControlRegister0_03h, 0x00); //0b0000_0000
  //4,5,6 are OFC
  //7,8,9 are FSC
  //ADS1148_writeRegister(IDACControlRegister0_0Ah, 0x90); //0b1001_ 0 000 IDAC off
  //ADS1148_writeRegister(IDACControlRegister1_0Bh, 0xFF); //0b1111_ 1111
  //C,D,E are GPIO
}

/*
   Function to read and print value from all registers
*/
void ADS1148_read_and_print_all_registers() {
  for (uint8_t i = 0; i < 15; i++) {
    int returnValue = ADS1148_readRegister(i);
    Serial.print("Reg_");
    Serial.print(i, HEX);
    Serial.print(": 0x");
    Serial.println(returnValue, HEX);
    delay(10);
  }
}

void ADS1148_read_all_registers() {
  digitalWrite(ADCstart, HIGH);//ADCstart pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  SPI.transfer(0x20);
  SPI.transfer(0x0E);
  SPI.transfer(NoOp);
  SPI.transfer(NoOp);
  SPI.transfer(NoOp);
  SPI.transfer(NoOp);
  SPI.transfer(NoOp);
  SPI.transfer(NoOp);
  SPI.transfer(NoOp);
  SPI.transfer(NoOp);
  SPI.transfer(NoOp);
  SPI.transfer(NoOp);
  SPI.transfer(NoOp);
  SPI.transfer(NoOp);
  SPI.transfer(NoOp);
  SPI.transfer(NoOp);
  SPI.transfer(NoOp);
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(ADCstart, LOW);
}

/*
   Capture and store initial register values after calibration.
*/
void ADS1148_save_initial_register_values() {
  for (int i = 0; i < 15; i++) {
    initialRegisterContents[i] = ADS1148_readRegister(i);
  }
  Serial.println("ADS1148 Initial Register Values saved.");
}





/*
   CODE FROM HADLEY
   hhuj1j..liio98yru7r43wvbazzxwrtht43  2q2ghre31146uhjl

*/

