
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
// v4.0 Apr 11 2016 Added support for ozone sensor
// v4.1 Apr 12 2016 Added LOCUS GPS logging command
/***************************************************************************************/


/*
 * 
 @NOTES
 -Must be set to 24MHz (non-optimized) prior to compiling/programming
 -DO NOT USE pins 7 or 8 breakouts, those are connected to GPS RX/TX
 -NOT USING LEDs, to much current draw.
 
 @TODO
 -If empty GPS string, keep going until we get a full one.
 
 @DONE
 -Finish adding ADC code
 -Add in new data to concat!!! Add new global variables for new data.
 -Save RAW data for all ADC shit.
 -Get every GPS string to save to SD card or LOCUS
 
 */

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
//https://www.adafruit.com/datasheets/PMTK_A11.pdf
#define PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ  "$PMTK220,10000*2F" // Once every 10 seconds, 100 millihertz.
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

//Antenna status updates
#define PGCMD_ANTENNA "$PGCMD,33,1*6C"
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D"
//Antenna responses are below
//$PGACK,33,0*6E<CR><LF> (antenna status/advisor messages are disabled)
//$PGACK,33,1*6F<CR><LF> (antenna status/advisor messages are enabled)

#define PGCMD_ANTENNA_STATUS_INQUIRY "$PGTOP,11,3*6F" //inquiry antenna status
/*  Response packet:
    $PGTOP,11,value*checksum
      Value:
      1=>Active Antenna Shorted
      2=>Using Internal Antenna
      3=>Using Active Antenna
 */

#define PMTK_START_LOCUS_LOGGING "PMTK185,0*22"
#define PMTK_LOCUS_STOP_LOGGER "PMTK185,1*23"
#define PMTK_DUMP_LOCUS_FLASH_DATA "PMTK622,0*28"
//http://mt-system.ru/sites/default/files/documents/locus_manual_for_mtk_gnss_platform_v1.00.pdf

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

#define R1                      149121.0
#define R2                      238000.0

#define ADC_INTERNAL_REFERENCE_VOLTAGE    2.0480
#define ADC_EXTERNAL_REFERENCE_VOLTAGE    2.9900 //Get exact reading in lab and modify this
#define SENSOR_INPUT_VOLTAGE              4.9200 //Get exact reading in lab and modify this

#define ADC_MAX_VALUE           32767.0

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

//int flashRate = 100;

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

int loopCounter = 0;

SoftwareSerial ssIridium(0, 1); // RX, TX
IridiumSBD isbd(ssIridium, 10);


/*
 * Displays 10DOF Sensor details.
 *
 * @TODO
 *  -Nothing
 *
 */
void displaySensorDetails(void) {
    
    sensor_t sensor;
    
    accel.getSensor(&sensor);
    Serial.println("####################################################################");
    Serial.println("#........................ACCELEROMETER.............................#");
    Serial.println("####################################################################");
    Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" m/s^2"));
    Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" m/s^2"));
    Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" m/s^2"));
    Serial.println(F(""));
    
    gyro.getSensor(&sensor);
    Serial.println("####################################################################");
    Serial.println("#..........................GYROSCOPE...............................#");
    Serial.println("####################################################################");
    Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" rad/s"));
    Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" rad/s"));
    Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" rad/s"));
    Serial.println(F(""));
    
    mag.getSensor(&sensor);
    Serial.println("####################################################################");
    Serial.println("#.........................MAGNETOMETER.............................#");
    Serial.println("####################################################################");
    Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" uT"));
    Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" uT"));
    Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" uT"));
    Serial.println(F(""));
    
    bmp.getSensor(&sensor);
    Serial.println("####################################################################");
    Serial.println("#.....................PRESSURE/ALTITUDE............................#");
    Serial.println("####################################################################");
    Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" hPa"));
    Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" hPa"));
    Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" hPa"));
    Serial.println(F(""));
    
    delay(500);
}


/*
 * Initializes GPS Module.
 *
 * @TODO
 *  -Check return value from command to verify command was properly passed. If not, try again?
 *
 */
void InitializeGPSModule() {
    
    Serial.println("####################################################################");
    Serial.println("#.......................INITIALIZING GPS MODULE....................#");
    Serial.println("####################################################################");
    Serial.println("");
    delay(200);
    Serial.println("PMTK_SET_NMEA_OUTPUT_OFF");
    Serial.println(PMTK_SET_NMEA_OUTPUT_OFF);
    Serial3.println(PMTK_SET_NMEA_OUTPUT_OFF);
    delay(200);
    Serial.println("PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ");
    Serial.println(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);
    Serial3.println(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);
    delay(200);
    Serial.println("PMTK_API_SET_FIX_CTL_100_MILLIHERTZ");
    Serial.println(PMTK_API_SET_FIX_CTL_100_MILLIHERTZ);
    Serial3.println(PMTK_API_SET_FIX_CTL_100_MILLIHERTZ);
    delay(200);
    Serial.println("PGCMD_NOANTENNA");
    Serial.println(PGCMD_NOANTENNA);
    Serial3.println(PGCMD_NOANTENNA);
    delay(200);
    Serial.println("PMTK_ENABLE_SBAS");
    Serial.println(PMTK_ENABLE_SBAS);
    Serial3.println(PMTK_ENABLE_SBAS);
    delay(200);
    Serial.println("PMTK_ENABLE_WAAS");
    Serial.println(PMTK_ENABLE_WAAS);
    Serial3.println(PMTK_ENABLE_WAAS);
    delay(200);
    Serial.println("Checking firmware version...");
    Serial.println("PMTK_Q_RELEASE");
    Serial.println(PMTK_Q_RELEASE);
    Serial3.println(PMTK_Q_RELEASE);
    delay(200);
    //Serial.println("PMTK_DUMP_LOCUS_FLASH_DATA");
    //Serial.println(PMTK_DUMP_LOCUS_FLASH_DATA);
    //Serial3.println(PMTK_DUMP_LOCUS_FLASH_DATA);
    //delay(200);
    //Serial.println("PMTK_START_LOCUS_LOGGING");
    //Serial.println(PMTK_START_LOCUS_LOGGING);
    //Serial3.println(PMTK_START_LOCUS_LOGGING);
    //delay(200);
    Serial.println("PGCMD_ANTENNA_STATUS_INQUIRY");
    Serial.println(PGCMD_ANTENNA_STATUS_INQUIRY);
    Serial3.println(PGCMD_ANTENNA_STATUS_INQUIRY);
    delay(200);
    Serial.println("PMTK_START_LOCUS_LOGGING");
    Serial.println(PMTK_START_LOCUS_LOGGING);
    Serial3.println(PMTK_START_LOCUS_LOGGING);
    delay(200);
    Serial.println("PMTK_SET_NMEA_OUTPUT_RMCGGAVTG");
    Serial.println(PMTK_SET_NMEA_OUTPUT_RMCGGAVTG);
    Serial3.println(PMTK_SET_NMEA_OUTPUT_RMCGGAVTG);
    delay(200);
    
    Serial.println("####################################################################");
    Serial.println("#.......................GPS MODULE INITIALIZED.....................#");
    Serial.println("####################################################################");
    Serial.println("");
}


/*
 * Function to capture Serial3 (GPS) data into global variable.
 *
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
 * Set up Writing and reading to SD card
 *
 *
 */
void InitializeSDCard() {
    
    Serial.println("####################################################################");
    Serial.println("#..................INITIALIZING SD CARD............................#");
    Serial.println("####################################################################");
    Serial.println();
    //See if the card is present and can be initialized:
    SD.begin(SDchipSelect);
    // The code below returns an error, don't know why.
    //  if (!SD.begin(SDchipSelect)) {
    //    Serial.println("Card failed, or not present...");
    //    return;
    //  }
    Serial.println("SD card initialized.");
    Serial.println();
    
    char* name1 = "datalog.txt";
    File myFile1 = SD.open(name1, FILE_WRITE);
    if (myFile1) {
        myFile1.println("**************************GPS_DATA_LOGGER***************************");
        myFile1.close();
    }
    //If the file isn't open, pop up an error:
    else {
        Serial.println("error opening datalog.txt");
    }

    File myFile2 = SD.open("alldata.txt", FILE_WRITE);
    if (myFile2) {
        myFile2.print("***********************LOGGING ALL DATA**********************");
        myFile2.close();
    }
    else {
        Serial.println("error opening alldata.txt");
        Serial.println();
    }

    Serial.println("####################################################################");
    Serial.println("#..................SD CARD INITIALIZED.............................#");
    Serial.println("####################################################################");
    
    //Reads everything on the SD card
    File myFileReader = SD.open(name1, FILE_READ);
    if (myFileReader) {
        Serial.println("####################################################################");
        Serial.println("#...................PRINTING ALL SD CARD DATA......................#");
        Serial.println("####################################################################");
        while (myFileReader.available()) {
            Serial.write(myFileReader.read());
        }
        myFileReader.close();
        Serial.println("####################################################################");
        Serial.println("#..............DONE PRINTING ALL SD CARD DATA......................#");
        Serial.println("####################################################################");
        Serial.println("");
    }
}


/*
 * Log to file the entire string that is sent to iridium satellite
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
 * Log to file all data as it is read.
 */
void writeAllDataToFile(char* dataType, float data) {
    File fileDescriptor1 = SD.open("alldata.txt", FILE_WRITE);
    if (fileDescriptor1) {
        fileDescriptor1.print(dataType);
        fileDescriptor1.print(": ");
        fileDescriptor1.println(data);
        fileDescriptor1.close();
    }
    else {
        Serial.println("error opening alldata.txt");
        Serial.println();
    }
}


/*
 double sensorOneReading;    // I  Internal temperature
 double sensorTwoReading;    // E  External temperature
 double sensorThreeReading;  // W  Window temperature
 double ozoneLevel;          // O Ozone Level
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
    //Serial.println("starting concat function....");
    temp = NMEAstringToSend + "I" + String(sensorOneReading) + "E" + String(sensorTwoReading) + "W" + String(sensorThreeReading) + "O" + String(ozoneLevel) + "A" + String(AVDD) + "D" + String(DVDD) + "AX" + String(accelerometer_x) + "AY" + String(accelerometer_y) + "AZ" + String(accelerometer_z) + "MX" + String(magnetometer_x) + "MY" + String(magnetometer_y) + "MZ" + String(magnetometer_z) + "H" + String(DOF_heading) + "GX" + String(gyroscope_x) + "GY" + String(gyroscope_y) + "GZ" + String(gyroscope_z) + "P" + String(DOF_pressure) + "T" + String(DOF_temperature);
    NMEAstringToSend = (char*) temp.c_str();
    //Serial.println("finishing concat function....");
}


void setup() {
    
    int signalQuality = -1;
    
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
    Serial.println("");
    Serial.println("####################################################################");
    Serial.println("#..........IRIDIUM SATELLITE TRANSMITTER INITIALIZED...............#");
    Serial.println("####################################################################");
    
    Serial3.begin(9600);
    InitializeGPSModule();
    delay(1000);
    Serial3.end();
    
    //flashRate = 1000;
    
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
    Serial.println("####################################################################");
    Serial.println("#...................#COUGBALLOON INITIALIZED.......................#");
    Serial.println("####################################################################");
    Serial.println("####################################################################");
    Serial.println("");
    Serial.println("");
    
    //Added for testing
    Serial3.begin(9600);
}

void loop() {
    
    Serial3.begin(9600);
    delay(10000);//Delay 15 seconds, long enough to capture new GPS data
    Serial3.end();
    
    ADS1148_set_ADC_for_ozone_sensor();
    rawOzoneLevel = ADS1148_read_data_once();
    Serial.print("Ozone sensor raw data: ");
    Serial.println(rawOzoneLevel);
    writeAllDataToFile("Ozone sensor raw data: ", rawOzoneLevel);
    if (rawOzoneLevel < 0) {
        Serial.println("Error, rawOzoneLevel value < 0");
        ADS1148_read_and_print_all_registers();
    }
    double ozonePPB = convertOzoneRawToPPB(rawOzoneLevel);
    Serial.print(ozonePPB);
    Serial.println(" ppb");
    writeAllDataToFile("Ozone sensor ppb: ", ozonePPB);
    delay(500);
    
    ADS1148_set_ADC_for_sensor_one();
    rawTemperatureOne = ADS1148_read_data_once();
    //writeIntegerToFile(rawTemperatureOne);
    Serial.print("Sensor 1 raw data: ");
    Serial.println(rawTemperatureOne);
    writeAllDataToFile("Sensor 1 raw data: ", rawTemperatureOne);
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
    writeAllDataToFile("Temperature Probe One (°C): ", temp1);
    delay(500);
    
    ADS1148_set_ADC_for_sensor_two();
    rawTemperatureTwo = ADS1148_read_data_once();
    //writeIntegerToFile(rawTemperatureTwo);
    Serial.print("Sensor 2 raw data: ");
    Serial.println(rawTemperatureTwo);
    writeAllDataToFile("Sensor 2 raw data: ", rawTemperatureTwo);
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
    writeAllDataToFile("Temperature Probe Two (°C): ", temp2);
    delay(500);
    
    ADS1148_set_ADC_for_sensor_three();
    rawTemperatureThree = ADS1148_read_data_once();
    //writeIntegerToFile(rawTemperatureThree);
    Serial.print("Sensor 3 raw data: ");
    Serial.println(rawTemperatureThree);
    writeAllDataToFile("Sensor 3 raw data: ", rawTemperatureThree);
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
    writeAllDataToFile("Temperature Probe Three (°C): ", temp3);
    delay(500);

    //Measure and display the AVDD
    ADS1148_set_ADC_for_AVDD_reading();
    AVDD = ADS1148_read_data_once();
    //Serial.println();
    Serial.print("AVDD: ");
    AVDD = AVDD/32767.0*ADC_EXTERNAL_REFERENCE_VOLTAGE*4.0;
    Serial.println(AVDD, 4);
    writeAllDataToFile("AVDD: ", AVDD);
    delay(300);
    
    //Measure and display the DVDD
    ADS1148_set_ADC_for_DVDD_reading();
    DVDD = ADS1148_read_data_once();
    //Serial.println();
    Serial.print("DVDD: ");
    DVDD = DVDD/32767.0*ADC_EXTERNAL_REFERENCE_VOLTAGE*4.0;
    Serial.println(DVDD, 4);
    writeAllDataToFile("DVDD: ", DVDD);
    delay(300);

    //Measure and display the Vref
    ADS1148_set_ADC_for_external_Vref_reading();
    double Vref = ADS1148_read_data_once();
    Serial.print("Vref: ");
    Vref = Vref/32767.0*ADC_INTERNAL_REFERENCE_VOLTAGE*4.0;
    Serial.println(Vref, 4);
    writeAllDataToFile("Vref: ", Vref);
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
    writeAllDataToFile("Acc_x: ", accelerometer_x);
    writeAllDataToFile("Acc_y: ", accelerometer_y);
    writeAllDataToFile("Acc_z: ", accelerometer_z);
    
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
    writeAllDataToFile("Mag_x: ", magnetometer_x);
    writeAllDataToFile("Mag_y: ", magnetometer_y);
    writeAllDataToFile("Mag_z: ", magnetometer_z);
    if (dof.magGetOrientation(SENSOR_AXIS_Z, &event, &orientation)){
      /* 'orientation' should have valid .heading data now */
      DOF_heading = orientation.heading;
      Serial.print(F("Heading: "));
      Serial.print(DOF_heading);
      Serial.println(F("; "));
      writeAllDataToFile("DOF_heading: ", DOF_heading);
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
    writeAllDataToFile("Gyro_x: ", gyroscope_x);
    writeAllDataToFile("Gyro_y: ", gyroscope_y);
    writeAllDataToFile("Gyro_z: ", gyroscope_z);
    
    /* Display the pressure sensor results (barometric pressure is measure in hPa) */
    bmp.getEvent(&event);
    if (event.pressure) {
      /* Display atmospheric pressure in hPa */
      DOF_pressure = event.pressure;
      Serial.print(F("Pressure "));
      Serial.print(DOF_pressure);
      Serial.print(F(" hPa, "));
      writeAllDataToFile("DOF_pressure: ", DOF_pressure);
      /* Display ambient temperature in C */
      float temperature;
      bmp.getTemperature(&temperature);
      DOF_temperature = temperature;
      Serial.print(temperature);
      Serial.println(F("°C "));
      writeAllDataToFile("DOF_temperature: ", temperature);
    
    }
    
    Serial.println();
    Serial.println("10DOF data retrieval complete...");
    Serial.println();
    
    //Concats NMEA string and three temperatures together //AND NEW STUFF!!
    concatAllData(temp1, temp2, temp3, ozonePPB, AVDD, DVDD);
    
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

      //UNCOMMENT THIS WHEN READY TO TRANSMIT TO SATELLITES
      int err = isbd.sendSBDTextLARGE(NMEAstringToSend1);
      if (err != 0) {
        Serial.print("sendSBDText failed: error ");
        Serial.println(err);
        return;
      }


      Serial.println("Data transmitted...");
      Serial.println();
      newGPSdataReady = false;
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


/*
 * Function to calculate Ozone in Parts Per Billion
 */
double convertOzoneRawToPPB(int16_t rawOzoneLevel){
    double resistance = ((SENSOR_INPUT_VOLTAGE*R1*ADC_MAX_VALUE)/(ADC_INTERNAL_REFERENCE_VOLTAGE * rawOzoneLevel)) - R1 - R2;
    int k_ohms = resistance/1000.0;
    double returnValue = (0.000000075 * pow(k_ohms,3) + (0.0000222 * pow(k_ohms, 2)) + (0.0570806 * k_ohms) - (0.31211));
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
 Negative Input - AIN4
 */
void ADS1148_set_ADC_for_ozone_sensor() {
    ADS1148_writeRegister(MultiplexerControlRegister0_00h, 0x34); //0b00 11_0 100
    ADS1148_writeRegister(BiasVoltageRegister_01h, 0x00); //0b0000_0000
    ADS1148_writeRegister(MultiplexerControlRegister1_02h, 0x30); //0b0 01 1_0 000
    ADS1148_writeRegister(SystemControlRegister0_03h, 0x00); //0b0000_0000
    //4,5,6 are OFC
    //7,8,9 are FSC
    ADS1148_writeRegister(IDACControlRegister0_0Ah, 0x00); //0b1001_ 0 000 IDAC off
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
 Function to set registers to read external voltage reference
 */
void ADS1148_set_ADC_for_external_Vref_reading() {
    ADS1148_writeRegister(MultiplexerControlRegister1_02h, 0x25); //0b0 01 0_0 000
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

