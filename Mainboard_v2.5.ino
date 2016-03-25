
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
/***************************************************************************************/


/*
    @NOTES
    -Must be set to 24MHz (non-optimized) prior to compiling/programming
    -DO NOT USE pins 7 or 8 breakouts, those are connected to GPS RX/TX
    
*/


/*
    @TODO
    -Finish adding ADC code
    -Add LEDs and LEDpins: 5,6,9,22,23
      Main Power Status (red) (5)
      GPS fix (red) (6)
      Iridium waiting to transmit (callback function, red) (9)
      Iridium transmitted successfully (green) (22)
      SD card writing (red) (23)
    -Get every GPS string to save to SD card
*/


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

const int LED_Main_Power = 5; // RED
const int LED_GPS_fix = 6; // RED
const int LED_Iridium_waiting_to_transmit = 9; // RED (callback function)
const int LED_Iridium_transmitted_successfully = 22; // GREEN
const int LED_SD_writing = 23; // GREEN //Eliminate

SoftwareSerial ssIridium(0, 1); // RX, TX
IridiumSBD isbd(ssIridium, 10);


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
  if (!SD.begin(SDchipSelect)) {
    Serial.println("Card failed, or not present...");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  Serial.println();

  char* name1 = "datalog.txt";
  File myFile = SD.open(name1, FILE_WRITE);
  if (myFile) {
    myFile.println("***************GPS_DATA_LOGGER******************");
    myFile.close();
    Serial.println(NMEAstringToSend);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
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
    //Serial.println(dataToWrite);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
    Serial.println();
  }
}


void concatAllData(double sensorOneReading, double sensorTwoReading, double sensorThreeReading) {
  String temp = "";
  temp = NMEAstringToSend + "I" + String(sensorOneReading) + "E" + String(sensorTwoReading) + "W" + String(sensorThreeReading);
  NMEAstringToSend = (char*) temp.c_str();
}


void setup() {

  int signalQuality = -1;

  pinMode(LED_Main_Power, OUTPUT);
  digitalWrite(LED_Main_Power, LOW);
  pinMode(LED_GPS_fix, OUTPUT);
  digitalWrite(LED_GPS_fix, LOW);
  pinMode(LED_Iridium_waiting_to_transmit, OUTPUT);
  digitalWrite(LED_Iridium_waiting_to_transmit, LOW);
  pinMode(LED_Iridium_transmitted_successfully, OUTPUT);
  digitalWrite(LED_Iridium_transmitted_successfully, LOW);
  pinMode(LED_SD_writing, OUTPUT);
  digitalWrite(LED_SD_writing, LOW);

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

  delay(50);
  Serial3.begin(9600);
  delay(500);
  InitializeGPSModule();
  delay(1000);

  flashRate = 1000;

  SPI.begin();
  delay(1000);

  InitializeSDCard();
  delay(1000);

  ADS1148_reset();
  delay(500);


  Serial.println("ADS1148 Printing Register Values.");
  ADS1148_read_and_print_all_registers();
  Serial1.println("");
  delay(520);

  Serial1.println("");
  Serial1.println("");
  Serial1.println("########################################");
  Serial1.println("#.......#COUGBALLOON INITIALIZED.......#");
  Serial1.println("########################################");
  Serial1.println("");
  Serial1.println("");

}

void loop() {

  Serial3.end();

  ADS1148_set_ADC_for_sensor_one();
  rawTemperatureOne = ADS1148_read_data_once();
  //writeIntegerToFile(rawTemperatureOne);
  //Serial.print("Sensor 1 raw data: ");
  //Serial.println(rawTemperatureOne);
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
  //Serial.print("Sensor 2 raw data: ");
  //Serial.println(rawTemperatureTwo);
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
  //Serial.print("Sensor 3 raw data: ");
  //Serial.println(rawTemperatureThree);
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

  Serial.println();

  Serial3.begin(9600);
  delay(15000);//Delay 15 seconds, long enough to capture new GPS data

  //Concats NMEA string and three temperatures together
  concatAllData(temp1, temp2, temp3);

  Serial3.end();

  //Send GPS data if new data has arrived
  if (newGPSdataReady) {

    Serial.println("Beginning transmission...");
    Serial.println();

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
    Serial.println("Data saved...");
    Serial.println();

//    int err = isbd.sendSBDTextLARGE(NMEAstringToSend1);
//    if (err != 0) {
//      Serial.print("sendSBDText failed: error ");
//      Serial.println(err);
//      return;
//    }
    Serial.println("Data transmitted...");
    Serial.println();
    newGPSdataReady = false;

  }

  Serial3.begin(9600);
  delay(30000);

}


bool ISBDCallback() {

  //In this function, put what you want to happen during RockBLOCK operations.
  digitalWrite(LED_Iridium_waiting_to_transmit, (millis() / flashRate) % 2 == 1 ? HIGH : LOW);

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
   Positive Input - AIN0 (IDAC source)
   Negative Input - AIN7
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
   Positive Input - AIN0 (IDAC source)
   Negative Input - AIN5
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
   Positive Input - AIN0 (IDAC source)
   Negative Input - AIN3
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

