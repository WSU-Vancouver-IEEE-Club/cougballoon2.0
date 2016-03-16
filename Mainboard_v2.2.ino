
/***********************************************/
// Written for Teensy 3.1/3.2
// Emanuel Brici, Cody Glascoe, Zach Moore, Rachael Campbell, Mike Hamilton
// #cougballoon
// Iridium Transmitter
// v1.0 Mar 5 2016
// v2.0 Mar 11 2016 New board
// v2.1 Mar 11 2016 Added support for SD card/ADC/GPS
// v2.2 Mar 15 2016 Added full support for ADS1148 ADC
/***********************************************/

/*
 * @TODO
 * -Fix datalogging function, conbine all three into one, make it so we know what info we are writing
 * -Concatenate all data into one long string, then transmit
 * 
 */

#include <IridiumSBD.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>

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

#define NoOp                                            0xFF

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

String NMEAstring = ""; //Will hold both GGA and RMC strings together.
char* NMEAstringToSend = "";
char c;

bool rockBLOCKready = false;
bool newGPSdataReady = false;
bool ADCinactive = true;
bool GPSbusy = false;


int flashRate = 100;

const int ADCchipSelect = 10;
const int startPin = 9;
static const int START = 3;

const int SDchipSelect = 15;

int initialRegisterContents[15];

const int LED = 13;

SoftwareSerial ssIridium(0, 1); // RX, TX
IridiumSBD isbd(ssIridium, 10);

static const int ledPin = 13;


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
  //Serial3.println(PGCMD_NOANTENNA);
  delay(50);
  Serial3.println(PMTK_ENABLE_SBAS);
  delay(50);
  Serial3.println(PMTK_ENABLE_WAAS);
  delay(50);
  //Serial3.println(PMTK_Q_RELEASE);
  delay(50);
}


/*
   Initialize serial to pin 13
*/
void serialEvent3() {

  if (rockBLOCKready) {
    GPSbusy = true;
    Serial.print("Incoming GPS data: ");
    NMEAstring = "";
    while (Serial3.available()) {
      c = Serial3.read();
      NMEAstring.concat(c);
      delayMicroseconds(1200);
    }
    NMEAstringToSend = (char*)NMEAstring.c_str();
    Serial.println(NMEAstringToSend);
    newGPSdataReady = true;
  }
  GPSbusy = false;
}


/*
   Set up Writing and reading to SD card
*/
void InitializeSDCard() {

  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(SDchipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

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
void writeCharStarToFile(char* dataToWrite) {

  Serial.println("Writing to file");
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
    Serial.println(dataToWrite);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

/*
   Log to file all GPS data needed
*/
void writeStringToFile(String dataToWrite) {
  Serial.println("Writing to file");
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
    Serial.println(dataToWrite);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

/*
   Log to file all GPS data needed
*/
void writeIntegerToFile(int dataToWrite) {

  Serial.println("Writing to file");
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
    Serial.println(dataToWrite);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}



void setup() {

  int signalQuality = -1;

  pinMode(ledPin, OUTPUT);

  digitalWrite(ADCchipSelect, HIGH);
  pinMode(ADCchipSelect, OUTPUT);

  digitalWrite(startPin, LOW);
  pinMode(startPin, OUTPUT);

  pinMode(START, OUTPUT);
  digitalWrite(START, HIGH);

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
  isbd.attachDiags(Serial);

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
    return;
  }

  Serial.print("Signal quality is ");
  Serial.println(signalQuality);

  delay(50);
  Serial3.begin(9600);
  delay(500);
  InitializeGPSModule();
  delay(1000);

  rockBLOCKready = true;
  //rockBLOCKready = false;
  flashRate = 1000;

  SPI.begin();
  delay(1000);

  InitializeSDCard();
  delay(1000);

  ADS1148_self_offset_calibration();
  delay(3500);

  Serial.println("#cougballoon initialized......");

}

void loop() {

  //Send GPS data
  //  if (newGPSdataReady) {
  //
  //    //TEST AND SEE IF WE HAVE A FIX, IF NOT, DO NOT TRAMSIT THE INFO
  //    rockBLOCKready = false;
  //
  //    Serial.println("Beginning transmission...");
  //
  //    int len = strlen(NMEAstringToSend);
  //    for (int i = 0;i < len;i++) {
  //      if (NMEAstringToSend[i] == '\r') {
  //        //Serial.println("CR!!");
  //        NMEAstringToSend[i] = '#';
  //      }
  //      if (NMEAstringToSend[i] == '\n') {
  //        //Serial.println("NL!!");
  //        NMEAstringToSend[i] = '!';
  //      }
  //    }
  //    NMEAstringToSend[len] = '\0';
  //    Serial.print("Send this: ");
  //    Serial.println(NMEAstringToSend);
  //
  //    writeStringToFile(NMEAstringToSend);
  //    Serial.println("Data saved...");
  //
  //    int err = isbd.sendSBDTextLARGE(NMEAstringToSend);
  //    if (err != 0) {
  //      Serial.print("sendSBDText failed: error ");
  //      Serial.println(err);
  //      return;
  //    }
  //    Serial.println("Data transmitted...");
  //    newGPSdataReady = false;
  //    rockBLOCKready = true;
  //
  //  }




  Serial3.end();

  //Write GPS data to SD card
  if (newGPSdataReady) {
    newGPSdataReady = false;
    writeStringToFile(NMEAstringToSend);
  }


  ADS1148_set_ADC_for_sensor_one();
  int16_t sensorOneReading = 0;
  sensorOneReading = ADS1148_read_data_once();
  if (sensorOneReading) {
    writeIntegerToFile(sensorOneReading);
  }
  Serial.print("Sensor 1: ");
  Serial.println(sensorOneReading);
  delay(210);


  ADS1148_set_ADC_for_sensor_two();
  int16_t sensorTwoReading = 0;
  sensorTwoReading = ADS1148_read_data_once();
  if (sensorTwoReading) {
    writeIntegerToFile(sensorTwoReading);
  }
  Serial.print("Sensor 2: ");
  Serial.println(sensorTwoReading);
  delay(210);


  ADS1148_set_ADC_for_sensor_three();
  int16_t sensorThreeReading = 0;
  sensorThreeReading = ADS1148_read_data_once();
  if (sensorThreeReading) {
    writeIntegerToFile(sensorThreeReading);
  }
  Serial.print("Sensor 3: ");
  Serial.println(sensorThreeReading);
  delay(210);

  Serial3.begin(9600);
  delay(10000);


  //  DELAY FUNCTION
  //  for (int i = 0;i < 30;i++) {
  //      digitalWrite(ledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
  //      delay(1000);               // wait for a second
  //      digitalWrite(ledPin, LOW);    // turn the LED off by making the voltage LOW
  //      delay(1000);               // wait for a second
  //  }

}


bool ISBDCallback() {

  //In this function, put what you want to happen during RockBLOCK operations.
  //digitalWrite(ledPin, (millis() / flashRate) % 2 == 1 ? HIGH : LOW);

  return true;
}



/*
   Wakes up ADS1148 from sleep mode.
*/
void ADS1148_wakeup() {
  digitalWrite(startPin, HIGH);//Start pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  SPI.transfer(0x00);//Can be 0x01 also.
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(startPin, LOW);
}

/*
   Sets the ADS1148 to sleep mode.
*/
void ADS1148_sleep() {
  digitalWrite(startPin, HIGH);//Start pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  SPI.transfer(0x02);//Can be 0x03 also.
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(startPin, LOW);
}


/*
   Synchronize the A/D conversion
*/
void ADS1148_sync() {
  digitalWrite(startPin, HIGH);//Start pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  SPI.transfer(0x04);//Can be 0x05 also.
  SPI.transfer(0x04);//Can be 0x05 also.
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(startPin, LOW);
}

/*
   Wakes up ADS1148 from sleep mode.
*/
void ADS1148_reset() {
  digitalWrite(startPin, HIGH);//Start pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  SPI.transfer(0x00);//Can be 0x01 also.
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(startPin, LOW);
}




/*
   Reads the current data from the ADS1148.
*/
uint16_t ADS1148_read_data_once() {
  digitalWrite(startPin, HIGH);//Start pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
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
  digitalWrite(startPin, LOW);
  return returnValue;
}


/*
   Reads one register at a time.
*/
int ADS1148_readRegister(byte registerNumber) {
  digitalWrite(startPin, HIGH);//Start pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  SPI.transfer(0x20 | registerNumber);
  SPI.transfer(0x00);
  int returnValue = SPI.transfer(NoOp);
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(startPin, LOW);
  return returnValue;
}

/*
   Writes to one register at a time.
*/
void ADS1148_writeRegister(byte registerNumber, byte value) {
  digitalWrite(startPin, HIGH);//Start pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  SPI.transfer(0x40 | registerNumber);
  //Serial.print("Writing: 0x");
  //Serial.print(value, HEX);
  //Serial.print(" to register:");
  //Serial.println(registerNumber,HEX);
  SPI.transfer(0x00);
  SPI.transfer(value);
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(startPin, LOW);
  delay(200);
}

/*
   System offset calibration.
   This command initiates a system offset calibration. For a system offset calibration, the input
   should be externally set to zero. The OFC register is updated when this operation completes.
*/
void ADS1148_system_offset_calibration() {
  digitalWrite(startPin, HIGH);//Start pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  SPI.transfer(0x60);
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(startPin, LOW);
}

/*
   System gain calibration.
   This command initiates the system gain calibration. For a system gain calibration, the input
   should be set to full-scale. The FSC register is updated after this operation.
*/
void ADS1148_system_gain_calibration() {
  digitalWrite(startPin, HIGH);//Start pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  SPI.transfer(0x61);
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(startPin, LOW);
}

/*
   Self offset calibration.
   This command initiates a self-calibration for offset. The device internally shorts the inputs
   and performs the calibration. The OFC register is updated after this operation.
*/
void ADS1148_self_offset_calibration() {
  digitalWrite(startPin, HIGH);//Start pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  SPI.transfer(0x62);
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(startPin, LOW);
}

/*
   Function to read and return value from Sensor 1
   Positive Input - AIN0 (IDAC source)
   Negative Input - AIN7
*/
void ADS1148_set_ADC_for_sensor_one() {
  digitalWrite(startPin, HIGH);//Start pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  ADS1148_writeRegister(MultiplexerControlRegister0_00h, 0x38); //0b00 11_1 000
  ADS1148_writeRegister(BiasVoltageRegister_01h, 0x00); //0b0000_0000
  ADS1148_writeRegister(MultiplexerControlRegister1_02h, 0x20); //0b0 01 0_0 000
  ADS1148_writeRegister(SystemControlRegister0_03h, 0x00); //0b0000_0000
  //4,5,6 are OFC
  //7,8,9 are FSC
  ADS1148_writeRegister(IDACControlRegister0_0Ah, 0x93); //0b1001 _0 011
  ADS1148_writeRegister(IDACControlRegister1_0Bh, 0x7F); //0b0111_ 1111
  //B,C,D are GPIO
  //delay(210);
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(startPin, LOW);
}

/*
   Function to read and return value from Sensor 2
   Positive Input - AIN0 (IDAC source)
   Negative Input - AIN5
*/
void ADS1148_set_ADC_for_sensor_two() {
  digitalWrite(startPin, HIGH);//Start pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  ADS1148_writeRegister(MultiplexerControlRegister0_00h, 0x28); //0b00 10_1 000
  ADS1148_writeRegister(BiasVoltageRegister_01h, 0x00); //0b0000_0000
  ADS1148_writeRegister(MultiplexerControlRegister1_02h, 0x20); //0b0 01 0_0 000
  ADS1148_writeRegister(SystemControlRegister0_03h, 0x00); //0b0000_0000
  //4,5,6 are OFC
  //7,8,9 are FSC
  ADS1148_writeRegister(IDACControlRegister0_0Ah, 0x93); //0b1001 _0 011
  ADS1148_writeRegister(IDACControlRegister1_0Bh, 0x5F); //0b0101_ 1111
  //B,C,D are GPIO
  //delay(210);
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(startPin, LOW);
}

/*
   Function to read and return value from Sensor 3
   Positive Input - AIN0 (IDAC source)
   Negative Input - AIN3
*/
void ADS1148_set_ADC_for_sensor_three() {
  digitalWrite(startPin, HIGH);//Start pin must be high communicate with registers
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADCchipSelect, LOW);
  ADS1148_writeRegister(MultiplexerControlRegister0_00h, 0x18); //0b00 01_1 000
  ADS1148_writeRegister(BiasVoltageRegister_01h, 0x00); //0b0000_0000
  ADS1148_writeRegister(MultiplexerControlRegister1_02h, 0x20); //0b0 01 0_0 000
  ADS1148_writeRegister(SystemControlRegister0_03h, 0x00); //0b0000_0000
  //4,5,6 are OFC
  //7,8,9 are FSC
  ADS1148_writeRegister(IDACControlRegister0_0Ah, 0x93); //0b1001_ 0 011
  ADS1148_writeRegister(IDACControlRegister1_0Bh, 0x3F); //0b0011_ 1111
  //B,C,D are GPIO
  //delay(210);
  digitalWrite(ADCchipSelect, HIGH);
  SPI.endTransaction();
  digitalWrite(startPin, LOW);
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

/*
   Capture and store initial register values after calibration.
*/
void ADS1148_save_initial_register_values() {
  for (int i = 0; i < 15; i++) {
    initialRegisterContents[i] = ADS1148_readRegister(i);
  }
}


/*
   CODE FROM HADLEY
   hhuj1j..liio98yru7r43wvbazzxwrtht43  2q2ghre31146uhjl

*/

