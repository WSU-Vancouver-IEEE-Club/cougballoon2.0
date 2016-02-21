//#include <TinyGPS.h> // Not needed.
#include <SPI.h>
#include <SD.h>

// different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
// Note that these only control the rate at which the position is echoed, to actually speed up the
// position fix you must also send one of the position fix rate commands below too.
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

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
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


String NMEAstring = ""; //Will hold both GGA and RMC strings together.
char c;
int counter = 0;
File myFile;


/*
 * Set up Serial Ports
 */
void InitializeSerialPorts() {
  Serial.begin(9600);
  delay(1000);
  Serial1.begin(9600);
  delay(50);
  pinMode(13,OUTPUT);
}

/*
 * Set up GPS Modules
 */
void InitializeGPSModule() {
   Serial1.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //Serial1.println(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  delay(50);
  Serial1.println(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(50);
  Serial1.println(PMTK_API_SET_FIX_CTL_1HZ);
  delay(50);
  Serial1.println(PGCMD_NOANTENNA);
  delay(50);
  //Serial1.println(PMTK_ENABLE_SBAS);
  delay(50);
  //Serial1.println(PMTK_ENABLE_WAAS);
  delay(50);
}

/*
 * Initialize serial to pin 13
 */
void serialEvent1() {
  digitalWrite(13, HIGH);
  //Serial.println("EVENT!");
  getNMEAstring();
  digitalWrite(13, LOW);
}

/*
 * Read in GPS data and store in variable 
 */
void getNMEAstring() {
  NMEAstring = "";
  while (Serial1.available()) {
    c = Serial1.read();
    NMEAstring.concat(c); 
    delayMicroseconds(1200);
  } 
}

/*
 * Set up Writing and reading to SD card
 */
void InitializeSDCard() {
   Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(10)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  File myFile = SD.open("datalog.txt", FILE_WRITE);
    if (myFile) {
    myFile.println("***************GPS_DATA_LOGGER******************");
    myFile.close();
    Serial.println(NMEAstring);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

/*
 * Log to file all GPS data needed
 */
void writeToFile() {
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File myFile = SD.open("datalog.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (myFile) {
    myFile.print("Data ID Number: ");
    myFile.println(counter);
    myFile.println(NMEAstring);
    counter++;
    myFile.close();
    // print to the serial port too:
    Serial.println(NMEAstring);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

void setup() {
  InitializeSerialPorts();
  InitializeGPSModule();
  while(!Serial);
  while(!Serial1);
  InitializeSDCard();
}

void loop() {
  delay(3000);
  writeToFile();
}



