/***********************************************/
// Written for Teensy 3.1/3.2
// Emanuel Brici, Cody Glascoe, Zach Moore, Rachael Campbell
// #cougballoon
// Iridium Transmitter
// v1.0 Mar 5 2015
//
//
//
/***********************************************/

#include <IridiumSBD.h>
#include <SoftwareSerial.h>

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
char* oldNMEAstring = ""; 
char c;

bool rockBLOCKready = false;
bool newData = false;

SoftwareSerial ssIridium(0, 1); // RX, TX
IridiumSBD isbd(ssIridium, 10);

static const int ledPin = 13;


/*
 * Set up GPS Modules
 */
void InitializeGPSModule() {
   Serial3.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
//=======
  Serial3.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
//>>>>>>> Stashed changes
  //Serial1.println(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  delay(50);
  Serial3.println(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(50);
  Serial3.println(PMTK_API_SET_FIX_CTL_1HZ);
  delay(50);
  Serial3.println(PGCMD_NOANTENNA);
  delay(50);
  //Serial1.println(PMTK_ENABLE_SBAS);
  delay(50);
  //Serial1.println(PMTK_ENABLE_WAAS);
  delay(50);
}


///*
// * Read in GPS data and store in variable 
// */
//void getNMEAstring() {
//  NMEAstring = "";
//  while (Serial3.available()) {
//    c = Serial3.read();
//    NMEAstring.concat(c); 
//    delayMicroseconds(1200);
//  } 
//  NMEAstring.toCharArray(oldNMEAstring, NMEAstring.length());
//}

/*
 * Initialize serial to pin 13
 */
void serialEvent3() {

  if (rockBLOCKready) {
    //digitalWrite(ledPin, HIGH);
    Serial.println("serialEvent3 occured.");
    //getNMEAstring();
    NMEAstring = "";
    while (Serial3.available()) {
      c = Serial3.read();
      NMEAstring.concat(c); 
      delayMicroseconds(1200);
    } 
    //Serial.println("before the toCharArray");
    oldNMEAstring=(char*)NMEAstring.c_str();
    //Serial.println("after the toCharArray");
    //Serial.println(NMEAstring);
    //Serial.println(oldNMEAstring);
    newData = true;
  } 
}


void setup(){
  
  int signalQuality = -1;

  pinMode(ledPin, OUTPUT);

  Serial.begin(115200);
  ssIridium.begin(19200);
  Serial3.begin(9600);
  //while(!Serial3);

  InitializeGPSModule();
  digitalWrite(ledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(30);               // wait for a second
  digitalWrite(ledPin, LOW);    // turn the LED off by making the voltage LOW
  delay(30);               // wait for a second

  /*
   * These allow the host application to provide a Stream object (serial port) that can be used to 
   * monitor the RockBLOCK serial traffic and diagnostic messages, respectively.  The typical usage 
   * is to simply use the Arduino serial port to monitor both of these—assuming that it is connected 
   * to a PC serial console and not otherwise used:
   */
  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);

  /*
   * 1 for low-current USB power source, 0 for default power. This method defines the internal delays 
   * between retransmission.  Low current applications need longer delays.
   */
  //isbd.setPowerProfile(1); //Use this when connected to USB Serial Cable direct to computer
  isbd.setPowerProfile(0); //Use this when connected to Blue Battery

  /*
   * begin() also serves as the way to wake a RockBLOCK that is asleep. At initial power up, this 
   * method make take several tens of seconds as the device charges.  When waking from sleep the 
   * process should be faster. If provided, the user’s ISBDCallback function is repeatedly called 
   * during this operation. This function should be called before any transmit/receive message.
   */
  isbd.begin();

  /*
   * The minimum signal quality on a scale of 0 (nonexistent) to 5 (superb) needed before the library 
   * allows a transmission to begin. (default=2) (AT+CSQ)
   */
  isbd.setMinimumSignalQuality(1);

  /*
   * If provided, the user’s ISBDCallback function is repeatedly called during this operation. 
   * This method is mostly informational.  It is not strictly necessary for the user application 
   * to verify that a signal exists before calling one of the transmission functions, as these 
   * check signal quality themselves.
   */
  int err = isbd.getSignalQuality(signalQuality);
  if (err != 0) {
    Serial.print("SignalQuality failed: error ");
    Serial.println(err);
    return;
  }

  Serial.print("Signal quality is ");
  Serial.println(signalQuality);

  delay(3000);
  rockBLOCKready = true;  

}

void loop(){

  //Varying LED delays for testing since serial port not working.
  Serial.println(newData);

  if (newData) {
    //noInterrupts();
    //cli();
    
    rockBLOCKready = false;
    Serial.println("ATTEMPTING TO SEND!!!!");
    //const char* test = "A1B2C3D4";
    //isbd.begin();
    //const char* testNMEAstring = "$GPGGA,135624.000,4541.6478,N,12234.2998,W,2,07,1.18,110.6,M,-19.1,M,0000,0000*52";
    //const char* testNMEAstring = "$GPGGA,135624.000,4541.6478,N,12234.2998,W,2,07";//,1.18,110.6,M,-19.1,M,0000,0000*52";
    const char* testNMEAstring = "ABCDEFGHIJKLMNOPQRSTUVWXYZ1";
    //Serial.println(testNMEAstring);
    int err = isbd.sendSBDText(testNMEAstring);
    //int err = isbd.sendSBDText("ABCDEFG");
    //int err = isbd.sendSBDText("03271a081700152e177f141c00a200d1016401d401d6fefa244750524d432c3132333531392c412c343830372e3033382c4e2c30313133312e3030302c452c3032322e342c3038342e342c3233303339342c3030332e312c572a36412447504747412c3132333531392c343830372e3033382c4e2c30313133312e3030302c452c312c30382c302e392c3534352e342c4d2c34362e392c4d2c2c2a3437");
    if (err != 0) {
      Serial.print("sendSBDText failed: error ");
      Serial.println(err);
      return;
    }
    //interrupts();
    //sei();
    Serial.println("It should have sent....");
    newData = false;
    rockBLOCKready = true;
  }

  

  //Serial.println("Hey, it worked!");
  //delay(60000);
  //for (int i = 0;i < 30;i++) {
  for (int i = 0;i < 3;i++) {
      digitalWrite(ledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);               // wait for a second
      digitalWrite(ledPin, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);               // wait for a second
  }
}

bool ISBDCallback(){

  //In this function, put what you want to happen during RockBLOCK operations.
  //digitalWrite(ledPin, (millis() / 1000) % 2 == 1 ? HIGH : LOW);
  digitalWrite(ledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(300);               // wait for a second
  digitalWrite(ledPin, LOW);    // turn the LED off by making the voltage LOW
  delay(300);               // wait for a second
  return true;
   
}
