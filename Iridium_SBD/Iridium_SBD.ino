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
 
SoftwareSerial ssIridium(0, 1);// RX, TX
IridiumSBD isbd(ssIridium, 10);

static const int ledPin = 13;

void setup(){
  
  int signalQuality = -1;

  pinMode(ledPin, OUTPUT);

  Serial.begin(115200);
  ssIridium.begin(19200);

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

}

void loop(){

  //Varying LED delays for testing since serial port not working.
  
  int err = isbd.sendSBDText("Testing in the middle of the night.");
  //int err = isbd.sendSBDText("03271a081700152e177f141c00a200d1016401d401d6fefa244750524d432c3132333531392c412c343830372e3033382c4e2c30313133312e3030302c452c3032322e342c3038342e342c3233303339342c3030332e312c572a36412447504747412c3132333531392c343830372e3033382c4e2c30313133312e3030302c452c312c30382c302e392c3534352e342c4d2c34362e392c4d2c2c2a3437");
  
  if (err != 0) {
    Serial.print("sendSBDText failed: error ");
    Serial.println(err);
    return;
  }

  for (int i = 0;i < 1000;i++) {
    digitalWrite(ledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(15);               // wait for a second
    digitalWrite(ledPin, LOW);    // turn the LED off by making the voltage LOW
    delay(15);               // wait for a second
  }

  Serial.println("Hey, it worked!");
  //delay(60000);
  for (int i = 0;i < 30;i++) {
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
