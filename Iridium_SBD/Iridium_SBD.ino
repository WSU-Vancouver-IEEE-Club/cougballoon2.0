

#include <IridiumSBD.h>
#include <SoftwareSerial.h>
 
SoftwareSerial ssIridium(18, 19);// RX, TX
IridiumSBD isbd(ssIridium, 10);
static const int ledPin = 13;

void setup()
{
  int signalQuality = -1;

  pinMode(ledPin, OUTPUT);

  Serial.begin(115200);
  ssIridium.begin(19200);

  isbd.attachConsole(Serial);
  isbd.setPowerProfile(1);
  isbd.begin();

  isbd.setMinimumSignalQuality(1);//ADDED BY MIKE

  int err = isbd.getSignalQuality(signalQuality);
  if (err != 0)
  {
    Serial.print("SignalQuality failed: error ");
    Serial.println(err);
    return;
  }

  Serial.print("Signal quality is ");
  Serial.println(signalQuality);

  err = isbd.sendSBDText("Hello, world!");
  //err = isbd.sendSBDText("This little message went to space.");
  //err = isbd.sendSBDText("03271a081700152e177f141c00a200d1016401d401d6fefa244750524d432c3132333531392c412c343830372e3033382c4e2c30313133312e3030302c452c3032322e342c3038342e342c3233303339342c3030332e312c572a36412447504747412c3132333531392c343830372e3033382c4e2c30313133312e3030302c452c312c30382c302e392c3534352e342c4d2c34362e392c4d2c2c2a3437");
  //err = isbd.sendSBDText("Something came in the mail today...");
  if (err != 0)
  {
    Serial.print("sendSBDText failed: error ");
    Serial.println(err);
    return;
  }

  Serial.println("Hey, it worked!");
  Serial.print("Messages left: ");
  Serial.println(isbd.getWaitingMessageCount());
}

void loop()
{
   digitalWrite(ledPin, HIGH);
   
}

bool ISBDCallback()
{
   digitalWrite(ledPin, (millis() / 1000) % 2 == 1 ? HIGH : LOW);
   return true;
}
