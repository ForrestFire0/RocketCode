#include <SoftwareSerial.h>

int ledPin =  13; //Status LED connected to digital pin 13

SoftwareSerial Logger(2, 3);

void setup()
{
  pinMode(ledPin, OUTPUT);

  Logger.begin(115200); //9600bps is default for OpenLog
  //Logger.begin(513600); //Much faster serial, used for testing buffer overruns on OpenLog
  delay(3000);
  
  int x = 0;
  while (x < 10) {
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
    delay(50);
    x++;
  }
  delay(1000); //Wait a second for OpenLog to init

  for(int i = 0; i < 5; i++) {
    Logger.println("Testing Logger");
    delay(50);
  }
  stopLogging();
}

void loop()
{
  //Blink the Status LED because we're done!
  digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(ledPin, LOW);
  delay(100);
  digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(ledPin, LOW);
  delay(1000);
}

void stopLogging() {
  Logger.write(26);
  Logger.write(26);
  Logger.write(26);
}
