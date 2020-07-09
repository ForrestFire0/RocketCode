/*
 Name: Sketch1.ino
 Created: 3/11/2020 10:42:17 PM
 Author: Chris Milner

  Connections:
    Blue: GND (-)
    Brow: Plus (+)
    Black: Signal

   PWM:
     1278 - 150;
     1327 - 400
 
*/
#include <CRC.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <PID_v1.h>

Servo allTraxController;

#define INPUT_PIN 2
#define PWM_OUTPUT_PIN 3

#define OUTPUT_MSG_HZ 25
unsigned long lastOutputMessageTime;

double Commanded_RPM, Current_RPM, Output_PWM;
//Specify the links and initial tuning parameters
double 
  Kp = 0.01, 
  Ki = 0.05, 
  Kd = 0;
PID myPID(&Current_RPM, &Output_PWM, &Commanded_RPM, Kp, Ki, Kd, DIRECT);

unsigned long lastRevolutionTime;
unsigned long lastPIDLoopTime;

bool x;

void change() {
    x = true;
    return;
    unsigned long _micros = micros();
    // Deals with long data type overflow.
    if (_micros < lastRevolutionTime) {
        lastRevolutionTime = _micros;
    }
    else {
        Current_RPM = (60000000.0 / (_micros - lastRevolutionTime));
    }
    lastRevolutionTime = _micros;
}

// the setup function runs once when you press reset or power the board
void setup() {

    Current_RPM = 0;
    Commanded_RPM = 0;
    Output_PWM = 0;
    lastOutputMessageTime = 0;
    lastPIDLoopTime = 0;

    Serial.begin(115200);

    pinMode(INPUT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INPUT_PIN), change, RISING);

    allTraxController.attach(PWM_OUTPUT_PIN);
    allTraxController.write(Output_PWM/100*180.0);

    //turn the PID on
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(50);   // Sample Time in milliseconds -> 20 Hz
    myPID.SetOutputLimits(0, 100);
}

// the loop function runs over and over again until power down or reset
void loop() {
  if(x) Serial.println("Change");
  x = false;
  return;
    // if there are incoming serial bytes, then read and process
    if(Serial.available() > 0) {
        char receivedChar = Serial.read();
        while(Serial.available() > 0) {
          Serial.read();
        }
        if(receivedChar == 'q') { //Stop the mower
          Commanded_RPM = 0;
        }
        else if(receivedChar == 'a') { //Lower the speed of the mover by a bunch.
          Commanded_RPM -= 50;
        }
        else {
          Commanded_RPM += 50;
        }
    }
    unsigned long _micros = micros();

    //If the time since the last change has been more than .1 seconds, calculate the RPM as if the thing ticked now.
    if(_micros - lastRevolutionTime > 100000) {
      //Deal with long data type overlfow.
      if (_micros < lastRevolutionTime) {    
          lastRevolutionTime = _micros;
      } 
      Current_RPM = (60000000.0 / (_micros - lastRevolutionTime));
      //if RPM is very low, set to 0.
      if(Current_RPM < 50) Current_RPM = 0;
    }
    
    // PID loop
    myPID.Compute();
    Serial.print("Current: ");
    Serial.print(Current_RPM);
    Serial.print("  Target: ");
    Serial.print(Commanded_RPM);
    Serial.print("  Output_PWM: ");
    Serial.println(min(Output_PWM * 9 + 1100, 1400));
    allTraxController.writeMicroseconds(min(Output_PWM * 9 + 1100, 1400));
    delay(50);
}
