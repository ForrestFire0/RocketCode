/*

  This is the code running on the arduino nano (old bootloader). It was developed for the TARC rocket competition.

  Things to do:

  USE MAGNETOMETER???

  Fix the pitot library. It has some unnessary code in it.

  Get the Alt only 10 times a second. You could get the accel data every 0.01 seconds but what is already done is probably good. --> DONE (tested)

  Secondly, update caulculation methods to fit those from test 3. --> DONE (needs real flights to be tested).

  Test, because now the alt is gotten 20x a second, and the alt speed is calculated 10x a second.

  get the alt 20x a second. Calculate the speed based on the altimeter 10x a second.

*/

#include <LPS.h> //Pressure
#include <LSM303.h> //Accelerometer
#include <L3G.h> //gyro
#include <Wire.h> //talking to the stuff
#include <SoftwareSerial.h> //Talking to open logger
#include <MadgwickAHRS.h> //quaternions

//Things to screw with:
#define servoPin 9 //doesnt do anything rn
#define address 0x28
#define TICKSPEED_MS 20 //time between loops in miliseconds. (PROB DONT EDIT) (BUT YOU COULD  - no now you cant.) (we tested this. You can edit this. Currently, the tick takes 20 ms, so theoretically it could run 50 times a second. 
#define TICKSPEED_S (TICKSPEED_MS / 1000.0f) // this tickspeed in terms of seconds.
//UPDATE: the accelerometer run 50x a second (20ms). The alitude runs 20x a second (50ms). The speed calculatoin for the altitude runs 10x a second.

#define targetAlt 800 //Target altitude of the launch.
#define sensitivity 1 //Multiplier for servo control.
#define gravity 32.17f //gravitational constant.
#define alt_alpha 0.25f //determined by spread sheets
#define speed_alpha 0.2427f //determinded by spread sh
#define VtSq 7103.91f

#define debugMode false//debug mode allows for all of the flight software to be running (aka, accelerometers no longer dictate orientation, also gryos stop being calibrated). You can tast flight stuff while still on the ground. Also, this sets communications to the serial monitor and not the logger.
#define printToScreen false //print to screen changes logger the screen if not in debug mode.

//Gyro Calibration Values.
#define gxOFFSET -0.13429
#define gyOFFSET 1.12185
#define gzOFFSET 2.17824

Madgwick filter; //creates a quaternion.
LPS ps; // PRESSURE SENSOR
LSM303 lsm; //accelerometer
L3G gyro; //gyro

#if (debugMode == true or printToScreen == true)
#define Logger Serial
#else
SoftwareSerial Logger(2, 3); //Logger
#endif

//Altitude
unsigned long tickTime;
float barAlt; //Most Recent Altitude
float smoothedBarAlt; //Smoothed altitude, 10 ticks old.
float avgBarSpd;

float ax, ay, az;
float gx, gy, gz;
float roll, pitch, yaw;
unsigned int count;

float upSpeed; //the best guess of the speed.
float upAcc;
float gravitySpeed;
float apoapsisAlt; //the guess of what the apoapsis hieght is going to be.

double pitotSpeed;

byte stage = 0;

void setup() {

  Logger.begin(115200);
  delay(10);
  Logger.println(F("Starting Startup.."));
  filter.begin(1000 / TICKSPEED_MS);
  // Altimiter Stuff
  Wire.begin();
  while (!ps.init())
  {
    Logger.println(F("pres sens fail"));
    while (1);
  }

  ps.enableDefault();
#if(debugMode==true)
  Serial.println("Warning, Debug Mode on");
#endif
  //Accelerometer Stuff
  lsm.init();
  lsm.enableDefault(); //AHHAHAHAHAHA ITS NOT ACTUALLY DEFAULT I SET IT TO 16G's

  if (!gyro.init())
  {
    Logger.println("Failed to autodetect gyro type!");
    while (1);
  }
  gyro.enableDefault();

  Logger.println(F("Calibration Offsets: "));
  Logger.print(F("gxOFFSET:"));
  Logger.println(gxOFFSET);
  Logger.print(F("gyOFFSET:"));
  Logger.println(gyOFFSET);
  Logger.print(F("gxOFFSET:"));
  Logger.println(gzOFFSET);

  Logger.println(F("Succesful Boot"));

  Logger.println(F("Place On Lanuch Pad....."));
}

void calculateSpeedAndAccel() {
  float pitotTrust;

  if (abs(gravitySpeed - avgBarSpd) > 50) { //the difference between the speeds is unreasonable.
#if (debugMode==false)
    Logger.println(F("Error. gravSpd and avgBarSpd dont match. Adjusting Vars"));
#endif
    gravitySpeed = gravitySpeed - ((gravitySpeed - avgBarSpd) / 2); //subtract difference between the two to get gravity speed closer to avgBarSpd.
  }

  if ((stage > 1) && (pitotSpeed > 10))  { //aka flying in the air
    pitotTrust = min(0.95, upSpeed / 50); // a scale on how much we trust the pitot tube.
  }
  else {
    pitotTrust = 0; //we dont trust it at all.
  }

  upSpeed = (gravitySpeed + avgBarSpd) / 2.0f; //add a second that guesses the speed based on the drag of the thing.

  upSpeed = (upSpeed * (1 - pitotTrust)) + (pitotSpeed * pitotTrust); //Upspeed is adjusted. If we are going slowly, the pitot is not to be trusted. If we are fast, the pitot trust score is higher, so we trust the pitot tube more on how fast we are going.

  float kinematicsGuess = smoothedBarAlt + (sq(upSpeed) / (2 * (gravity + (0.333333f * upAcc)))); //this does a rough estimate of the max speed we will hit. (the Bates way)

  float powGuess = smoothedBarAlt + (0.023f * pow(abs(upSpeed), 1.89f)); //uses the best guess of the speed to find how many more feet we have to go. (the google sheets way)

  float lnGuess = smoothedBarAlt + ((VtSq / (2 * gravity)) * log((sq(upSpeed) + VtSq) / VtSq)); //uses the official method of determining max hieght. (the nasa way)


  //This whole section  \/ \/ \/ \/ \/ \/ needs to be completly redone. But it doesnt really matter yet...
  /*
    float altError = apoapsisAlt - targetAlt; // the amount of error between the target and current projection. Positive is over.
    float spdMultiplier; // call this here so we can print it later.
    float errorMultiplier; //^^^^^^^^^^^^^^^^^

    if (altError < 5) { //We are too low. Retract the fins.
    //setAB(0);
    }
    else { //we are to high. Extend fins accordingly.
    spdMultiplier = min((3.1622776601f / (pow(upSpeed, 0.5f))), 1); // a multiplier to adjust for speed though the air. Higher speed means a lower spdMultiplier.
    errorMultiplier = pow(altError, 1.5) / 500; //makes more far our altitude values more sensitive.
    //setAB(min(max(sensitivity * errorMultiplier * spdMultiplier, 0), 1));
    }
  */

  Logger.print(F("Time "));
  Logger.print(millis());

  Logger.print(F(" RawAlt "));
  Logger.print(barAlt, 3);

  Logger.print(F(" RawAcc "));
  Logger.print(ax, 3);
  Logger.print(F(" Pitch "));
  Logger.print(pitch, 3);

  Logger.print(F(" gx "));
  Logger.print(gx);
  Logger.print(F(" gy "));
  Logger.print(gy);
  Logger.print(F(" gz "));
  Logger.print(gz);
  Logger.print(F(" Yaw "));
  Logger.print(yaw);
  Logger.print(F(" Roll "));
  Logger.print(roll);

  Logger.print(F(" smoothedBarAlt "));
  Logger.print(smoothedBarAlt, 4);

  Logger.print(F(" pitotTrust "));
  Logger.print(pitotTrust, 2);
  Logger.print(F(" pitotSpeed "));
  Logger.print(pitotSpeed, 4);

  Logger.print(F(" avgBarSpd "));
  Logger.print(avgBarSpd, 4);
  Logger.print(F(" gravitySpeed "));
  Logger.print(gravitySpeed, 4);
  Logger.print(F(" acc(ft/s) "));
  Logger.print(upAcc, 3);
  Logger.print(F(" upSpeed "));
  Logger.print(upSpeed, 5);
  Logger.print(F(" kinGuess "));
  Logger.print(kinematicsGuess, 5);
  Logger.print(F(" powGuess "));
  Logger.print(powGuess, 5);
  Logger.print(F(" lnGuess "));
  Logger.print(lnGuess, 5);

  Logger.println();

}

void updateAlt() { //finds the current altidute values. Also, if the smoothed speed needs to be changed, which is every other tick, the smoothed speed will be updated.

  barAlt = ps.pressureToAltitudeFeet(ps.readPressureInchesHg()); //finds the altitude.
  float lastSmoothedBarAlt = smoothedBarAlt;
  smoothedBarAlt = (alt_alpha * barAlt + (1 - alt_alpha) * smoothedBarAlt); //finds the smoothed altitude. Should be regarded as most correct altitude.

  float currentSmoothedSpd = (smoothedBarAlt - lastSmoothedBarAlt) / (TICKSPEED_S * 5); //multiplied by 5 because it is called only every 5th time.
  //float lastAvgBarSpd = avgBarSpd;
  avgBarSpd = (speed_alpha * currentSmoothedSpd + (1 - speed_alpha) * avgBarSpd); //smooths the speed.
}

void updateGryoAndAccel() { //finds the pitch

  lsm.read();

  gyro.read();

  // convert from raw data to gravity and degrees/second units
  ax = lsm.a.x * 0.000732;
  ay = lsm.a.y * 0.000732;
  az = lsm.a.z * 0.000732;
  gx = (gyro.g.x * (8.75 / 1000));
  gy = (gyro.g.y * (8.75 / 1000));
  gz = (gyro.g.z * (8.75 / 1000));

  gx = gx - gxOFFSET;
  gy = gy - gyOFFSET;
  gz = gz - gzOFFSET;

  // update the filter, which computes orientation

  if (stage < 2) {
    filter.updateIMU(gx, gy, gz, ax, ay, az, true);
  } else if (stage > 1) {
    filter.updateIMU(gx, gy, gz, ax, ay, az, false);
  }
  // print the heading, pitch and roll
  roll = filter.getRoll();
  pitch = 90 + filter.getPitch();
  yaw = filter.getYaw();

  if (stage < 2) { //we are waiting on the pad.
    upAcc = gravity * (ax - 1); //upward accleration in feet per second.
    if (upAcc < 0.5) { // if acceleration is very low, assume it is just from error. Turn it into noting.
      upAcc = round(upAcc); //current accelertation we are having.
    }
  } else { //we are in the air. This time the acceleration is the hypotenuse, and we need to find the upward component. We do this by multiplying by the cos.
    upAcc = gravity * ((ax / cos(DEG_TO_RAD * pitch)) - 1.0f);
    gravitySpeed  = (upAcc * TICKSPEED_S) + gravitySpeed; //this is called every tick, so we need to multiply the acceleration (feet per secod) by (seconds per tick) to get feet in the last tick. Then add that to the total buildup.
  }

}

void updatePitot() { //finds the speed from the pitot tube.
  byte _status;
  unsigned int P_dat;
  unsigned int T_dat;
  double PR;
  double TR;
  double V;

  byte Press_H, Press_L;

  Wire.requestFrom((int)address, (int) 4);//Request 4 bytes need 4 bytes are read
#if (debugMode == true)
  if (Wire.available() != 4) {
    Serial.println("Error. ");
    Serial.println(Wire.available());
    Serial.println(" bytes given. Need 4");
  }
#endif

  Press_H = Wire.read();
  Press_L = Wire.read();
  byte Temp_H = Wire.read();
  byte  Temp_L = Wire.read();

  _status = (Press_H >> 6) & 0x03;
  Press_H = Press_H & 0x3f;
  P_dat = (((unsigned int)Press_H) << 8) | Press_L;

  Temp_L = (Temp_L >> 5);
  T_dat = (((unsigned int)Temp_H) << 3) | Temp_L;


  switch (_status)
  {
    case 0:
      break;
    case 1: Logger.println("Busy");
      break;
    case 2: Logger.println("Slate");
      break;
    default: Logger.println("Error");
      break;
  }


  PR = (double)((P_dat - 8130.8) / (14744.7));

  if (PR < 0) PR = PR * -1; //aka absolute value

  V = ((PR * 13789.5144) / 1.225);
  pitotSpeed = (sqrt((V)));


  TR = (double)((T_dat * 0.09770395701));
  TR = TR - 50;
}

void checkStage() {

  /* If stage =
    0 = being put on pad; do not collect values.
    1 = waiting
    2 = accelerating
    3 = coasting
    4 = falling under chutes
  */
#if (debugMode==false) //regular operation:
  switch (stage) {
    case 0: //being put on the pad
      Logger.println("Pitch: ");
      Logger.println(pitch);
      
      if ((pitch < 20) and (ax > 0.9) and (ax < 1.1) ) {
        Logger.print(millis());
        Logger.println(F(": Pad Detected!"));
        stage = 1;
        delay(2000);
        tickTime = tickTime + 2000;
        Logger.print(millis());
      }
      break;
    case 1: //collecting values and averages on pad
      if (ax - 1 > 0.25f) {
        stage = 2; //we have taken off.
        Logger.println(F("We have launched."));
        Logger.println();
      }
      break;
    case 2: //accelerating
      if (upAcc < (gravity / 3)) {
        stage = 3; //we are coasting
        Logger.println();
        Logger.println(F("We have lost propulsion."));
      }
      break;
    case 3: //coasting
      if (-5 > upSpeed) { //if its going down, then we need to tell everyone that we are going down.
        Logger.println();
        Logger.println(F("We have passed Apogee."));
        stage = 4;
      }
      break;
    default: //everything else
      break;
  }

#else /*DEBUG MODE
  Runs Case 0 normally
  after 10 seconds on case 1 it decides that it has launched. It prints everything to the screen.
*/
  switch (stage) {
    case 0: //being put on the pad
      if ((pitch < 20) and (ax > 0.9) and (ax < 1.1) ) {
        Logger.print(millis());
        Logger.println(F(": Pad Detected!"));
        stage = 1;
        delay(2000);
        tickTime = tickTime + 2000;
        Logger.print(millis());
        Logger.println(F(": Logging Values..."));
      }
      break;

    case 1: //collecting values and averages on pad
      if (Serial.available() > 0) { //if there is something available.
        stage = 2; //we have taken off.
        Logger.println(F("Calibration Complete."));
        Logger.println();
        Logger.print("gx offset ");
        Logger.print(gxOFFSET);
        Logger.print(" gyOFFSET ");
        Logger.print(gyOFFSET);
        Logger.print(" gzOFFSET: ");
        Logger.println(gzOFFSET);
      }
      break;

    default: //everything else
      break;
  }
#endif
}

void loop() {

  stage = 0; //waiting for the launch
  int runAltSpd = 0;
  tickTime = millis();

  while (true) { //main loop.

    if (millis() >= tickTime) { //tick is called 50 times a second. (20 ms)
      //long startTime = millis();
      tickTime += TICKSPEED_MS; //tickspeed is 20.
      checkStage();
      updateGryoAndAccel();
      if (runAltSpd == 4) { //only runs if it is every 5th time. (takes 22 (24 now with the pitot) ms, which means it goes over the limit)
        updateAlt();
        updatePitot();
        runAltSpd = 0;
        if (stage > 0) {
          calculateSpeedAndAccel(); //this is when it calculates and prints things. (10x a second)
        }
      } else {
        runAltSpd++;
      }
      /*
        if (millis() - startTime > tickSpeedMS) {
        Serial.print("OVERTIME: ");
        Serial.println(millis() - startTime);
        }
      */
    }
  }
}
