#include <Wire.h>
#include <L3G.h>
#define TIMEOUT_MILLIS 2000
#define TIMEOUT_SECONDS TIMEOUT_MILLIS/1000
L3G gyro;

double gxt, gyt, gzt;
int count;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!gyro.init())
  {
    Serial.println("Error: Failed to autodetect gyro type! Stopping.");
    while (1);
  }

  gyro.enableDefault();
}

void loop() {

  double gx, gy, gz;

  Serial.print("Send any key to start calibration. Press any key to end calibration.\nCalibration automatically ends at ");
  Serial.print(TIMEOUT_SECONDS);
  Serial.println(" seconds.");

  while (!(Serial.available() > 0)) {
    delay(1000);
  }

  Serial.println("Starting Calibration. Keep still!");
  long startMillis = millis();

  while (Serial.available() > 0) {
    Serial.read();
  }

  float gxMAX, gyMAX, gzMAX;
  float gxMIN, gyMIN, gzMIN;
  const int tenth = TIMEOUT_MILLIS / 10;
  long counter;
  long startOfLastTenth = millis();
  while (millis() < TIMEOUT_MILLIS + startMillis and Serial.available() == 0) {

    gyro.read();
    gx = gyro.g.x * (8.75 / 1000);
    gy = gyro.g.y * (8.75 / 1000);
    gz = gyro.g.z * (8.75 / 1000);

    gxt = gxt + gx;
    gyt = gyt + gy;
    gzt = gzt + gz;
    count++;
    /*
      Serial.print("X: ");
      Serial.print(gx);
      Serial.print(" Y: ");
      Serial.print(gy);
      Serial.print(" Z: ");
      Serial.print(gz);
    */
    gxMAX = max(gxMAX, gx);
    gyMAX = max(gyMAX, gy);
    gzMAX = max(gzMAX, gz);

    gxMIN = min(gxMIN, gx);
    gyMIN = min(gyMIN, gy);
    gzMIN = min(gzMIN, gz);

    if (millis() - startOfLastTenth > tenth) {
      startOfLastTenth = millis();
      Serial.print("-");
    }
    delay(5);
  }
  Serial.println();
  Serial.println("Gyro Calibration Complete.");
  Serial.print("#define gxOFFSET ");
  Serial.println(gxt / count, 5);
  Serial.print("#define gyOFFSET ");
  Serial.println(gyt / count, 5);
  Serial.print("#define gzOFFSET ");
  Serial.println(gzt / count, 5);
  Serial.println("\n\n\n");

  Serial.println("GX Avg: ");
  Serial.print(gxt / count);
  Serial.print(" Max: ");
  Serial.print(gxMAX);
  Serial.print(" Min: ");
  Serial.println(gxMIN);


  delay(1000);
  Serial.println("Restarting");
}
