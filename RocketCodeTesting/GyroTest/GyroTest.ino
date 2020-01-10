/*
  The sensor outputs provided by the library are the raw 16-bit values
  obtained by concatenating the 8-bit high and low gyro data registers.
  They can be converted to units of dps (degrees per second) using the
  conversion factors specified in the datasheet for your particular
  device and full scale setting (gain).

  Example: An L3GD20H gives a gyro X axis reading of 345 with its
  default full scale setting of +/- 245 dps. The So specification
  in the L3GD20H datasheet (page 10) states a conversion factor of 8.75
  mdps/LSB (least significant bit) at this FS setting, so the raw
  reading of 345 corresponds to 345 * 8.75 = 3020 mdps = 3.02 dps.
*/

#include <Wire.h>
#include <L3G.h>
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

  Serial.println("Starting Gyro Test. For calibration of values, please run Gyro Calibration");

  gyro.read();
  gx = gyro.g.x * (8.75 / 1000);
  gy = gyro.g.y * (8.75 / 1000);
  gz = gyro.g.z * (8.75 / 1000);

  gxt = gxt + gx;
  gyt = gyt + gy;
  gzt = gzt + gz;
  count++;

  Serial.print("X: ");
  Serial.print(gx);
  Serial.print(" Y: ");
  Serial.print(gy);
  Serial.print(" Z: ");
  Serial.print(gz);

  delay(20);
}
