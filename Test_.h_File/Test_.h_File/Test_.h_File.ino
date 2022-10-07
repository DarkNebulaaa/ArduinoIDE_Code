#include "Motioncapture.h"

Mocap mocap;
void setup() {
  Serial.begin(115200);
  mocap.Initialize();
  mocap.GetCalibrationData();
}

void loop() {
  mocap.get9MotionData();

  Serial.print(F("  AccX : "));
  if(mocap.AccX >= 0)Serial.print(" ");
  Serial.print(mocap.AccX);
  Serial.print(F("  AccY : "));
  if(mocap.AccY >= 0)Serial.print(" ");
  Serial.print(mocap.AccY);
  Serial.print(F("  AccZ : "));
  if(mocap.AccZ >= 0)Serial.print(" ");
  Serial.print(mocap.AccZ);

  Serial.print(F("  GyroX : "));
  if(mocap.GyroX >= 0)Serial.print(" ");
  Serial.print(mocap.GyroX);
  Serial.print(F("  GyroY : "));
  if(mocap.GyroY >= 0)Serial.print(" ");
  Serial.print(mocap.GyroY);
  Serial.print(F("  GyroZ : "));
  if(mocap.GyroZ >= 0)Serial.print(" ");
  Serial.print(mocap.GyroZ);

  Serial.print(F("  MagnX : "));
  if(mocap.MagnX >= 0)Serial.print(" ");
  Serial.print(mocap.MagnX);
  Serial.print(F("  MagnY : "));
  if(mocap.MagnY >= 0)Serial.print(" ");
  Serial.print(mocap.MagnY);
  Serial.print(F("  MagnZ : "));
  if(mocap.MagnZ >= 0)Serial.print(" ");
  Serial.println(mocap.MagnZ);

}
