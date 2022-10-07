#include <QMC5883L_App.h>

float Zx ,Zy ,Zz; //get zero Vector;
float Xx , Yy , Zz;
QMC5883LApp compass;
void setup() {
  Serial.begin(115200);
  compass.init();
  //compass.setMode(01 , 11 , 00 , 00);
  Serial.println("Starting Calibration");
  compass.getCalibration();
  compass.setCalibration();
  compass.clampVec();
  compass.read();
  Zx = compass.getVectorX();
  Zy = compass.getVectorY();
  Zz = compass.getVectorZ();
}

void loop() 
{
  float x, y, z;
  
  // Read compass values
  compass.read();

  // Return XYZ readings
  x = compass.getVectorX();
  y = compass.getVectorY();
  z = compass.getVectorZ();
  
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.print(z);
  Serial.println();
}
