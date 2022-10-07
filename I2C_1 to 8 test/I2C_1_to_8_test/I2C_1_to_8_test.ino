// ================================================================
// ===                 Define TCA9548A Address                  ===
// ================================================================


#include "Wire.h"

#define MUX_Address 0x70 // TCA9548A Encoders address


void SelectMpu(uint8_t i) {

if (i > 7) return;

Wire.beginTransmission(MUX_Address);

Wire.write(1 << i);

Wire.endTransmission();

}

// ================================================================
// ===                      Define MpuDMP                       ===
// ================================================================
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps612.h"

#define OUTPUT_READABLE_QUATERNION

MPU6050 HandsR , ArmRD , ArmRU;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSizeHandsR , packetSizeArmRD , packetSizeArmRU;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t HandsRfifoBuffer[64] , ArmRDfifoBuffer[64] ,ArmRUfifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q0 ,q1 ,q2;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

/*###########################################################################################
#############################################################################################
###########################################################################################*/

void setup()
{
   Wire.begin();
   Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
   #if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
   Fastwire::setup(400, true);
   #endif

   Serial.begin(115200);

   InitializeMpu();
   delay(100);
   SetOffsetMpu();
   delay(100);
   CalibrateMpuTime();
   delay(100);
   setDMPEnabled();
   delay(100);
   dmpGetFIFOPacketSize();
}




void loop()
{
    dmpGetQUATERNION();
    Serial.print("q0.x  ");
    Serial.print(q0.x);
    Serial.print("q1.x  ");
    Serial.print(q1.x);
    Serial.print("q2.x  ");
    Serial.println(q2.x);
}

/*###########################################################################################
#############################################################################################
###########################################################################################*/


void InitializeMpu()
{   
    SelectMpu(0);
    Serial.println(F("Initializing I2C devices...ArmRU......"));
    ArmRU.initialize();
    ArmRU.dmpInitialize();
    delay(10);
    SelectMpu(1);
    Serial.println(F("Initializing I2C devices...ArmRD......"));
    ArmRD.initialize();
    ArmRD.dmpInitialize();
    delay(10);
    SelectMpu(2);
    Serial.println(F("Initializing I2C devices...HandsR......"));
    HandsR.initialize();
    HandsR.dmpInitialize();
    delay(10);
    
}


void SetOffsetMpu()
{
    SelectMpu(0);
    Serial.println(F("SetOffsetMpu...ArmRU......"));
    ArmRU.setXGyroOffset(51);
    ArmRU.setYGyroOffset(8);
    ArmRU.setZGyroOffset(21);
    ArmRU.setXAccelOffset(1150);
    ArmRU.setYAccelOffset(-50);
    ArmRU.setZAccelOffset(1060);
    delay(10);
    SelectMpu(1);
    Serial.println(F("SetOffsetMpu...ArmRD......"));
    ArmRD.setXGyroOffset(51);
    ArmRD.setYGyroOffset(8);
    ArmRD.setZGyroOffset(21);
    ArmRD.setXAccelOffset(1150);
    ArmRD.setYAccelOffset(-50);
    ArmRD.setZAccelOffset(1060);
    delay(10);
    SelectMpu(2);
    Serial.println(F("SetOffsetMpu...HandsR......"));
    HandsR.setXGyroOffset(51);
    HandsR.setYGyroOffset(8);
    HandsR.setZGyroOffset(21);
    HandsR.setXAccelOffset(1150);
    HandsR.setYAccelOffset(-50);
    HandsR.setZAccelOffset(1060);
    delay(10);
}
void CalibrateMpuTime()
{   
    SelectMpu(0);
    Serial.println(F("CalibrateMpuTime...ArmRU......"));
    ArmRU.CalibrateAccel(6);
    ArmRU.CalibrateGyro(6);
    delay(10);
    SelectMpu(1);
    Serial.println(F("CalibrateMpuTime...ArmRD......"));
    ArmRD.CalibrateAccel(6);
    ArmRD.CalibrateGyro(6);
    delay(10);
    SelectMpu(2);
    Serial.println(F("CalibrateMpuTime...HandsR......"));
    HandsR.CalibrateAccel(6);
    HandsR.CalibrateGyro(6);
    delay(10);
}
void setDMPEnabled()
{   
    SelectMpu(0);
    Serial.println(F("setDMPEnabled...ArmRU......"));
    ArmRU.setDMPEnabled(true);
    delay(10);
    SelectMpu(1);
    Serial.println(F("CalibrateMpuTime...ArmRD......"));
    ArmRD.setDMPEnabled(true);
    delay(10);
    SelectMpu(2);
    Serial.println(F("CalibrateMpuTime...HandsR......"));
    HandsR.setDMPEnabled(true);
    delay(10);
}

void dmpGetFIFOPacketSize()
{   
    SelectMpu(0);
    Serial.println("dmpGetFIFOPacketSize...ArmRU......");
    packetSizeArmRU = ArmRU.dmpGetFIFOPacketSize();
    delay(5);
    SelectMpu(1);
    Serial.println("CalibrateMpuTime...ArmRD......");
    packetSizeArmRD = ArmRD.dmpGetFIFOPacketSize();
    delay(5);
    SelectMpu(2);
    Serial.println("CalibrateMpuTime...HandsR......");
    packetSizeHandsR = HandsR.dmpGetFIFOPacketSize();
    delay(5);
}

void dmpGetQUATERNION()
{   
  if (ArmRU.dmpGetCurrentFIFOPacket(ArmRUfifoBuffer)) 
  {
    SelectMpu(0);
    ArmRU.dmpGetQuaternion(&q0, ArmRUfifoBuffer);
    delay(5);    
  }

  if (ArmRD.dmpGetCurrentFIFOPacket(ArmRDfifoBuffer)) 
  {
    SelectMpu(1);
    ArmRD.dmpGetQuaternion(&q1, ArmRDfifoBuffer);
    delay(5);    
  }    

  if (HandsR.dmpGetCurrentFIFOPacket(HandsRfifoBuffer)) 
  {
    SelectMpu(2);
    HandsR.dmpGetQuaternion(&q2, HandsRfifoBuffer);
    delay(5);   
  }    

}
