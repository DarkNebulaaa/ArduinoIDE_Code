#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "esp_now.h"
#include "WiFi.h"

//---------------------Define initial Value--------------------------//
#define Correction_Times 1000

#define Reciver_Server   0     //定義接收主機

int            Sensor_Number         =     36   ;  //..........@@感測器編號@@............//
const int      Sensor_Data_Count     =     2    ;  //..........@@感測器數量@@............//


double X_Gyro_Offset;
double Y_Gyro_Offset;
double Z_Gyro_Offset;

const int Gyro_Offset_Row = 3;
const int Gyro_Offset_Columns = Correction_Times;


double Gyro_Offset_Matrix[Gyro_Offset_Row][Gyro_Offset_Columns];


const int Motion6_Data = 6;   /*    [ Acceleration: X , Y , Z  ; Gyro: X , Y , Z ; ]; */

//..........................所有ESP32   MACAddr...............................//
const uint8_t MacAddr[Sensor_Data_Count][6] = 
                                    {
                                              {0x24,0xD7,0xEB,0x0E,0xBB,0xC8},    
                                              {0x24,0xD7,0xEB,0x0E,0xB8,0xC8}
                                    };

String MacAddrStr[Sensor_Data_Count][17] =
                                    {
                                              {"24:D7:EB:0E:BB:C8"},
                                              {"24:D7:EB:0E:B8:C8"}
                                    };


uint8_t *Get_MAC_ADDR( int sensorGet )
{
  uint8_t MACAddr1[6] ={};
  for (int i = 0;i < 6;i++)
  {
    MACAddr1[i]=MacAddr[sensorGet][i];
  }
  return  MACAddr1;
}
                                    
//................................ESP_NOW Def...............................//

typedef struct Matrix{                            //EspNow_資料結構
  
  double Final_Data_Matrix[Sensor_Data_Count][Motion6_Data];
  
};
  struct Matrix Final_Data;

//------------------------------------------------------------------------------//



// ............................計算結果回傳Matrix.................................//
void Data_Matrix(double ax ,double ay ,double az ,double gx ,double gy ,double gz)         
{
  Final_Data.Final_Data_Matrix[ Sensor_Number ][ 0 ] = ax ;
  Final_Data.Final_Data_Matrix[ Sensor_Number ][ 1 ] = ay ;
  Final_Data.Final_Data_Matrix[ Sensor_Number ][ 2 ] = az ;
  Final_Data.Final_Data_Matrix[ Sensor_Number ][ 3 ] = gx ;
  Final_Data.Final_Data_Matrix[ Sensor_Number ][ 4 ] = gy ;
  Final_Data.Final_Data_Matrix[ Sensor_Number ][ 5 ] = gz ;

  
}
//...............................................................................//



esp_now_peer_info_t peerInfoRecv1;


Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
//----------------------------Set_WifiMode------------------------------------//

  WiFi.mode(WIFI_STA);
  
 //--------------------------Get_ESP MAC_Addr---------------------------------//


String Addr = WiFi.macAddress();

String compareMAC ={""};


//--------------------------Check Sensor_Number-------------------------------//
for ( int j = 0 ; j < Sensor_Data_Count ; j++ )   //測試是哪個sensor編號
{
      for(int i = 0; i< 17 ; i++)
      
          { 
                compareMAC += MacAddrStr[j][i];
          }
      if(compareMAC == Addr)
          {
                Serial.println("SensorNumberSet!!....");
                Sensor_Number = j;
                break;
          } 
      compareMAC ={""};

}
if (Sensor_Number == 36)
{
  Serial.print("SensorError :  Can not find match MAC_Address");
}

//---------------------------------------------------------------------------//

Serial.print("SensorNumber......");
Serial.println(Sensor_Number);




delay(2000);
 //------------------- Try to initialize  MPU 6050!---------------------------//
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);


//-----------------------Esp_Now init----------------------------//
  if (esp_now_init() != ESP_OK) 
  {

    Serial.println("Error initializing ESP-NOW");
    ESP.restart();
    return;
  }

//---------------set Esp_Now Sender&Reciver Address--------------//
    memcpy( peerInfoRecv1.peer_addr , Get_MAC_ADDR( Reciver_Server ) , 6);
    peerInfoRecv1.channel = 0;
    peerInfoRecv1.encrypt = false;
  
//---------------get sensor Correction Data----------------------//
 
  

 
for(int i = 0;i < Correction_Times ;i ++)
  { 
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    Gyro_Offset_Matrix[0][i] = g.gyro.x;
    Gyro_Offset_Matrix[1][i] = g.gyro.y;
    Gyro_Offset_Matrix[2][i] = g.gyro.z;
    delay(10);
    Serial.print(".....get Data.....  "); 
    Serial.print(i); 
    Serial.print("/ "); 
    Serial.println(Correction_Times);
  }
  int j;

for(j = 0;j < Gyro_Offset_Columns ;j ++)
  {
        X_Gyro_Offset += Gyro_Offset_Matrix[0][j];
        Y_Gyro_Offset += Gyro_Offset_Matrix[1][j];
        Z_Gyro_Offset += Gyro_Offset_Matrix[2][j];
  }

    
X_Gyro_Offset /= Gyro_Offset_Columns;
Y_Gyro_Offset /= Gyro_Offset_Columns;
Z_Gyro_Offset /= Gyro_Offset_Columns; 


 //  Serial.println(X_Gyro_Offset);
 //  Serial.println(Y_Gyro_Offset);
 //  Serial.println(Z_Gyro_Offset);
}

//---------------------------Data Recv---------------------------------//

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Final_Data, incomingData, sizeof(Final_Data));
  
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

 /* save Data to Matrix */
  
Data_Matrix( a.acceleration.x , a.acceleration.y , a.acceleration.z , g.gyro.x - X_Gyro_Offset , g.gyro.y - Y_Gyro_Offset , g.gyro.z - Z_Gyro_Offset );

if ( Sensor_Number != Reciver_Server )
{
  esp_now_send(Get_MAC_ADDR( Reciver_Server ) ,(uint8_t *) &Final_Data , sizeof(Final_Data));
}
else if (Sensor_Number != Reciver_Server)
{
  ;
}


Serial.print(Final_Data.Final_Data_Matrix[0][0]);
Serial.print("........");
Serial.println(Final_Data.Final_Data_Matrix[1][0]);

 /* Print out the values */
 /*  
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");
 

 
  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x - X_Gyro_Offset);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y - Y_Gyro_Offset);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z - Z_Gyro_Offset);
  Serial.println(" rad/s");

*/


 // Serial.println("");
  
}
