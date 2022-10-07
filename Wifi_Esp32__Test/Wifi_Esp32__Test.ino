#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "esp_now.h"
#include "WiFi.h"



#define Correction_Times 1000

#define SEND_INTERVAL 20

#define Reciver_Server   0     //定義接收主機

int            Sensor_Number         =     36   ;  //..........@@感測器編號@@............//
const int      Sensor_Data_Count     =     2    ;  //..........@@感測器數量@@............//


int sendTmr=0,sendTry=0;



Adafruit_MPU6050 mpu;
esp_now_peer_info_t peerInfoRecv1;
double X_Gyro_Offset;
double Y_Gyro_Offset;
double Z_Gyro_Offset;

const int Gyro_Offset_Row = 3;
const int Gyro_Offset_Columns = Correction_Times;


double Gyro_Offset_Matrix[Gyro_Offset_Row][Gyro_Offset_Columns];


const int Motion6_Data = 6;   /*    [ Acceleration: X , Y , Z  ; Gyro: X , Y , Z ; ]; */

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
//###################################################################################
//###################################################################################
//###################################################################################

uint8_t *Get_MAC_ADDR( int sensorGet )
{
  uint8_t MACAddr1[6] ={};
  for (int i = 0;i < 6;i++)
  {
    MACAddr1[i]=MacAddr[sensorGet][i];
  }
  return  MACAddr1;
}
//###################################################################################
//###################################################################################
//###################################################################################
typedef struct Matrix{                            //EspNow_資料結構
  int    sender;
  double Send_Data_Matrix[Motion6_Data];
  
};
  struct Matrix Send_Data;

double Final_Data_Matrix[ Sensor_Data_Count ][ 6 ];
double Data_MoSensor[6];


void Data_Matrix()         
{

  for (int i = 0 ; i < Sensor_Data_Count ; i++)
    {

        for( int j = 0 ; j < 6 ; j++ )  //這邊之後需要加上 Send_Data 的sender判斷
          { 
            if (i == Sensor_Number )
            {
            Final_Data_Matrix[ i ][ j ] = Data_MoSensor[j];
            }
            else if(Send_Data.sender == i)
            {
            Final_Data_Matrix[ i ][ j ] = Send_Data.Send_Data_Matrix[j];
            }
          }
    }
}


//###################################################################################
//###################################################################################
//###################################################################################


void onDataReceiver(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  memcpy(&Send_Data, incomingData, sizeof(Send_Data));
  
}




void onSent(const uint8_t *mac_addr, esp_now_send_status_t sendStatus) 
{
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0)
  {
    if(sendTry == 1) Serial.println("Delivery success");
    else  Serial.printf("Tried %03d\n", sendTry);
    sendTry = 0; //reset flag
  }
  else
  {
    if(sendTry++ >= 99) 
    {
      Serial.println("Fatal Error: Delivery Failed!!!");
    }
  }
}






//###################################################################################
//###################################################################################
//###################################################################################
void setup() {
Serial.begin(115200);
WiFi.mode(WIFI_STA);
String Addr = WiFi.macAddress();
esp_now_init();
String compareMAC ={""};

pinMode(LED_BUILTIN,OUTPUT);

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


if (esp_now_init() != ESP_OK) 
  {

    Serial.println("Error initializing ESP-NOW");
    ESP.restart();
    return;
  }


if(Reciver_Server != Sensor_Number)
{
  memcpy( peerInfoRecv1.peer_addr , Get_MAC_ADDR( Reciver_Server ) , 6);
  peerInfoRecv1.channel = 0;
  peerInfoRecv1.encrypt = false;
}
else if (Reciver_Server == Sensor_Number)
{
digitalWrite(LED_BUILTIN,HIGH);
Serial.print("This is Reciver");        
}


//###################################################################################
//###################################################################################
//###################################################################################

esp_now_register_send_cb(onSent);


memcpy(peerInfoRecv1.peer_addr,Get_MAC_ADDR(Reciver_Server), 6);
peerInfoRecv1.channel = 0;  
peerInfoRecv1.encrypt = false;



esp_now_register_recv_cb(onDataReceiver);


Send_Data.sender = Sensor_Number;


//###################################################################################
//###################################################################################
//###################################################################################
}



void loop() {

//-------------------------------------------------------------
sensors_event_t a, g, temp;
mpu.getEvent(&a, &g, &temp);

Data_MoSensor[0]=a.acceleration.x ;
Data_MoSensor[1]= a.acceleration.y ;
Data_MoSensor[2]= a.acceleration.z ;
Data_MoSensor[3]=g.gyro.x - X_Gyro_Offset ;
Data_MoSensor[4]= g.gyro.y - Y_Gyro_Offset ;
Data_MoSensor[5]= g.gyro.z - Z_Gyro_Offset;


Data_Matrix();
//------------------------------------------------------------

//###################################################################################
//###################################################################################
//###################################################################################


if((sendTmr++ >= SEND_INTERVAL) && (sendTry == 0)) {
    sendTmr = 0; //reset tmier
    sendTry = 1; //start sending flag
  }
  
  if(sendTry > 0){
    //Serial.println("Send a new message");
    //esp_now_send(NULL, (uint8_t *) &myMessage, sizeof(myMessage));
     esp_now_send(Get_MAC_ADDR(Reciver_Server), (uint8_t *) &Send_Data, sizeof(Send_Data));
  }
  //###################################################################################
//###################################################################################
//###################################################################################
}
