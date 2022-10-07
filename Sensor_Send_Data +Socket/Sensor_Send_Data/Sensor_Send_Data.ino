
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/*##################################################################################################
########################################## 定義基本參數###########################################
##################################################################################################*/
const int      Sensor_Count     =     3 ;
int            Sensor_Number    =     36;
String MacAddrStr[Sensor_Count][17] =
                                    {
                                              {"24:D7:EB:0E:BB:C8"},
                                              {"24:D7:EB:0E:B8:C8"},
                                              {"24:D7:EB:10:6A:DC"}
                                    };

Adafruit_MPU6050 mpu;



/*##################################################################################################
########################################## 定義傳輸資料結構###########################################
##################################################################################################*/
typedef struct Sensor_Data {
  int Sensor_Number;
  double Motion6_Data[6];
};

typedef struct Recive_Message{
  int Sensor_Num_Recive;
  int Option_Code ;
};


struct Sensor_Data Sensor_Data1;

struct Recive_Message Recive_Message1;

/*##################################################################################################
###################################################################################################
##################################################################################################*/
uint8_t ServerMac[] = {0x94,0xB5,0x55,0xF4,0x9A,0xF0}; //Sever的MAC位址

#define SEND_INTERVAL 20  //n*200ms
int sendTmr=0,sendTry=0;
 
/*##################################################################################################
#########################################接收資料回調函數#############################################
##################################################################################################*/
void onDataReceiver(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Recive_Message1, incomingData, sizeof(Recive_Message1));
  Serial.printf("Message received: Sensor_Num: ",Recive_Message1.Sensor_Num_Recive,"............ OptionCode",Recive_Message1.Option_Code );
}
 
/*##################################################################################################
#########################################發送資料回調函數#############################################
##################################################################################################*/
//void onSent(uint8_t *mac_addr, uint8_t sendStatus) {
void onSent(const uint8_t *mac_addr, esp_now_send_status_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    
    if(sendTry == 1)
    {
      Serial.println("Delivery success");
     
    }
    else  Serial.printf("Tried %03d\n", sendTry);
    sendTry = 0; //reset flag
  }
  else{
    if(sendTry++ >= 99) {
      Serial.println("Fatal Error: Delivery Failed!!!");
      
    }
  }
}
 
/*##################################################################################################
#############################################任務排程################################################
##################################################################################################*/


/*##################################################################################################
###################################################################################################
##################################################################################################*/
void setup() {
  Serial.begin(115200);
  while (!Serial) {};
  Serial.println();
  delay(1000);
  
  WiFi.mode(WIFI_STA);
  
  Serial.print("Mac Address: ");
  Serial.print(WiFi.macAddress());
  Serial.println("ESP32 ESP-Now Broadcast");
  
  // Initializing the ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Problem during ESP-NOW init");
    return;
  }
 /*##################################################################################################
########################################get WIFI_MAC##############################################
##################################################################################################*/
String Addr = WiFi.macAddress();

String compareMAC ={""};


//--------------------------Check Sensor_Number-------------------------------//
for ( int j = 0 ; j < Sensor_Count ; j++ )   //測試是哪個sensor編號
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

Sensor_Data1.Sensor_Number = Sensor_Number;
Serial.print("Sensor_Data1.Sensor_Number .... = ");
Serial.print(Sensor_Data1.Sensor_Number);
//---------------------------------------------------------------------------//

/*##################################################################################################
#################################################################################################
##################################################################################################*/
  //******************************************************************
  // Register the peer
  //Serial.println("Registering a peer");
  esp_now_register_send_cb(onSent);

  // Register peer
  esp_now_peer_info_t ServerInfo;
  memcpy(ServerInfo.peer_addr, ServerMac, 6);
  ServerInfo.channel = 0;  
  ServerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&ServerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    
    return;
  }  
  
  //******************************************************************
  esp_now_register_recv_cb(onDataReceiver);
  

 
/*##################################################################################################
######################################MPU6050初始化########################################
##################################################################################################*/
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
/*##################################################################################################
###############################################設定排程##############################################
##################################################################################################*/




/*##################################################################################################
#################################################################################################
##################################################################################################*/
}

void loop() {
  
 /*##################################################################################################
######################################MPU6050獲取值########################################
##################################################################################################*/ 
while(1)
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Sensor_Data1.Motion6_Data[0] = a.acceleration.x ;
  Sensor_Data1.Motion6_Data[2] = a.acceleration.z ;
  Sensor_Data1.Motion6_Data[4] = g.gyro.y ;
  Sensor_Data1.Motion6_Data[1] = a.acceleration.y ;
  Sensor_Data1.Motion6_Data[3] = g.gyro.x ;
  Sensor_Data1.Motion6_Data[5] = g.gyro.z ;

  esp_now_send(ServerMac, (uint8_t *) &Sensor_Data1, sizeof(Sensor_Data1));
  delayMicroseconds(1000);
}



/*##################################################################################################
#################################################################################################
##################################################################################################*/ 
/*  if((sendTmr++ >= SEND_INTERVAL) && (sendTry == 0)) {
    sendTmr = 0; //reset tmier
    sendTry = 1; //start sending flag
  }
  
  if(sendTry > 0){

    
  }*/
  
  
}
