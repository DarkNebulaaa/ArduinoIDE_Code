
#include <WiFi.h>
#include <esp_now.h>

#define Sensor_Count 3
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


struct Sensor_Data Sensor_Data0 ;

struct Recive_Message Recive_Message1;

double Final_Motion_Data[Sensor_Count][6];
/*##################################################################################################
###################################################################################################
##################################################################################################*/
uint8_t SensorMac00[] = {0x24,0xD7,0xEB,0x0E,0xBB,0xC8}; //Sensor00的MAC位址
uint8_t SensorMac01[] = {0x24,0xD7,0xEB,0x0E,0xB8,0xC8}; //Sensor01的MAC位址
uint8_t SensorMac02[] = {0x24,0xD7,0xEB,0x10,0x6A,0xDC}; //Sensor02的MAC位址

#define SEND_INTERVAL 20  //n*200ms
int sendTmr=0,sendTry=0;
 
/*##################################################################################################
#########################################接收資料回調函數#############################################
##################################################################################################*/
void onDataReceiver(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Sensor_Data0, incomingData, sizeof(Sensor_Data0));
  Serial.printf("Message received" );
  for (int i = 0; i < 6 ; i++)
  {
    Final_Motion_Data[Sensor_Data0.Sensor_Number][i] = Sensor_Data0.Motion6_Data[i];
  }
  
}
 
/*##################################################################################################
#########################################發送資料回調函數#############################################
##################################################################################################*/
//void onSent(uint8_t *mac_addr, uint8_t sendStatus) {
void onSent(const uint8_t *mac_addr, esp_now_send_status_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    if(sendTry == 1) Serial.println("Delivery success");
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
#########################################利用plotter匯出感測器數值#############################################
##################################################################################################*/
#include "Plotter.h"
double x;

Plotter p; // create plotter


/*##################################################################################################
####################################################################################################
##################################################################################################*/
void setup() {
  Serial.begin(115200);
  while (!Serial) {};
  pinMode(LED_BUILTIN,OUTPUT);
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
 
/*#####################################################################################################
                                      // Register the peer
######################################################################################################*/
  
  //Serial.println("Registering a peer");
  esp_now_register_send_cb(onSent);

  // Register peer
  esp_now_peer_info_t SensorInfo0 ,SensorInfo1 , SensorInfo2;
  memcpy(SensorInfo0.peer_addr,  SensorMac00, 6);
  memcpy(SensorInfo1.peer_addr,  SensorMac01, 6);
  memcpy(SensorInfo2.peer_addr,  SensorMac02, 6);
  SensorInfo0.channel = 0;  
  SensorInfo0.encrypt = false;
  SensorInfo0.ifidx = WIFI_IF_STA;
  
  SensorInfo1.channel = 0;  
  SensorInfo1.encrypt = false;
  SensorInfo0.ifidx = WIFI_IF_STA;
  
  SensorInfo2.channel = 0;  
  SensorInfo2.encrypt = false;  
  SensorInfo0.ifidx = WIFI_IF_STA;
  digitalWrite(LED_BUILTIN,HIGH);
  // Add peer        
  if (esp_now_add_peer(&SensorInfo0) != ESP_OK){
    Serial.println("Failed to add SensorInfo0 peer");
    digitalWrite(LED_BUILTIN,LOW);
    return;
  }  
  if (esp_now_add_peer(&SensorInfo1) != ESP_OK){
    Serial.println("Failed to add SensorInfo1 peer");
    digitalWrite(LED_BUILTIN,LOW);
    return;
  }
  if (esp_now_add_peer(&SensorInfo2) != ESP_OK){
    Serial.println("Failed to add SensorInfo2 peer");
    digitalWrite(LED_BUILTIN,LOW);
    return;
  }  
  //******************************************************************
  esp_now_register_recv_cb(onDataReceiver);











}
  
/*##################################################################################################
/*##################################################################################################
####################################################################################################
##################################################################################################*/
/*##################################################################################################
/*##################################################################################################
####################################################################################################
##################################################################################################*/
void loop() {

 
Serial.print(Final_Motion_Data[0][0]); 
Serial.print("\t"); 
Serial.print(Final_Motion_Data[1][0]);
Serial.print("\t"); 
Serial.println(Final_Motion_Data[2][0]);

/*  if((sendTmr++ >= SEND_INTERVAL) && (sendTry == 0)) {
    sendTmr = 0; //reset tmier
    sendTry = 1; //start sending flag
  }
  
  if(sendTry > 0){

    esp_now_send(ServerMac, (uint8_t *) &Sensor_Data1, sizeof(Sensor_Data1));
  }
  */

}
