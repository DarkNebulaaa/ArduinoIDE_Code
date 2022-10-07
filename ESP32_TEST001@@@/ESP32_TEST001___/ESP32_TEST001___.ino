#include <WiFi.h>
#include <esp_now.h>
 
typedef struct message {
  float temperature;
  float humidity;
};
 
struct message myMessage;

uint8_t peer1[] = {0x24,0xD7,0xEB,0x0E,0xB8,0xC8}; //接收端的MAC位址

#define SEND_INTERVAL 20  //n*200ms
int sendTmr=0,sendTry=0;
 
/*##################################################################################################

##################################################################################################*/
void onDataReceiver(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myMessage, incomingData, sizeof(myMessage));
  Serial.printf("Message received: T=%03.1f H=%03.1f\n", myMessage.temperature, myMessage.humidity);
}
 
/*##################################################################################################

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
 
  //******************************************************************
  // Register the peer
  //Serial.println("Registering a peer");
  esp_now_register_send_cb(onSent);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, peer1, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }  
  
  //******************************************************************
  esp_now_register_recv_cb(onDataReceiver);
  
}
 
/*##################################################################################################

##################################################################################################*/
void loop() {
  myMessage.temperature = 18.5;
  myMessage.humidity = 60.7;
 
  if((sendTmr++ >= SEND_INTERVAL) && (sendTry == 0)) {
    sendTmr = 0; //reset tmier
    sendTry = 1; //start sending flag
  }
  
  if(sendTry > 0){
    //Serial.println("Send a new message");
    //esp_now_send(NULL, (uint8_t *) &myMessage, sizeof(myMessage));
    esp_now_send(peer1, (uint8_t *) &myMessage, sizeof(myMessage));
  }
  
  delay(200);
}
