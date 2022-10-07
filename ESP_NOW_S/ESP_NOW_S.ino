/**
 * ESP-NOW: ESP32 Broadcast with ESP-Now
 * 
 * 
*/
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
// ESP8266 Mac address (first peer)
uint8_t mac_peer1[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
// ESP8266 Mac address (second peer)
uint8_t mac_peer2[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peer1;
esp_now_peer_info_t peer2;
int i = 0;
typedef struct message {
   int red;
   int green;
   int blue;
};
struct message myMessage;
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  // Get Mac Add
  Serial.print("Mac Address: ");
  Serial.print(WiFi.macAddress());
  Serial.println("ESP32 ESP-Now Broadcast");
  // Initializing the ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Problem during ESP-NOW init");
    return;
  }
  memcpy(peer1.peer_addr, mac_peer1, 6);
  peer1.channel = 4;
  peer1.encrypt = 0;
  // Register the peer
  Serial.println("Registering a peer 1");
  if ( esp_now_add_peer(&peer1) == ESP_OK) {
    Serial.println("Peer 1 added");
  }  
  memcpy(peer2.peer_addr, mac_peer2, 6);
  peer2.channel = 4;
  peer2.encrypt = 0;
  // Register the peer
  Serial.println("Registering a peer 2");
  if ( esp_now_add_peer(&peer2) == ESP_OK) {
    Serial.println("Peer 2 added");
  }  
}
void loop() {
  myMessage.red = random(0, 254);
  myMessage.green = random(0, 254);
  myMessage.blue = random(0, 254);
  Serial.println("Send a new message");
  esp_now_send(NULL, (uint8_t *) &myMessage, sizeof(myMessage));
  delay(2000);
}
