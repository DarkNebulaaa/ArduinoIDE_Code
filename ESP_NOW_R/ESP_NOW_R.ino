#include <WiFi.h>
#include <esp_now.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"


// 設定資料結構體
typedef struct struct_message {
  String sensor;
  
  double acc_x;
  double gyo_x;

  double acc_y;
  double gyo_y;

  double acc_z;
  double gyo_z;
  
} struct_message;

struct_message sensor;

// 資料接收回調函式
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&sensor, incomingData, sizeof(sensor));
  Serial.print("        gyo_x");
  Serial.print(sensor.gyo_x);
  Serial.print("        gyo_y");
  Serial.print(sensor.gyo_y);
  Serial.print("        gyo_z");
  Serial.println(sensor.gyo_z);
}

void setup() {
  Serial.begin(115200);

  // 初始化 ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // 設定接收資料回撥函式
  esp_now_register_recv_cb(OnDataRecv);

  ledcSetup(7, 50, 8);  
  ledcAttachPin(2, 7);  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
}

void loop() {
    int angle;
    if (sensor.gyo_x>9)
    {
      angle=90;
    }
    angle = (sensor.gyo_x)*10;
    int value=map(angle,0,180,6.4,30.72);
    ledcWrite(7, value);
    delay(1);
   



}
