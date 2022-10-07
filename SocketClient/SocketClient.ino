#include <WiFi.h>
#include <WiFiUdp.h> //引用以使用UDP

const char *ssid = "DarkNebula";
const char *password = "0937109667";

WiFiUDP Udp;                      //建立UDP物件
unsigned int localUdpPort = 2333; //本地埠號

void setup()
{
  Serial.begin(115200);
  Serial.println();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (!WiFi.isConnected())
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected");
  Serial.print("IP Address:");
  Serial.println(WiFi.localIP());

  Udp.begin(localUdpPort); //啟用UDP監聽以接收資料
}

void loop()
{
  int packetSize = Udp.parsePacket(); //獲取當前隊首資料包長度
  if (packetSize)                     //如果有資料可用
  {
    
  }
    char buf[packetSize] ;
    Udp.read(buf, packetSize); //讀取當前包資料

    Serial.println();
    Serial.print("Received: ");
    Serial.println(buf);
    Serial.print("From IP: ");
    Serial.println(Udp.remoteIP());
    Serial.print("From Port: ");
    Serial.println(Udp.remotePort());

    Udp.beginPacket("192.168.137.1", 2333); //準備傳送資料
    Udp.print("Received:12345 ");    //複製資料到傳送快取
    Udp.write((const uint8_t*)buf, packetSize); //複製資料到傳送快取
    Udp.endPacket();            //傳送資料

  
}
