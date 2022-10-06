/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other
  這是我們的Adafruit 16通道PWM和伺服驅動器的一個例子，驅動16個伺服電機

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4
  這些顯示器使用I2C進行通訊，需要2個引腳。
  介面。對於ARDUINO UNOS，這是SCL->模擬5，SDA - >模擬4

  ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
////以這種方式呼叫，它使用預設地址0x40。
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
//也可以用不同的地址呼叫它

/* Depending on your servo make, the pulse width min and max may vary, you  want these to be as small/large as possible without hitting the hard stop
 for max range. You'll have to tweak them as necessary to match the servos you
have!*/
/*根據你的伺服制作，脈衝寬度最小和最大可能變化，你想要這些儘可能小大而不碰到
硬停止，對於最大範圍。你必須調整它們以匹配你的伺服系統！*/
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
//這是“最小”脈衝長度計數（在4096）中
#define SERVOMAX  500 // this is the 'maximum' pulse length count (out of 4096)
//這是“最大”脈衝長度計數（在4096中）

// our servo # counter
//uint8_t servonum = 0;

void setup() {
   Serial.begin(9600);
   Serial.println("16 channel Servo test!");

  pwm.begin();
   
   pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
   ////模擬伺服在60赫茲更新下執行
}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
//如果您想以秒為單位設定脈衝長度，則可以使用此函式。
//例如SET伺服脈衝（0，0.001）是一個1毫秒的脈衝寬度。它不是
void setServoPulse(uint8_t n, double pulse) {
   double pulselength;//精度浮點數
   
   pulselength = 1000000;   // 1,000,000 us per second 每秒100萬
   pulselength /= 60;   // 60 Hz
   Serial.print(pulselength); Serial.println(" us per period"); 
   pulselength /= 4096;  // 12 bits of resolution 12位解析度
   Serial.print(pulselength); Serial.println(" us per bit"); 
   pulse *= 1000;
   pulse /= pulselength;
   Serial.println(pulse);
   pwm.setPWM(n, 0, pulse);
}

void loop() {
   // Drive each servo one at a time
   //Serial.println(servonum);
   //每次驅動一個伺服驅動器
//序列列印（伺服）；
   for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
     pwm.setPWM(0, 0, pulselen);
     pwm.setPWM(1, 0, pulselen);
     pwm.setPWM(2, 0, pulselen);
     pwm.setPWM(3, 0, pulselen);
     pwm.setPWM(4, 0, pulselen);
     pwm.setPWM(5, 0, pulselen);
     pwm.setPWM(6, 0, pulselen);
     pwm.setPWM(7, 0, pulselen);
     pwm.setPWM(8, 0, pulselen);
     pwm.setPWM(9, 0, pulselen);
     pwm.setPWM(10, 0, pulselen);
     pwm.setPWM(11, 0, pulselen);
     pwm.setPWM(12, 0, pulselen);
     pwm.setPWM(13, 0, pulselen);
     pwm.setPWM(14, 0, pulselen);

     pwm.setPWM(15, 0, pulselen);
   }
   delay(10);
   for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
     pwm.setPWM(0, 0, pulselen);
     pwm.setPWM(1, 0, pulselen);
     pwm.setPWM(2, 0, pulselen);
     pwm.setPWM(3, 0, pulselen);
     pwm.setPWM(4, 0, pulselen);
     pwm.setPWM(5, 0, pulselen);
     pwm.setPWM(6, 0, pulselen);
     pwm.setPWM(7, 0, pulselen);
     pwm.setPWM(8, 0, pulselen);
     pwm.setPWM(9, 0, pulselen);
     pwm.setPWM(10, 0, pulselen);
     pwm.setPWM(11, 0, pulselen);
     pwm.setPWM(12, 0, pulselen);
     pwm.setPWM(13, 0, pulselen);
     pwm.setPWM(14, 0, pulselen);

     pwm.setPWM(15, 0, pulselen);
   }
   delay(10);

}

