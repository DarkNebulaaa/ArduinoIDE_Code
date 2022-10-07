#include <ArduinoJson.h>
#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"
/*  including file at the upside */
/*===============================*/
/*===============================*/
/*===============================*/
/*          Task Define          */

TaskHandle_t Task1 ;// This Task keep track of ReciveJsonMessage Function
TaskHandle_t Task2 ;// This Task keep track of MotorControl Function


/*       global    parameter     */

StaticJsonDocument<300> motorDoc;  //JsonFile recive from TX2

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();////以這種方式呼叫，它使用預設地址0x40。
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
//這是“最小”脈衝長度計數（在4096）中
#define SERVOMAX  500 // this is the 'maximum' pulse length count (out of 4096)
//這是“最大”脈衝長度計數（在4096中）

/*######### Motor parameter ########*/

float move ;
float turn ;
/*===============================*/
void setup() {

 
  /*##########################################################################################
#########################  Initialize all of the serial port #################################
#############################################################################################*/

  // Initialize "debug" serial port
  // The data rate must be much higher than the "link" serial port
  Serial.begin(115200);
  //while (!Serial) continue;

  // Initialize the "link" serial port
  // Use a low data rate to reduce the error ratio
  Serial2.begin(115200);

  /*##########################################################################################
############################  Initialize the ESP32 Task  #####################################
#############################################################################################*/

xTaskCreatePinnedToCore(
                    ReciveJsonMessage,   /* Task function. */
                    "ReciveJsonMessage",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500);

xTaskCreatePinnedToCore(
                    MotorControl,   /* Task function. */
                    "MotorControl",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 0 */                  
  delay(500);

/*##########################################################################################
############################  Initialize the pwm motor  #####################################
#############################################################################################*/

pwm.begin();

pwm.setPWMFreq(60);

}

 
void loop() {

}

/*##########################################################################################
######################################  Function Define ####################################
##########################################################################################*/

void ReciveJsonMessage(void * pvParameters)
{ 
  for(;;)
  {
      if (Serial2.available()) 
    {
    // Allocate the JSON document
    // This one must be bigger than the sender's because it must store the strings
    //StaticJsonDocument<300> motorDoc;

    // Read the JSON document from the "link" serial port
    DeserializationError err = deserializeJson(motorDoc, Serial2);

    if (err == DeserializationError::Ok) 
    {
      // Print the values
      // (we must use as<T>() to resolve the ambiguity)
      //Serial.print("move = ");
      //Serial.print(motorDoc["move"].as<float>());
      move = motorDoc["move"].as<float>();
      //Serial.print("  ; turnValue = ");
      //Serial.println(motorDoc["turn"].as<float>());
      turn = motorDoc["turn"].as<float>();
    } 
    else 
      {
      // Print error to the "debug" serial port
         Serial.print("deserializeJson() returned ");
          Serial.println(err.c_str());
  
      // Flush all bytes in the "link" serial port buffer
          while (Serial2.available() > 0)
            Serial2.read();
      }
    } 
    delay(1);
  }
}


void MotorControl(void *pvParameters)
{
  for(;;)
  {
    if(move>0)
    {
      pwm.setPWM(0, 0, 500);
      Serial.println(move);
    }
    
    else 
    {
      pwm.setPWM(0, 0, 200);
      Serial.println(move);
    }
  }
}
