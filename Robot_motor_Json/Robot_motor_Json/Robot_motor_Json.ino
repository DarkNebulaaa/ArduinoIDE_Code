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
 unsigned long lastTime;

 unsigned long TaskTime;
StaticJsonDocument<300> motorDoc;  //JsonFile recive from TX2

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();////以這種方式呼叫，它使用預設地址0x40。
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
//這是“最小”脈衝長度計數（在4096）中
#define SERVOMAX  500 // this is the 'maximum' pulse length count (out of 4096)
//這是“最大”脈衝長度計數（在4096中）

/*######### Motor parameter ########*/





 volatile float move ;
 volatile float turn ;



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
  //Serial2.begin(115200);

  /*##########################################################################################
############################  Initialize the ESP32 Task  #####################################
#############################################################################################*/

xTaskCreate(
                    ReciveJsonMessage,   /* Task function. */
                    "ReciveJsonMessage",     /* name of task. */
                    20000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    tskIDLE_PRIORITY,           /* priority of the task */
                    &Task1      /* Task handle to keep track of created task */
                  );          /* pin task to core 0 */                  
  delay(500);

/*xTaskCreatePinnedToCore(
                    MotorControl,   
                    "MotorControl",     
                    10000,      
                    NULL,        
                    0,           
                    &Task2,      
                    1);                          
  delay(500);
*/
/*##########################################################################################
############################  Initialize the pwm motor  #####################################
#############################################################################################*/

pwm.begin();

pwm.setPWMFreq(60);

}

 
void loop()
 {
   delay(1);
}

/*##########################################################################################
######################################  Function Define ####################################
##########################################################################################*/

void ReciveJsonMessage(void * arg)
{ 
 for(;;)
  {
   
 

          DeserializationError err = deserializeJson(motorDoc, Serial);






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
                while(millis() - lastTime <= TaskTime)
                {
                  
                }
                 xTaskCreate(MotorControl, "MotorControl", 10000, NULL,tskIDLE_PRIORITY, &Task2);
                 //vTaskDelete(NULL);
            } 
          else 
            {
              // Print error to the "debug" serial port
              Serial.print("deserializeJson() returned ");
              Serial.println(err.c_str());
  
              // Flush all bytes in the "link" serial port buffer
               while (Serial.available() > 0)
               Serial.read();
            }
         
          
     
   }

}


void MotorControl(void *pvParameters)
{
  //for(;;)
 // { 
      lastTime = millis();
      if(move>0)
        {
          pwm.setPWM(0, 0, 650);
         //Serial.println(move);
          
        }

      else 
        {
         pwm.setPWM(0, 0,380);
        //Serial.println(move);
        }
        TaskTime = millis() - lastTime;
      vTaskDelete(NULL);

    
    
    
 // }
}
void someFunction(void* arg) {
  
}
