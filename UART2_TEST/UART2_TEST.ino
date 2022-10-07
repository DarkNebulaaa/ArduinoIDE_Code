
#include <ArduinoJson.h>

    float moveValue ;
    float turnValue ;
#define BUTTON 2
void setup() {
  // Initialize "debug" serial port
  // The data rate must be much higher than the "link" serial port
  Serial.begin(115200);
  Serial2.begin(115200);
  while (!Serial) continue;

  // Initialize the "link" serial port
  // Use a low data rate to reduce the error ratio
  pinMode(BUTTON ,INPUT_PULLUP);
}
 
void loop() {
  // Values we want to transmit
  if(digitalRead(BUTTON))
  {
     moveValue = 1.0;
     turnValue = 0.5;
  }
  else
  {
     moveValue = 0;
     turnValue = 0;
  }
  

  // Print the values on the "debug" serial port


  // Create the JSON document
  StaticJsonDocument<200> motorDoc;
  motorDoc["move"] = moveValue;
  motorDoc["turn"] = turnValue;

  // Send the JSON document over the "link" serial port
  serializeJson(motorDoc, Serial2);
  Serial.print(" moveValue = ");
  Serial.print(moveValue);
  Serial.print(" ; trunValue = ");
  Serial.println(turnValue);
  // Wait
  delay(1);
}