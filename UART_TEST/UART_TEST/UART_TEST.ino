#include <ArduinoJson.h>


/*  including file at the upside  */

/*       global    parameter      */


/*===============================*/
void setup() {
  // Initialize "debug" serial port
  // The data rate must be much higher than the "link" serial port
  Serial.begin(115200);
  while (!Serial) continue;

  // Initialize the "link" serial port
  // Use a low data rate to reduce the error ratio
  Serial2.begin(115200);







}
 
void loop() {
  // Check if the other Arduino is transmitting
  if (Serial2.available()) 
  {
    // Allocate the JSON document
    // This one must be bigger than the sender's because it must store the strings
    StaticJsonDocument<300> motorDoc;

    // Read the JSON document from the "link" serial port
    DeserializationError err = deserializeJson(motorDoc, Serial2);

    if (err == DeserializationError::Ok) 
    {
      // Print the values
      // (we must use as<T>() to resolve the ambiguity)
      Serial.print("move = ");
      Serial.println(motorDoc["move"].as<float>());
      //Serial.print("value = ");
     //Serial.println(doc["value"].as<int>());
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
}



