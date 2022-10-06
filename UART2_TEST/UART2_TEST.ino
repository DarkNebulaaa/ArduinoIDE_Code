#include <ArduinoJson.h>

void setup() {
  // Initialize "debug" serial port
  // The data rate must be much higher than the "link" serial port
  Serial.begin(9600);
  while (!Serial) continue;

  // Initialize the "link" serial port
  // Use a low data rate to reduce the error ratio
  
}
 
void loop() {
  // Values we want to transmit
  long timestamp = millis();
  int value = analogRead(1);

  // Print the values on the "debug" serial port


  // Create the JSON document
  StaticJsonDocument<200> doc;
  doc["timestamp"] = timestamp;
  doc["value"] = value;

  // Send the JSON document over the "link" serial port
  serializeJson(doc, Serial);

  // Wait
  delay(5000);
}