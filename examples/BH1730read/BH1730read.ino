#include <Arduino.h>
#include <BH1730.h>

BH1730 lightSensor = BH1730();

void setup() {
  Serial.begin(9600);
  if(lightSensor.begin() != true){
    Serial.println("Could not find the BH1730 sensor.");
  }
}

void loop() {
  Serial.println(lightSensor.readLux());
  delay(500);
}