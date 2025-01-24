#include <Arduino.h>

void setup() {
    pinMode(5, INPUT);
}

void loop() {
    Serial.begin(9600);
    Serial.println("Loop running"); // Add this line to ensure loop is running
    bool newSwitchState = digitalRead(5);
    Serial.println(newSwitchState);
}