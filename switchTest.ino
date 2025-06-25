#define SWITCH_PIN 2  // Pin where the toggle switch is connected

void setup() {
    pinMode(SWITCH_PIN, INPUT_PULLUP);  // Enable internal pull-up resistor
    Serial.begin(115200);  // Start serial communication
    Serial.println("Toggle switch test started...");
}

void loop() {
    int switchState = digitalRead(SWITCH_PIN); // Read the switch state

    if (switchState == LOW) {
        Serial.println("Switch is ON");
    } else {
        Serial.println("Switch is OFF");
    }

    delay(500); // Add a small delay to prevent flooding the serial monitor
}
