#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DFRobot_I2C_Multiplexer.h>

// Define the OLED display dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Create the I2C multiplexer object, passing the I2C bus and the address (default 0x70)
DFRobot_I2C_Multiplexer multiplexer(&Wire, 0x70);

// Initialize the OLED display with I2C address 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
  Serial.begin(9600);

  // Initialize I2C communication
  Wire.begin();

  // Initialize the multiplexer
  multiplexer.begin();

  // Select port 0 (or whichever port your OLED is connected to on the multiplexer)
  multiplexer.selectPort(2);  // Change the port number if your OLED is on a different port
  Serial.println(F("Selected port 0 on multiplexer"));

  // Initialize the OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Clear the buffer
  display.clearDisplay();

  // Set text size, color, and cursor position
  display.setTextSize(1);     
  display.setTextColor(SSD1306_WHITE);  
  display.setCursor(0, 0);

  // Display "Hello, World!" on the screen
  display.println(F("Hello, World!"));
  display.display();  // Render the display
}

void loop() {
  // Nothing to do here
}
