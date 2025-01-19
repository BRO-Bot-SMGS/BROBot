#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <QTRSensors.h>
#include <DFRobot_I2C_Multiplexer.h>

//pins
    //motor
#define MOTOR1_PWM 3
#define MOTOR1_DIR 12
#define MOTOR1_BRAKE 8
#define MOTOR2_PWM 11
#define MOTOR2_DIR 13
#define MOTOR2_BRAKE 9
    //sensors
#define ULTRASONIC_TRIG 2
#define ULTRASONIC_ECHO 4
#define CALIBRATION_SWITCH 5
#define PROGRAM_SWITCH 6
#define SETTINGS_SWITCH 7
        //sensor array
#define SENSOR_ARRAY_1 0
#define SENSOR_ARRAY_2 1
#define SENSOR_ARRAY_3 2
#define SENSOR_ARRAY_4 3
#define SENSOR_ARRAY_5 4
#define SENSOR_ARRAY_6 5
#define SENSOR_ARRAY_7 6
#define SENSOR_ARRAY_8 7
#define SENSOR_ARRAY_9 8
#define SENSOR_ARRAY_10 9
#define SENSOR_ARRAY_11 10
#define SENSOR_ARRAY_12 11
#define SENSOR_ARRAY_13 12

#define SENSOR_ARRAY_SIZE 13
#define EMITTER_PIN 34

const uint8_t SensorCount = 13;

byte SENSOR_PIN_ARRAY[SENSOR_ARRAY_SIZE] = 
    {SENSOR_ARRAY_1, SENSOR_ARRAY_2, SENSOR_ARRAY_3, SENSOR_ARRAY_4, SENSOR_ARRAY_5, SENSOR_ARRAY_6, SENSOR_ARRAY_7, SENSOR_ARRAY_8, SENSOR_ARRAY_9, SENSOR_ARRAY_10, SENSOR_ARRAY_11, SENSOR_ARRAY_12, SENSOR_ARRAY_13};

//i2c addresses
#define SDA_PIN 20
#define SCL_PIN 21
#define OLED_1_I2C_ADDRESS 0x78 // or 0x7a
#define OLED_2_I2C_ADDRESS 0x78 // or 0x7a
#define RGB_I2C_ADDRESS 0x05
#define TOF_I2C_ADDRESS 0x06
#define MULTIPLEXER_I2C_ADDRESS 0x70
#define _I2C_ADDRESS 0x08

#define OLED_1_I2C_PORT 0
#define OLED_2_I2C_PORT 2
#define RGB_I2C_PORT 3
#define TOF_I2C_PORT 6


//settings
float speed = 1;
float kp = 0.5;
float ki = 0.5;
float kd = 0.5;
float samplesPerRead = 4;
float recommendedCalibrationTime = 12;
int brightness = 255;
int contrast = 255;
#define OLED_DISPLAY_WIDTH 128
#define OLED_DISPLAY_HEIGHT 64
//rgb colors 
byte greenColor[3] = {0, 255, 0};
byte whiteColor[3] = {255, 0, 0};
byte blackColor[3] = {0, 0, 255};


//variables
bool can_detected = false;
float can_distance = 0;
unsigned int calibratedMinimumOn[SensorCount];
unsigned int calibratedMaximumOn[SensorCount];

//global setup
    // multiplexer setup
DFRobot_I2C_Multiplexer multiplexer(&Wire, MULTIPLEXER_I2C_ADDRESS);
    // sensor array setup
    // For QTRSensorsAnalog
QTRSensors qtra;
// QTRSensors qtra((unsigned char[]) {SENSOR_ARRAY_1, SENSOR_ARRAY_2, SENSOR_ARRAY_3, SENSOR_ARRAY_4, SENSOR_ARRAY_5, SENSOR_ARRAY_6, SENSOR_ARRAY_7, SENSOR_ARRAY_8, SENSOR_ARRAY_9, SENSOR_ARRAY_10, SENSOR_ARRAY_11, SENSOR_ARRAY_12, SENSOR_ARRAY_13}, SENSOR_ARRAY_SIZE, EMITTER_PIN);
    // OLED display setup
Adafruit_SSD1306 display[2] = {
    Adafruit_SSD1306(OLED_DISPLAY_WIDTH, OLED_DISPLAY_HEIGHT, &Wire, -1), //display 1 - use it with display[0]
    Adafruit_SSD1306(OLED_DISPLAY_WIDTH, OLED_DISPLAY_HEIGHT, &Wire, -1) //display 2 - use it with display[1]
};

//functions



void setPinModes(){
    for (int i = 0; i < SENSOR_ARRAY_SIZE; i++){
        pinMode(SENSOR_PIN_ARRAY[i], INPUT);
    }
    pinMode(CALIBRATION_SWITCH, INPUT);
    pinMode(PROGRAM_SWITCH, INPUT);
    pinMode(SETTINGS_SWITCH, INPUT);
    pinMode(MOTOR1_PWM, OUTPUT);
    pinMode(MOTOR1_DIR, OUTPUT);
    pinMode(MOTOR1_BRAKE, OUTPUT);
    pinMode(MOTOR2_PWM, OUTPUT);
    pinMode(MOTOR2_DIR, OUTPUT);
    pinMode(MOTOR2_BRAKE, OUTPUT);
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    pinMode(EMITTER_PIN, OUTPUT);
}

void resetDefaults(){
    digitalWrite(MOTOR1_PWM, LOW);
    digitalWrite(MOTOR1_DIR, LOW);
    digitalWrite(MOTOR1_BRAKE, LOW);
    digitalWrite(MOTOR2_PWM, LOW);
    digitalWrite(MOTOR2_DIR, LOW);
    digitalWrite(MOTOR2_BRAKE, LOW);
    digitalWrite(ULTRASONIC_TRIG, LOW);
    digitalWrite(EMITTER_PIN, HIGH);
}

void initOLED(int displayNumber, int display_I2C, int textSize, int displayWidth, int displayHeight){
    String displayInstance = displayNumber + "display";
    Serial.println("OLED " + String(displayNumber) + " init started");
    Wire.beginTransmission(display_I2C);
    if(!display[displayNumber].begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
    }
    display[displayNumber].clearDisplay(); // clear the display
    Serial.println("OLED " + String(displayNumber) + " init complete");

    display[displayNumber].clearDisplay(); // clear the display
    display[displayNumber].setTextSize(textSize); // set text size
    display[displayNumber].setTextColor(SSD1306_WHITE); // set text color (won't actually change anything)
    display[displayNumber].setCursor(0, 0); // set cursor position
}

void initDisplays(int display1_I2Cport, int display2_I2Cport, int textSize, int brightness, int contrast, int displayWidth, int displayHeight){
    initOLED(0, display1_I2Cport, textSize, displayWidth, displayHeight);
    initOLED(1, display2_I2Cport, textSize, displayWidth, displayHeight);
    Wire.write(0x00);
    Wire.endTransmission();
    //calibration display stuff

}

void displayTest(int displayNumber){
    display[displayNumber].clearDisplay();
    display[displayNumber].setTextSize(1);
    display[displayNumber].setTextColor(SSD1306_WHITE);
    display[displayNumber].setCursor(0, 0);
    display[displayNumber].println(F("Hellllo, World!"));
    display[displayNumber].display();
}

void calibrateSensor(){
    qtra.calibrate();
}

void printCalibrationResults(){
    Serial.println("Calibrated Minimum Values:");
    for (int i = 0; i < SensorCount; i++){
        Serial.print(calibratedMinimumOn[i]);
    }
    Serial.println();

    Serial.println("Calibrated Maximum Values:");
    for (int i = 0; i < SensorCount; i++){
        Serial.print(calibratedMaximumOn[i]);
    }
    Serial.println();
}

void calibrateSensorsWhile(int switchPin, int calibrationDisplayPin){
    long unsigned int calibrationTimer = millis();
    int calibrations = 0;
    while(digitalRead(switchPin) == HIGH){
        calibrateSensor();
        calibrations++;
        delay(10);
    }
    return calibrations, calibrationTimer;
}

void updateCalibrationDisplay(int calibrations, int calibrationTimer, bool switchStatus){
    display[1].clearDisplay();
    display[1].setTextSize(1);
    display[1].setTextColor(SSD1306_WHITE);
    display[1].println("Status: " + String(switchStatus));
    display[1].setCursor(0, 0);
    display[1].println("Calibrations: " + String(calibrations));
    display[1].setCursor(0, 10);
    display[1].println("Time: " + String(millis() - calibrationTimer));
    display[1].display();
}

void initSensorArray(int emitterPin){
    qtra.setTypeAnalog();
    qtra.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
    qtra.setEmitterPin(emitterPin);
}

void selectMultiplexerPort(int port){
    multiplexer.selectPort(port);
    Serial.println("Selected port " + String(port) + " on multiplexer");
}





void setup() {
    Serial.begin(9600);
    Wire.begin();
    multiplexer.begin();
    selectMultiplexerPort(2);
    setPinModes();
    initDisplays(OLED_1_I2C_ADDRESS, OLED_2_I2C_ADDRESS, 1, brightness, contrast, OLED_DISPLAY_WIDTH, OLED_DISPLAY_HEIGHT);
    displayTest(0);
}


void loop() {
    // int calibrations = 0;
    // int calibrationTimer = 0;
    // if(digitalRead(CALIBRATION_SWITCH) == HIGH){
    //     calibrateSensorsWhile(CALIBRATION_SWITCH, calibrationTimer);
    //     updateCalibrationDisplay(calibrations, calibrationTimer, CALIBRATION_SWITCH);
    // }
    // delay(10);
}