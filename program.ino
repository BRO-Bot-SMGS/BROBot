#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <QTRSensors.h>
#include <EEPROM.h>
#include <string.h>
#include <Bounce2.h>
#include "Adafruit_VL53L0X.h"
#include "Adafruit_TCS34725.h"
// #include <DFRobot_I2C_Multiplexer.h>

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
#define CALIBRATION_SWITCH 24
#define PROGRAM_SWITCH 32
#define SETTINGS_SWITCH 36
// ultrasonic sensor
#define ULTRASONIC_TRIG 2
#define ULTRASONIC_ECHO 4
byte triggerPin = 21;
byte echoCount = 2;
byte* echoPins = new byte[echoCount] { 12, 13 };
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

const uint8_t sensorCount = 13;

byte SENSOR_PIN_ARRAY[SENSOR_ARRAY_SIZE] = 
    {SENSOR_ARRAY_1, SENSOR_ARRAY_2, SENSOR_ARRAY_3, SENSOR_ARRAY_4, SENSOR_ARRAY_5, SENSOR_ARRAY_6, SENSOR_ARRAY_7, SENSOR_ARRAY_8, SENSOR_ARRAY_9, SENSOR_ARRAY_10, SENSOR_ARRAY_11, SENSOR_ARRAY_12, SENSOR_ARRAY_13};

const uint8_t sensorPins[sensorCount] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12};

//i2c addresses
#define SDA_PIN 20
#define SCL_PIN 21
#define OLED_1_I2C_ADDRESS 0x78 // or 0x7a
#define OLED_2_I2C_ADDRESS 0x78 // or 0x7a
#define RGB_I2C_ADDRESS 0x05
#define TOF_I2C_ADDRESS 0x06
#define MULTIPLEXER_I2C_ADDRESS 0x70
#define _I2C_ADDRESS 0x08

#define OLED_1_MULTIPLEXER_PORT 0
#define OLED_2_MULTIPLEXER_PORT 2
#define RGB_I2C_PORT 3
#define TOF_I2C_PORT 6


//settings
//PID SETTINGS
#define kp 0.05 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define kd 2 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define ki 0.001 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
//speeds out of 255
#define rightMaxSpeed 200 // max speed of the robot
#define leftMaxSpeed 200 // max speed of the robot
#define rightBaseSpeed 150 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 150  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define timeout 2500  // waits for 2500 us for sensor outputs to go low
static int lastError = 0;
static float integral = 0;

//LINE ARRAY SETTINGS
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
//rgb autorange
#define TCS34725_R_Coef 0.136
#define TCS34725_G_Coef 1.000
#define TCS34725_B_Coef -0.444
#define TCS34725_GA 1.0
#define TCS34725_DF 310.0
#define TCS34725_CT_Coef 3810.0
#define TCS34725_CT_Offset 1391.0

//variables
bool can_detected = false;
float can_distance;
unsigned int calibratedMinimumOn[sensorCount];
unsigned int calibratedMaximumOn[sensorCount];
bool oldCalibrationSwitchState = false;
float calibrationStartTime;
String calibrationStatus = "Calibrated - outdated";
float duration, distance;  
bool calibrationSwitchState = false;
#define NUM_BUTTONS 3
#define NUM_SENSORS 13
#define targetPosition 3500 // Adjust this value based on your sensor configuration
float serialSpeed = 115200; // 4800, 9600, 38400, 115200
String robotStatus = "Idle";
bool systemReady = false;
float calibrationTime;
int calibrationAmount;
int errorCode = 0;
int speed = 0;
bool programRunning = false;
bool greenZoneDetected = false;
//eeprom addresses
#define lastCalibrationAmountAddress 101
#define lastCalibrationTimeAddress 102
#define lastCalibrationDataAddress 103

//global setup
    // multiplexer setup

    // sensor array setup
    // For QTRSensorsAnalog
QTRSensors qtra;
// QTRSensors qtra((unsigned char[]) {SENSOR_ARRAY_1, SENSOR_ARRAY_2, SENSOR_ARRAY_3, SENSOR_ARRAY_4, SENSOR_ARRAY_5, SENSOR_ARRAY_6, SENSOR_ARRAY_7, SENSOR_ARRAY_8, SENSOR_ARRAY_9, SENSOR_ARRAY_10, SENSOR_ARRAY_11, SENSOR_ARRAY_12, SENSOR_ARRAY_13}, SENSOR_ARRAY_SIZE, EMITTER_PIN);
    // OLED display setup
Adafruit_SSD1306 display[2] = {
    Adafruit_SSD1306(OLED_DISPLAY_WIDTH, OLED_DISPLAY_HEIGHT, &Wire, -1), //display 1 - use it with display[0]
    Adafruit_SSD1306(OLED_DISPLAY_WIDTH, OLED_DISPLAY_HEIGHT, &Wire, -1) //display 2 - use it with display[1]
};
// bounce2 button setup
Bounce calibrationSwitch = Bounce();
Bounce programSwitch = Bounce();
Bounce settingsSwitch = Bounce();
//functions



void setPinModes(){
    // sensor array
    for (int i = 0; i < SENSOR_ARRAY_SIZE; i++){
        pinMode(SENSOR_PIN_ARRAY[i], INPUT);
        }
    // toggle switches
    pinMode(CALIBRATION_SWITCH, INPUT_PULLUP);
    pinMode(PROGRAM_SWITCH, INPUT_PULLUP);
    pinMode(SETTINGS_SWITCH, INPUT_PULLUP);
    // ultrasonic sensor
    pinMode(ULTRASONIC_ECHO, INPUT);
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    // motor control
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

void buttonBounceSetup(){
    calibrationSwitch.attach(CALIBRATION_SWITCH, INPUT_PULLUP);
    calibrationSwitch.interval(50); // Set debounce interval to 50ms

    programSwitch.attach(PROGRAM_SWITCH, INPUT_PULLUP);
    programSwitch.interval(50); // Set debounce interval to 50ms

    settingsSwitch.attach(SETTINGS_SWITCH, INPUT_PULLUP);
    settingsSwitch.interval(50); // Set debounce interval to 50ms
}

void resetDefaults(){
    digitalWrite(ULTRASONIC_TRIG, LOW);
    digitalWrite(EMITTER_PIN, HIGH);
    pinMode(CALIBRATION_SWITCH, INPUT_PULLUP);
    pinMode(PROGRAM_SWITCH, INPUT_PULLUP);
}

//get stored values and store to variables
void retrieveEEPROMvalues() {
EEPROM.get(lastCalibrationTimeAddress, calibrationTime);
EEPROM.get(lastCalibrationAmountAddress, calibrationAmount);
}

int calculateUltrasonic(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);  
	  delayMicroseconds(2);  
	  digitalWrite(trigPin, HIGH);  
	  delayMicroseconds(10);  
	  digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = (duration*.0343)/2;
    Serial.print("Ultrasonic distance: ");
    Serial.println(distance);
    return distance;
}



void initOLED(int displayNumber, int display_I2C, int textSize, int displayWidth, int displayHeight, int multiplexerPort){
    String displayInstance = displayNumber + "display";
    Serial.println("OLED " + String(displayNumber) + " init started");

    if(!display[displayNumber].begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        return; // Exit function if initialization fails
    }

    display[displayNumber].clearDisplay(); // clear the display

    selectMultiplexerPort(multiplexerPort);
    display[displayNumber].clearDisplay(); // clear the display
    display[displayNumber].setTextSize(textSize); // set text size
    display[displayNumber].setTextColor(SSD1306_WHITE); // set text color (won't actually change anything)
    display[displayNumber].setCursor(0, 0); // set cursor position

    Serial.println("OLED " + String(displayNumber) + " init complete");
}

void initDisplays(int display1_I2Cport, int display2_I2Cport, int textSize, int brightness, int contrast, int displayWidth, int displayHeight, int multiplexerPort1, int multiplexerPort2){
    initOLED(0, display1_I2Cport, textSize, displayWidth, displayHeight, multiplexerPort1);
    initOLED(1, display2_I2Cport, textSize, displayWidth, displayHeight, multiplexerPort2);
    displayTest();
    // delay(2000);
    // displaySensorData();
    // displayCalibrationData();
    // calibration display stuff
}

void displaySensorData(int displayNumber){
    display[displayNumber].clearDisplay();
    display[displayNumber].setTextSize(1);
    display[displayNumber].setTextColor(SSD1306_WHITE);
    display[displayNumber].setCursor(0, 0);
    display[displayNumber].println(F("Sensor Data:"));
    display[displayNumber].display();
}

void displayCalibrationData(int displayNumber){
    display[displayNumber].clearDisplay();
    display[displayNumber].setTextSize(2);
    display[displayNumber].setTextColor(SSD1306_WHITE);
    display[displayNumber].setCursor(0, 0);
    display[displayNumber].println((robotStatus));
    display[displayNumber].setCursor(0, 20);
    display[displayNumber].setTextSize(1);
    display[displayNumber].println(("Cali Status: " + calibrationStatus));
    display[displayNumber].setCursor(0, 37);
    display[displayNumber].println(("Time: " + String(float(calibrationTime/1000)) + "s"));
    display[displayNumber].setCursor(0, 47);
    display[displayNumber].println("Min: " + String(calibratedMinimumOn[0]));
    display[displayNumber].setCursor(0, 55);
    display[displayNumber].println("Max: " + String(calibratedMaximumOn[0]));
    display[displayNumber].display();
}

void displayTest(){
    //display 0
    selectMultiplexerPort(OLED_1_MULTIPLEXER_PORT);
    delay(40);
    display[0].clearDisplay();
    display[0].setTextColor(SSD1306_WHITE);
    display[0].setTextSize(2);
    display[0].setCursor(0, 0);
    display[0].println(F("Loading..."));
    display[0].setTextSize(1);
    display[0].setCursor(0, 16);
    display[0].println(F("Please Wait"));
    display[0].setTextSize(1);
    display[0].setCursor(0, 54);
    display[0].println(F("Display 0"));
    display[0].display();
    Serial.println("Display 0 test displayed");

    delay(20);

    //display 1
    selectMultiplexerPort(OLED_2_MULTIPLEXER_PORT);
    delay(40);
    display[1].clearDisplay();
    display[1].setTextColor(SSD1306_WHITE);
    display[1].setTextSize(2);
    display[1].setCursor(0, 0);
    display[1].println(F("BRObotMini"));
    display[1].setTextSize(2);
    display[1].setCursor(0, 20);
    display[1].println(F("Billy and Rufus"));
    display[1].display();
    display[1].setTextSize(1);
    display[1].setCursor(0, 54);
    display[1].println(F("Display 1"));
    display[1].display();
    Serial.println("Display 1 test displayed");
}

void printCalibrationResults(){
    Serial.println("Calibrated Minimum Values:");
    for (int i = 0; i < sensorCount; i++){
        Serial.print(calibratedMinimumOn[i]);
    }
    Serial.println();

    Serial.println("Calibrated Maximum Values:");
    for (int i = 0; i < sensorCount; i++){
        Serial.print(calibratedMaximumOn[i]);
    }
    Serial.println();
}

void calibrateSensor(){
    qtra.calibrate();
    calibrationAmount++;
}

void longTermMemoryStore(int address, int value){
    // EEPROM.write(address, value); // Write the value to the specified address **BE CAREFUL max 100k writes
    Serial.println("Stored " + String(value) + " at address " + String(address)); 
}



// motor stuff

void initMotors(){
    //motor 1
    pinMode(MOTOR1_PWM, OUTPUT);
    pinMode(MOTOR1_DIR, OUTPUT);
    pinMode(MOTOR1_BRAKE, OUTPUT);
    //motor 2
    pinMode(MOTOR2_PWM, OUTPUT);
    pinMode(MOTOR2_DIR, OUTPUT);
    pinMode(MOTOR2_BRAKE, OUTPUT);
}
void setM1Speed(int speed){
    if (speed < 0){
        digitalWrite(MOTOR1_DIR, HIGH);
        analogWrite(MOTOR1_PWM, -speed);
    }
    else{
        digitalWrite(MOTOR1_DIR, LOW);
        analogWrite(MOTOR1_PWM, speed);
    }
}
void setM2Speed(int speed){
    if (speed < 0){
        digitalWrite(MOTOR2_DIR, HIGH);
        analogWrite(MOTOR2_PWM, -speed);
    }
    else{
        digitalWrite(MOTOR2_DIR, LOW);
        analogWrite(MOTOR2_PWM, speed);
    }
}
void setMotorSpeeds(int m1Speed, int m2Speed){
    setM1Speed(m1Speed);
    setM2Speed(m2Speed);
}
void setM1Brake(int brake){
    digitalWrite(MOTOR1_BRAKE, brake);
}
void setM2Brake(int brake){
    digitalWrite(MOTOR2_BRAKE, brake);
}
void setMotorBrakes(int m1Brake, int m2Brake){
    setM1Brake(m1Brake);
    setM2Brake(m2Brake);
}
void setMotorSpeedsAndBrakes(int m1Speed, int m2Speed, int m1Brake, int m2Brake){
    setMotorSpeeds(m1Speed, m2Speed);
    setMotorBrakes(m1Brake, m2Brake);
}
void stopMotors(){
    setMotorSpeedsAndBrakes(0, 0, 1, 1);
}


void motorTest() {
    
}

void updateDisplayStatus(String status) {
    robotStatus = status;
    selectMultiplexerPort(OLED_2_MULTIPLEXER_PORT);
    delay(20);
    displayCalibrationData(1);
}

void initSensorArray(int emitterPin){
    qtra.setTypeAnalog();
    qtra.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, sensorCount);
    qtra.setEmitterPin(emitterPin);
    Serial.println("Sensor array initialized");
}

void selectMultiplexerPort(int port) {
    if (port > 7) return;  // Multiplexer has 8 channels (0-7)
    
    Wire.beginTransmission(MULTIPLEXER_I2C_ADDRESS);
    Wire.write(1 << port);  // Send the channel bitmask
    Wire.endTransmission();

    Serial.println("Selected port " + String(port) + " on multiplexer");
}

void loadDefaultDisplay() {
    //settings
    selectMultiplexerPort(OLED_1_MULTIPLEXER_PORT);
    delay(40);
    display[0].clearDisplay();
    display[0].setTextColor(SSD1306_WHITE);
    // outline
    display[0].setTextSize(2);
    display[0].setCursor(0, 0);
    if (errorCode == 0) {
        display[0].println("BRObot");
    }
    else if (errorCode != 0) {
        display[0].println(("Err#: ") + String(errorCode));
    }
    display[0].setTextSize(1);
    display[0].setCursor(0, 54);
    display[0].println(("Billy and Rufus"));
    // info
    display[0].setCursor(0, 20);
    display[0].println(("Speed: " + String(speed)));
    display[0].setCursor(0, 30);
    display[0].println(("Rbt Stus: " + String(robotStatus)));
    display[0].display();
    Serial.println("Default display loaded");

    //display 2
    selectMultiplexerPort(OLED_2_MULTIPLEXER_PORT);
    delay(40);
    displayCalibrationData(1);
}

void setup() {
    Serial.begin(serialSpeed);
    Serial.println("");
    Serial.println("Power on, starting setup - wait 0.5 seconds");
    delay(200);
    Wire.begin();
    setPinModes();
    retrieveEEPROMvalues();
    initMotors();

Serial.println("Selecting OLED 1 Port...");
selectMultiplexerPort(OLED_1_MULTIPLEXER_PORT);
delay(30);
Serial.println("Selecting OLED 2 Port...");
selectMultiplexerPort(OLED_2_MULTIPLEXER_PORT);
delay(30);

    initDisplays(OLED_1_I2C_ADDRESS, OLED_2_I2C_ADDRESS, 1, brightness, contrast, OLED_DISPLAY_WIDTH, OLED_DISPLAY_HEIGHT, OLED_1_MULTIPLEXER_PORT, OLED_2_MULTIPLEXER_PORT);
    initSensorArray(EMITTER_PIN);
    buttonBounceSetup();
    Serial.println("Setup complete");
    delay(650);
    loadDefaultDisplay();
    Serial.println("Default display loaded");
    systemReady = true;
}


void loop() {

    // calibration

    // update calibration switch
    calibrationSwitch.update();
    bool calibrationSwitchState = calibrationSwitch.read() == HIGH;

    if (calibrationSwitch.fell()) { // if the calibration switch is turned off do this once
        calibrationSwitchState = false;
        selectMultiplexerPort(OLED_2_MULTIPLEXER_PORT);
        calibrationTime = millis() - calibrationStartTime;
        robotStatus = "UpdCalData";
        calibrationStatus = "Saving calibration data to EEPROM";
        displayCalibrationData(1);
        Serial.println("Calibration switch switched to LOW, stopping calibration...");
        Serial.println("Saving calibration data to EEPROM...");
        for (int i = 0; i < sensorCount; i++) { // store calibration data to EEPROM
            longTermMemoryStore(i, calibratedMinimumOn[i]);
            longTermMemoryStore(i + sensorCount, calibratedMaximumOn[i]);
        }
        longTermMemoryStore(lastCalibrationAmountAddress, calibrationAmount);
        longTermMemoryStore(lastCalibrationTimeAddress, (millis() - calibrationStartTime));
        Serial.println("Calibration data saved to EEPROM");
        Serial.println("Calibration data:");
        Serial.println("Updating display with calibration data...");
        robotStatus = "Idle";
        calibrationStatus = "Calibrated this session (:";
        displayCalibrationData(1);
        Serial.println("Display updated with calibration data");
        Serial.println("Calibration complete");
    }

    if (calibrationSwitch.rose()) { // if the calibration switch is turned on do this once
        calibrationSwitchState = true;
        selectMultiplexerPort(OLED_2_MULTIPLEXER_PORT);
        Serial.println("Calibration switch switched to HIGH, calibrating...");
        calibrationStartTime = millis();
        calibrationAmount = 0;
        robotStatus = "Calibratng";
        calibrationStatus = "Calibrating line array...";
    }
    if (calibrationSwitchState) { // while the calibration switch is on do this (loop)
        calibrateSensor();
        displayCalibrationData(1);
        delay(15);
    }

    // line following
    
    // update calibration switch
    programSwitch.update();
    bool programSwitchState = programSwitch.read() == HIGH;

    if (programSwitch.fell()) { // if the program switch is turned off do this once
        updateDisplayStatus("Stpng Prog");
        stopMotors();
        updateDisplayStatus("Idle");
        programRunning = false;
    }

    if (programSwitch.rose()) { // if the program switch is turned on do this once
        updateDisplayStatus("Runng Prog");
        setMotorBrakes(0, 0);
        programRunning = true;
    }

    if (programSwitchState) { // while the program switch is on do this (loop)
    
        uint16_t sensorValues[NUM_SENSORS];
        int position = qtra.readLineBlack(sensorValues); // Use readLineWhite for a white line on a black background

        // Calculate error.
        int error = position - targetPosition;

        // Calculate integral.
        integral += error;

        // Limit the integral term to prevent windup.
        integral = constrain(integral, -500, 500);

        // Calculate derivative.
        int derivative = error - lastError;

        // Calculate PID output.
        float pidOutput = kp * error + ki * integral + kd * derivative;

        // Calculate motor speeds.
        int leftSpeed = leftBaseSpeed - pidOutput;
        int rightSpeed = rightBaseSpeed + pidOutput;

        // Constrain motor speeds to valid range (0-255 for PWM).
        leftSpeed = constrain(leftSpeed, 0, 255);
        rightSpeed = constrain(rightSpeed, 0, 255);

        // Control motors (replace with your motor control logic).
        if (leftSpeed > leftBaseSpeed) {
            setM1Speed(leftSpeed); // Forward
        } else {
            setM1Speed(-leftSpeed); // Reverse
        }

        if (rightSpeed > rightBaseSpeed) {
            setM2Speed(rightSpeed); // Forward
        } else {
            setM2Speed(-rightSpeed); // Reverse
        }

        // Update last error.
        lastError = error;
    
    }
    
    // look for obstacles
    if (programRunning) {
        can_distance = calculateUltrasonic(ULTRASONIC_TRIG, ULTRASONIC_ECHO);
        stopMotors();
        }
        else {
            can_detected = false;
        }

    // look for green zone

    if (programRunning && greenZoneDetected) {
        stopMotors();
    }
    else {
        greenZoneDetected = false;
    }
}