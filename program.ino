#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <QTRSensors.h>
#include <EEPROM.h>
#include <string.h>
#include <Bounce2.h>
#include <Adafruit_VL53L1X.h>
#include <Adafruit_VL6180X.h>
#include "Adafruit_TCS34725.h"
#include <Servo.h>
// #include <DFRobot_I2C_Multiplexer.h>

// -- PINS --
    // led
    // led default (delete this)
#define LED_RED_PIN 10 //pwm pin
#define LED_GREEN_PIN 54 //pwm pin
#define LED_BLUE_PIN 46 //pwm pin
// SPEED LED (top center)
#define SPEED_LED_RED_PIN 28
#define SPEED_LED_GREEN_PIN 48
#define SPEED_LED_BLUE_PIN 32
// ERROR LED (bottom center)
#define ERROR_LED_GREEN_PIN 52
#define ERROR_LED_RED_PIN 53
#define ERROR_LED_BLUE_PIN 22
// POWER LED (bottom right next to power switch) - green plugged into battery after switch
#define POWER_LED_RED_PIN 50
#define POWER_LED_BLUE_PIN 51

    //motor
#define M1_DIR_PIN   7
#define M1_PWM_PIN   6
#define M2_DIR_PIN   4
#define M2_PWM_PIN   5
    // servos
#define SERVO1_PIN 44
#define SERVO2_PIN 45
    //sensors
        //ultrasonic
#define ULTRASONIC_TRIG 2
#define ULTRASONIC_ECHO 4
        // rgb sensor
#define RGB_LEFT_LED 34
#define RGB_RIGHT_LED 33

    // switches
#define PROGRAM_SWITCH 51 // not interupt pin
#define CALIBRATION_SWITCH 47 // not interupt pin
#define SETTINGS_SWITCH 49 // not interupt pin - still available 20, 21


// ultrasonic sensor
#define ULTRASONIC_TRIG 2
#define ULTRASONIC_ECHO 4

        //sensor array
// #define SENSOR_ARRAY_1 A0
// #define SENSOR_ARRAY_2 A1
// #define SENSOR_ARRAY_3 A2
#define SENSOR_ARRAY_4 A3
#define SENSOR_ARRAY_5 A4
#define SENSOR_ARRAY_6 A5
#define SENSOR_ARRAY_7 A6
#define SENSOR_ARRAY_8 A7
#define SENSOR_ARRAY_9 A8
#define SENSOR_ARRAY_10 A9
// #define SENSOR_ARRAY_11 A10
// #define SENSOR_ARRAY_12 A11
// #define SENSOR_ARRAY_13 A12

#define SENSOR_ARRAY_SIZE 13
#define EMITTER_ODD_PIN 36
#define EMITTER_EVEN_PIN 37

// const uint8_t sensorCount = 13;
const uint8_t sensorCount = 7;
// #define NUM_SENSORS 13
#define NUM_SENSORS 7
// #define targetPosition 7000
#define targetPosition (7 * 1000) / 2

// const uint8_t sensorPins[sensorCount] = {
//     SENSOR_ARRAY_1, SENSOR_ARRAY_2, SENSOR_ARRAY_3, SENSOR_ARRAY_4,
//     SENSOR_ARRAY_5, SENSOR_ARRAY_6, SENSOR_ARRAY_7, SENSOR_ARRAY_8,
//     SENSOR_ARRAY_9, SENSOR_ARRAY_10, SENSOR_ARRAY_11, SENSOR_ARRAY_12,
//     SENSOR_ARRAY_13
// };

const uint8_t sensorPins[sensorCount] = {
    SENSOR_ARRAY_4,
    SENSOR_ARRAY_5, SENSOR_ARRAY_6, SENSOR_ARRAY_7, SENSOR_ARRAY_8,
    SENSOR_ARRAY_9, SENSOR_ARRAY_10
};

//i2c addresses
#define SDA_PIN 20
#define SCL_PIN 21
#define OLED_1_I2C_ADDRESS 0x78 // or 0x7a
#define OLED_2_I2C_ADDRESS 0x78 // or 0x7a
#define RGB_I2C_ADDRESS 0x29
#define TOF_I2C_ADDRESS 0x29
#define MULTIPLEXER_I2C_ADDRESS 0x70

#define OLED_1_MULTIPLEXER_PORT 0
#define OLED_2_MULTIPLEXER_PORT 1
#define RGB_LEFT_MULTIPLEXER_PORT 4
#define RGB_RIGHT_MULTIPLEXER_PORT 3
#define TOF_HOLDER_MULTIPLEXER_PORT 5
#define TOF_FRONT_MULTIPLEXER_PORT 2


//settings
//PID SETTINGS
#define kp 0.25 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define kd 5.0// experiment to determine this, slowly increase the speeds and adjust this value.
#define ki 0.0 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
//speeds out of 255
// #define rightMaxSpeed 200 // max speed of the robot
// #define leftMaxSpeed 200 // max speed of the robot
#define rightBaseSpeed 240 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 240 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define timeout 2500  // waits for 2500 us for sensor outputs to go low
static int lastError = 0;
static float integral = 0;

//TOF SETTINGS
int slowDistance = 235; // distance from the bottle to start slowing down (in mm)
int bottleMaxSpeed = 60; // slow speed once bottle is in range
int bottleBaseSpeed = 40; // max speed of the robot
int goAroundDistance = 180; // distance from the bottle to start going around (in mm)
// movement temp variables
int currentRightBaseSpeed = rightBaseSpeed;
int currentLeftBaseSpeed = leftBaseSpeed;
// int currentRightMaxSpeed = rightMaxSpeed;
// int currentLeftMaxSpeed = leftMaxSpeed;
int minTOFdistance = 30; // minimum distance to the bottle to start going around (in mm)
bool lineFollow = true;

//LINE ARRAY SETTINGS
float samplesPerRead = 4;
float recommendedCalibrationTime = 12;
int brightness = 255;
int contrast = 255;
#define OLED_DISPLAY_WIDTH 128
#define OLED_DISPLAY_HEIGHT 64

//RGB SETTINGS
//rgb colors
uint16_t r, g, b, c;
String left_currentColor;
String right_currentColor;
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
float serialSpeed = 230400; // 4800, 9600, 38400, 115200
String robotStatus = "Idle";
bool systemReady = false;
float calibrationTime;
int calibrationAmount;
int errorCode = 0;
int speed = 0;
bool greenZoneDetected = false;
int lineBrightnessMinimum = 100;
bool programDisplayAlertStatus = false;
String programDisplayAlert = "Alert Test";
bool lineLost = false;
int sensorValues[SENSOR_ARRAY_SIZE];
int position = 0;
int error = 0;
int leftSpeed = 0;
int rightSpeed = 0;
int rapidDisplayRefreshDelay = 300;
int eepromCalibrationOffset = 1500;
int frontTOFdistance;
int holderTOFdistance;
bool tofSensorActive = false;
bool obstacleAvoidanceActive = false;
bool programSwitchState = false;
int lastMultiplexerPort = -1;
bool tempObsticleDetected;
// servo variables
int servo1Angle = 90;
int servo2Angle = 90;
// int servo1lastAngle = servo1.read();
// int servo2lastAngle = servo2.read();

int servo1potValue = 0; // potentiometer value for servo 1
int servo2potValue = 0; // potentiometer value for servo 2

// left rgb
// 0 - 65335
int leftRGB_redRaw = 0;
int leftRGB_greenRaw = 0;
int leftRGB_blueRaw = 0;
// 0-255
int leftRGB_red = map(leftRGB_redRaw, 0, 65535, 0, 255);
int leftRGB_green = map(leftRGB_greenRaw, 0, 65535, 0, 255);
int leftRGB_blue = map(leftRGB_blueRaw, 0, 65535, 0, 255);
// right rgb
// 0 - 65335
int rightRGB_redRaw = 0;
int rightRGB_greenRaw = 0;
int rightRGB_blueRaw = 0;
// 0-255
int rightRGB_red = map(rightRGB_redRaw, 0, 65535, 0, 255);
int rightRGB_green = map(rightRGB_greenRaw, 0, 65535, 0, 255);
int rightRGB_blue = map(rightRGB_blueRaw, 0, 65535, 0, 255);

#define EEPROM_MAGIC 0xDEAD
#define EEPROM_ADDR_MAGIC (eepromCalibrationOffset)
#define EEPROM_ADDR_MIN (eepromCalibrationOffset + sizeof(uint16_t))
#define EEPROM_ADDR_MAX (EEPROM_ADDR_MIN + NUM_SENSORS * sizeof(uint16_t))

//eeprom addresses
#define lastCalibrationAmountAddress 101
#define lastCalibrationTimeAddress 102
#define lastCalibrationDataAddress 103
#define EEPROMindex 100

//global setup
    // multiplexer setup

    // sensor array setup
    // For QTRSensorsAnalog
// QTRSensors setup
QTRSensors qtra;
// tof sensor setup
Adafruit_VL53L1X vl53l1x = Adafruit_VL53L1X();

// RGB sensor setup
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
// Servo setup
Servo servo1;
Servo servo2;



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
    for (int i = 0; i < SENSOR_ARRAY_SIZE; i++) {
        pinMode(sensorPins[i], INPUT); // Use sensorPins array here
    }
    // toggle switches
    pinMode(PROGRAM_SWITCH, INPUT_PULLUP);
    pinMode(CALIBRATION_SWITCH, INPUT_PULLUP);
    pinMode(SETTINGS_SWITCH, INPUT_PULLUP);
    // ultrasonic sensor
    pinMode(ULTRASONIC_ECHO, INPUT);
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    // motor control
    pinMode(M1_DIR_PIN, OUTPUT);
    pinMode(M1_PWM_PIN, OUTPUT);
    pinMode(M2_DIR_PIN, OUTPUT);
    pinMode(M2_PWM_PIN, OUTPUT);
    // line array emitters
    pinMode(EMITTER_ODD_PIN, OUTPUT);
    pinMode(EMITTER_EVEN_PIN, OUTPUT);
    // servo control
    pinMode(SERVO1_PIN, OUTPUT);
    pinMode(SERVO2_PIN, OUTPUT);
    //leds
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);
}

void initSensorArray() {
    qtra.setTypeAnalog();
    qtra.setSensorPins(sensorPins, sensorCount);
    qtra.setEmitterPins(EMITTER_ODD_PIN, EMITTER_EVEN_PIN);
    // qtra.setDimmable();
    qtra.emittersOn();
    Serial.println("Sensor array initialized");
}

void initTOFSensors() {
    initTOFSensorHolder();
}

void initTOFSensorFront() {
}

void initTOFSensorHolder() {
    selectMultiplexerPort(TOF_HOLDER_MULTIPLEXER_PORT);
    delay(150);
    if (!vl53l1x.begin(TOF_I2C_ADDRESS, &Wire)) {
        Serial.println("Failed to find VL53L1X sensor!");
        while (1);
    } else {
        Serial.println("VL53L1X ready.");
        vl53l1x.startRanging();
    }
}


void readTOFSensors() {
    // read front TOF sensor
    // selectMultiplexerPort(TOF_FRONT_MULTIPLEXER_PORT);
    // lox.rangingTest(&measure, false);
    // if (measure.RangeStatus != 4) {
    //     Serial.print("Front TOF distance: ");
    //     Serial.print(measure.RangeMilliMeter);
    //     Serial.println(" mm");
    //     frontTOFdistance = measure.RangeMilliMeter;
    // } else {
    //     Serial.println("Front TOF out of range");
    // }

    // read holder TOF sensor
    selectMultiplexerPort(TOF_HOLDER_MULTIPLEXER_PORT);
    if (vl53l1x.dataReady()) {
        holderTOFdistance = vl53l1x.distance();
        Serial.print("Holder VL53L1X: ");
        Serial.print(holderTOFdistance);
        Serial.println(" mm");
        vl53l1x.clearInterrupt();
    } else {
        Serial.println("Holder VL53L1X: Not ready");
    }
}

void initRGBSensor() {
    // init left RGB sensor
    selectMultiplexerPort(RGB_LEFT_MULTIPLEXER_PORT);
    delay(100);
    Serial.println("Left TCS34725 init...");
    if (!tcs.begin()) {
        Serial.println(F("Failed to boot left TCS34725"));
        errorCode = 401;
    }

    // init right RGB sensor
    selectMultiplexerPort(RGB_RIGHT_MULTIPLEXER_PORT);
    delay(100);
    Serial.println("Right TCS34725 init...");
    if (!tcs.begin()) {
        Serial.println(F("Failed to boot right TCS34725"));
        if (errorCode == 401) {
            errorCode = 403;
        }
        else {
            errorCode = 402;
        }
    }

    Serial.println("TCS34725 sensors initialized successfully.");
}

void convertToRGB(int r, int g, int b, int c, int &red, int &green, int &blue) {
    if (c == 0) c = 1;
    red = (uint32_t(r) * 255) / c;
    green = (uint32_t(g) * 255) / c;
    blue = (uint32_t(b) * 255) / c;
}

void readRGBsensors() {
    // read left RGB sensor
    silent_selectMultiplexerPort(RGB_LEFT_MULTIPLEXER_PORT);
    tcs.getRawData(&r, &g, &b, &c);
    convertToRGB(r, g, b, c, leftRGB_red, leftRGB_green, leftRGB_blue);
    
    silent_selectMultiplexerPort(RGB_RIGHT_MULTIPLEXER_PORT);
    tcs.getRawData(&r, &g, &b, &c);
    convertToRGB(r, g, b, c, rightRGB_red, rightRGB_green, rightRGB_blue);
}

void detectColor() {

}

void powerRGB_LEDstate(bool red, bool blue) {
    digitalWrite(POWER_LED_RED_PIN, red);
    digitalWrite(POWER_LED_BLUE_PIN, blue);
    // serial print status
    Serial.print("Power LED state set to red: ");
    Serial.print(red);
    Serial.print(", blue: ");
    Serial.println(blue);
}

void lineArray_emitterState(bool state) {
    if (state) {
        digitalWrite(EMITTER_ODD_PIN, HIGH);
        digitalWrite(EMITTER_EVEN_PIN, HIGH);
    } else {
        digitalWrite(EMITTER_ODD_PIN, LOW);
        digitalWrite(EMITTER_EVEN_PIN, LOW);
    }
    // serial print status
    Serial.print("Line array emitter state set to: ");
    Serial.println(state ? "ON" : "OFF");
}

void calibrateSensor(){
    qtra.calibrate();
    calibrationAmount++;
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
    lineArray_emitterState(true);
    pinMode(CALIBRATION_SWITCH, INPUT_PULLUP);
    pinMode(PROGRAM_SWITCH, INPUT_PULLUP);
}

//get stored values and store to variables
void retrieveEEPROMvalues() {
EEPROM.get(lastCalibrationTimeAddress, calibrationTime);
EEPROM.get(lastCalibrationAmountAddress, calibrationAmount);
}

void storeCalibrationDataToEEPROM() {
    int initialMinAddr = EEPROMindex;
    int initialMaxAddr = EEPROMindex + NUM_SENSORS + 1;
    int initialInfoAddr = EEPROMindex + NUM_SENSORS * 2 + 1;
    // total eeprom addresses should be 31?
    EEPROM.put(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);

    for (int i = 0; i < NUM_SENSORS; i++) { // store eeprom min values
        int addr = initialMinAddr + i;
        EEPROM.put(addr, qtra.calibrationOn.minimum[i]);
    }

    for (int i = 0; i < NUM_SENSORS; i++) { // store eeprom max values
        int addr = initialMaxAddr + i;
        EEPROM.put(addr, qtra.calibrationOn.maximum[i]);
    }

    // calibration info store
    // 1 - calibration amount
    // 2 - calibration time
    // 3 - calibration valid

    EEPROM.put(initialInfoAddr, calibrationTime); // calibration time
    EEPROM.put(initialInfoAddr+1, calibrationAmount); // calibration amount
    EEPROM.put(initialInfoAddr+2, 1); // calibration valid

    Serial.println("Calibration data successfully stored to EEPROM.");
}

bool retrieveCalibrationDataFromEEPROM() {
    Serial.println("Looking for EEPROM calibration data...");

    // if (EEPROM.read(initialInfoAddr+2) != true) {
    //     Serial.println("No valid calibration data found in EEPROM.");
    //     return false; // No valid calibration data found
    // }

    for (int i = 0; i < NUM_SENSORS; i++) { // load eeprom min values
        int addr = EEPROM_ADDR_MIN + i;
        EEPROM.get(addr, qtra.calibrationOn. minimum[i]);
    }

    Serial.println("Calibration data loaded from EEPROM.");

}

// servo stuff

void initServos() {
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    Serial.println("Servos initialized");
}

void servoCalibration() {
    servo1.write(90); // Set servo 1 to 90 degrees
    servo2.write(90); // Set servo 2 to 90 degrees
    delay(600);
}

void servoTest() {
    servo1.write(0); // Move servo 1 to 0 degrees
    servo2.write(0); // Move servo 2 to 0 degrees
    delay(2000);
    servo1.write(180); // Move servo 1 to 180 degrees
    servo2.write(180); // Move servo 2 to 180 degrees
    delay(2000);
    servo1.write(0); // Move servo 1 back to 0 degrees
    servo2.write(0); // Move servo 2 back to 0 degrees
}

void setServo1Angle(int angle) {
    servo1.write(angle); // Set servo 1 to the specified angle
}

void setServo2Angle(int angle) {
    servo2.write(angle); // Set servo 2 to the specified angle
}

void servoAttach() {
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    Serial.println("Servos attached");
}

void servoDetach() {
    servo1.detach();
    servo2.detach();
    Serial.println("Servos detached");
}

// ultrasonic stuff

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

void setLEDColor(int red, int green, int blue) {
    analogWrite(LED_RED_PIN, red);
    analogWrite(LED_GREEN_PIN, green);
    analogWrite(LED_BLUE_PIN, blue);
}

// display / oled stuff

void initOLED(int displayNumber, int display_I2C, int textSize, int displayWidth, int displayHeight, int multiplexerPort) {
    selectMultiplexerPort(multiplexerPort);
    delay(100);

    String displayInstance = displayNumber + "display";
    Serial.println("OLED " + String(displayNumber) + " init started");

    if(!display[displayNumber].begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        return; // Exit function if initialization fails
    }

    delay(100);

    display[displayNumber].clearDisplay(); // clear the display
    display[displayNumber].setRotation(0); // set rotation
    display[displayNumber].setTextSize(textSize); // set text size
    display[displayNumber].setTextColor(SSD1306_WHITE); // set text color (won't actually change anything)
    display[displayNumber].setCursor(0, 0); // set cursor position

    Serial.println("OLED " + String(displayNumber) + " init complete");
}

void initDisplays(int display1_I2Cport, int display2_I2Cport, int textSize, int brightness, int contrast, int displayWidth, int displayHeight, int multiplexerPort1, int multiplexerPort2){
    initOLED(0, display1_I2Cport, textSize, displayWidth, displayHeight, multiplexerPort1);
    initOLED(1, display2_I2Cport, textSize, displayWidth, displayHeight, multiplexerPort2);
    displayTest();
    delay(1000);
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

void displayProgramData(){
    selectMultiplexerPort(OLED_1_MULTIPLEXER_PORT);
    display[0].clearDisplay();
    if (programDisplayAlertStatus = true) {;
        display[0].setTextSize(2);
        display[0].setCursor(22, 0);
        display[0].println("ALERT");
        display[0].display();
    }
    // left wheel render
    display[0].setTextSize(2);
    display[0].setCursor(0, 0);
    display[0].println("L");

    // display[0].drawRoundRect(x - TLH, y - TLH, width, height, roundedAmount, WHITE);
    display[0].drawRoundRect(0, 20, 30, 30, 5, WHITE);
    display[0].setTextSize(1);
    display[0].setCursor(2, 30);
    display[0].println(leftSpeed);
    display[0].display();

    // right wheel render
    display[0].setTextSize(2);
    display[0].setCursor(116, 0);
    display[0].println("R");

    display[0].drawRoundRect(98, 20, 30, 30, 5, WHITE);
    display[0].setTextSize(1);
    display[0].setCursor(100, 30);
    display[0].println(rightSpeed);

    // line array render
    display[0].setTextSize(1);
    display[0].setCursor(53, 16);
    display[0].println("pos:");
    display[0].setCursor(40, 25);
    display[0].println(position);
    display[0].setCursor(50, 40);
    display[0].println("error:");
    display[0].setCursor(40, 50);
    display[0].println(error);


    display[0].display();
}


void alertProgramDataDisplay(String alert){
    alert = programDisplayAlert;
    programDisplayAlertStatus = true;
    displayProgramData();
}

void clearAlertProgramDataDisplay(){
    programDisplayAlertStatus = false;
    displayProgramData();
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
    delay(100);
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

void loadDefaultDisplay() {
    //settings
    selectMultiplexerPort(OLED_1_MULTIPLEXER_PORT);
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
    displayCalibrationData(1);
}

//eeprom stuff

void longTermMemoryStore(int address, int value){
    // EEPROM.write(address, value); // Write the value to the specified address **BE CAREFUL max 100k writes
    Serial.println("Stored " + String(value) + " at address " + String(address)); 
}


// motor control

void setM1Speed(int speed) {
    if (speed > 0) {
        digitalWrite(M1_DIR_PIN, LOW);
    } else {
        digitalWrite(M1_DIR_PIN, HIGH);
        speed = -speed;
    }
    analogWrite(M1_PWM_PIN, constrain(speed, 0, 255));
}

void setM2Speed(int speed) {
    if (speed > 0) {
        digitalWrite(M2_DIR_PIN, LOW);
    } else {
        digitalWrite(M2_DIR_PIN, HIGH);
        speed = -speed;
    }
    analogWrite(M2_PWM_PIN, constrain(speed, 0, 255));
}

void setMotorSpeeds(int m1Speed, int m2Speed){
    setM1Speed(m1Speed);
    setM2Speed(m2Speed);
}

void motorTest() {
    Serial.println("Starting motor test...");
    setMotorSpeeds(0, 0);
    setMotorSpeeds(255, 255);
    delay(800);
    setMotorSpeeds(-60, -60);
    delay(800);
    setMotorSpeeds(0, 0);
    Serial.println("Motor test complete");
}

void updateSwitchStates() {
    calibrationSwitch.update();
    programSwitch.update();
    settingsSwitch.update();
}

void updateDisplayStatus(String status) {
    robotStatus = status;
    selectMultiplexerPort(OLED_2_MULTIPLEXER_PORT);
    displayCalibrationData(1);
}

void obstacleGoAround () {
    Serial.println("Obstacle detected and verified, going around...");
    lineFollow = false; // Disable line following temporarily
    setMotorSpeeds(0, 0); // Stop motors
    delay(500); // Wait for a moment
    setMotorSpeeds(-100, 100); // Turn around
    delay(1000); // Adjust this delay based on your robot's turning speed
    setMotorSpeeds(0, 0); // Stop again after turning
    lineFollow = true; // Re-enable line following
    Serial.println("Obstacle avoidance complete, resuming line following.");
    updateDisplayStatus("Obstacle Avoided");
}

// multiplexer stuff

void selectMultiplexerPort(int port) {
    if (port > 7) return;  // Multiplexer has 8 channels (0-7)
    if (lastMultiplexerPort == port) return;

    Wire.beginTransmission(MULTIPLEXER_I2C_ADDRESS);
    Wire.write(1 << port);  // Send the channel bitmask
    Wire.endTransmission();

    lastMultiplexerPort = port;

    Serial.println("Selected port " + String(port) + " on multiplexer");
}

void silent_selectMultiplexerPort(int port) {
    if (port > 7) return;  // Multiplexer has 8 channels (0-7)
    if (lastMultiplexerPort == port) return;
    
    Wire.beginTransmission(MULTIPLEXER_I2C_ADDRESS);
    Wire.write(1 << port);  // Send the channel bitmask
    Wire.endTransmission();
    
    lastMultiplexerPort = port;
}

void testForLostLine() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensorValues[i] < lineBrightnessMinimum) { // Adjust threshold based on sensor readings
            lineLost=true;
            setMotorSpeeds(0, 0); // Stop motors if line is lost
            updateDisplayStatus("Lost Line");
        }
        else {
            lineLost=false;
        }
    }
}

void setup() {
    Serial.begin(serialSpeed);
    Serial.println("");
    Serial.println("Power on, starting setup - wait 0.5 seconds");
    delay(600);
    Wire.begin();
    setPinModes();
    retrieveEEPROMvalues();
    setMotorSpeeds(0, 0);
    setLEDColor(255, 0, 0); //red
    delay(200);
    setLEDColor(0, 255, 0); //green
    delay(200);
    setLEDColor(0, 0, 255); //blue
    delay(200);
    setLEDColor(100, 0, 255); //thinking color

    delay(500);
    initDisplays(OLED_1_I2C_ADDRESS, OLED_2_I2C_ADDRESS, 1, brightness, contrast, OLED_DISPLAY_WIDTH, OLED_DISPLAY_HEIGHT, OLED_1_MULTIPLEXER_PORT, OLED_2_MULTIPLEXER_PORT);
    initSensorArray();
    initTOFSensors();
    buttonBounceSetup();
    motorTest();
    initServos();
    servoCalibration();
    servoTest();
    Serial.println("Setup complete");
    delay(650);
    loadDefaultDisplay();
    Serial.println("Default display loaded");
    systemReady = true;
}


void loop() {
    // calibration
    // update calibration switch
    updateSwitchStates();

    if (calibrationSwitch.fell()) { // if the calibration switch is turned off do this once
        calibrationSwitchState = false;
        setLEDColor(100, 0, 255); //thinking color
        selectMultiplexerPort(OLED_2_MULTIPLEXER_PORT);
        calibrationTime = millis() - calibrationStartTime;
        robotStatus = "UpdCalData";
        calibrationStatus = "Saving calibration data to EEPROM";
        displayCalibrationData(1);
        Serial.println("Calibration switch switched to LOW, stopping calibration...");
        Serial.println("Saving calibration data to EEPROM...");
        // storeCalibrationDataToEEPROM();
        Serial.println("Calibration data:");
        Serial.println("Updating display with calibration data...");
        robotStatus = "Idle";
        calibrationStatus = "Calibrated this session (:";
        displayCalibrationData(1);
        Serial.println("Display updated with calibration data");
        Serial.println("Calibration complete");
        setLEDColor(0, 0, 0);
    }

    if (calibrationSwitch.rose()) { // if the calibration switch is turned on do this once
        setLEDColor(100, 0, 255); //thinking color
        calibrationSwitchState = true;
        Serial.println("Calibration switch switched to HIGH, calibrating...");
        calibrationStartTime = millis();
        calibrationAmount = 0;
        robotStatus = "Calibratng";
        calibrationStatus = "Calibrating line array...";
        setLEDColor(200, 255, 10);
        displayCalibrationData(1);
    }

    if (calibrationSwitchState == true) {
        calibrateSensor();
        // displayCalibrationData(1);
        delay(2);
    }

    if (programSwitch.rose()) { // if the program switch is turned on do this once
        Serial.println("Program switch switched to HIGH, starting program...");
        updateDisplayStatus("Runng Prog");
        setLEDColor(255, 255, 0);
        programSwitchState = true;
        displayProgramData();
        obstacleAvoidanceActive = true; // enable obstacle avoidance
        lineFollow = true; // enable line following
        tempObsticleDetected = false;
    }

    if (programSwitch.fell()) { // if the program switch is turned off do this once
        holderTOFdistance = 1000; // reset holder TOF distance
        Serial.println("Program switch switched to LOW, stopping program...");
        setMotorSpeeds(0, 0);
        updateDisplayStatus("Stpng Prog");
        setLEDColor(100, 0, 255); //thinking color
        programSwitchState = false;
        loadDefaultDisplay();
        setLEDColor(255, 255, 255);
        updateDisplayStatus("Idle");
    }

    // WHILE THE PROGRAM SWITCH IS ON

    if (programSwitchState == true && lineFollow == true) { // while the program switch is on do this (loop)

        readRGBsensors();
        readTOFSensors();

    // pid line following
        uint16_t sensorValues[NUM_SENSORS]; // Array to hold sensor values
        uint16_t position = qtra.readLineBlack(sensorValues, QTRReadMode::On); // Get the position of the line
        for (uint8_t i = 0; i < sensorCount; i++) // Loop through each sensor value and print them into the serial monitor
        {
            Serial.print(sensorValues[i]);
            Serial.print('\t');
        }
        Serial.println(position);
        
        // Calculate error
        int error = position - targetPosition; // Calculate error. - how far away it is from the line

        // Calculate integral
        integral += error; //cumulative sum of the errors over time. It accounts for past errors and helps eliminate residual steady-state errors.

        // Limit the integral term to prevent windup
        integral = constrain(integral, -500, 500);

        // Calculate derivative
        int derivative = error - lastError; //is the rate of change of the error. It predicts future error based on its current rate of change, helping to dampen the system response and reduce overshoot.

        // Calculate PID output
        float pidOutput = kp * error + ki * integral + kd * derivative;

        // Calculate motor speeds
        int leftSpeed = currentLeftBaseSpeed - pidOutput;
        int rightSpeed = currentRightBaseSpeed + pidOutput;

        // Constrain motor speeds to valid range (0-255 for PWM)
        leftSpeed = constrain(leftSpeed, 0, 255);
        rightSpeed = constrain(rightSpeed, 0, 255);

        Serial.print("Left speed: ");
        Serial.println(leftSpeed);
        Serial.print("Right speed: ");
        Serial.println(rightSpeed);

        setM1Speed(leftSpeed); // Forward
        setM2Speed(rightSpeed); // Forward

        // Update last error
        lastError = error;
        Serial.print("Position: ");
        Serial.println(holderTOFdistance);
    }

    if (holderTOFdistance < slowDistance && holderTOFdistance > minTOFdistance && obstacleAvoidanceActive == true) { // if the bottle is detected, slow down
        tempObsticleDetected = true;
        updateDisplayStatus("Obst Slow");
        Serial.println("Bottle detected, slowing down...");
        currentLeftBaseSpeed = bottleBaseSpeed;
        currentRightBaseSpeed = bottleBaseSpeed;
    }

    if (holderTOFdistance <= goAroundDistance && holderTOFdistance > minTOFdistance && obstacleAvoidanceActive == true) { // go around when bottle is detected under the minimum distance
        updateDisplayStatus("Go Around");
        Serial.println("Bottle detected and is under the minimum distance, going around...");
        lineFollow = false; // stop line following
        setM1Speed(40); // Backward
        setM2Speed(40); // Forward
        delay(100); // wait for 1 second to confirm the bottle is detected

            // instructions to go around the obstacle
            setM1Speed(-255);
            setM2Speed(255);
            delay(400);
            setM1Speed(255);
            setM2Speed(255);
            delay(600);
            setM1Speed(255);
            setM2Speed(-255);
            delay(400);
            setM1Speed(255);
            setM2Speed(255);
            delay(1500);
            setM1Speed(255);
            setM2Speed(-255);
            delay(400);
            setM1Speed(255);
            setM2Speed(255);
            delay(600);
            setM1Speed(-255);
            setM2Speed(255);
            delay(400);
            setM1Speed(40); // Stop
            setM2Speed(40); // Stop

            updateDisplayStatus("GoRoundCmplte");
            Serial.println("Go around complete, wait 1 second...");
            delay(100); // wait for 1 second

            // Once go around is complete
            currentLeftBaseSpeed = leftBaseSpeed;
            currentRightBaseSpeed = rightBaseSpeed;
            lineFollow = true; // start line following again
            updateDisplayStatus("Resume Line");
            Serial.println("Resuming line following...");
    }
    
    if (!(holderTOFdistance >= slowDistance && holderTOFdistance > minTOFdistance) && obstacleAvoidanceActive == true && tempObsticleDetected == true) { // if the bottle is not detected, set the base speed back to normal
        updateDisplayStatus("Obst Clear");
        tempObsticleDetected = false;
        Serial.println("Bottle lost, setting base speed back to normal...");
        currentLeftBaseSpeed = leftBaseSpeed;
        currentRightBaseSpeed = rightBaseSpeed;
    }
}