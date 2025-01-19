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

byte SENSOR_PIN_ARRAY[SENSOR_ARRAY_SIZE] = 
    {SENSOR_ARRAY_1, SENSOR_ARRAY_2, SENSOR_ARRAY_3, SENSOR_ARRAY_4, SENSOR_ARRAY_5, SENSOR_ARRAY_6, SENSOR_ARRAY_7, SENSOR_ARRAY_8, SENSOR_ARRAY_9, SENSOR_ARRAY_10, SENSOR_ARRAY_11, SENSOR_ARRAY_12, SENSOR_ARRAY_13};

//i2c addresses
#define SDA_PIN 20
#define SCL_PIN 21
#define OLED_1_I2C_ADDRESS 0x04
#define OLED_2_I2C_ADDRESS 0x05
#define RGB_I2C_ADDRESS 0x05
#define TOF_I2C_ADDRESS 0x06
#define MULTIPLEXER_I2C_ADDRESS 0x07
#define _I2C_ADDRESS 0x08

//settings
float speed = 1;
float kp = 0.5;
float ki = 0.5;
float kd = 0.5;
float samplesPerRead = 4;
float recommendedCalibrationTime = 0.5;
//rgb colors 
byte greenColor[3] = {0, 255, 0};
byte whiteColor[3] = {255, 0, 0};
byte blackColor[3] = {0, 0, 255};


//variables
bool can_detected = false;
float can_distance = 0;

//functions

void setPinModes(){
    for (int i = 0; i < SENSOR_ARRAY_SIZE; i++){
        pinMode(SENSOR_PIN_ARRAY[i], INPUT);
    }
}

void initI2C(){
    Wire.begin();
}

void initOLED(display1_I2C, display2_I2C, brightness, contrast){
    Wire.beginTransmission(OLED_I2C_ADDRESS);
    Wire.write(0x00);
    Wire.endTransmission();
    //calibration display stuff

}


void setup() {
    Serial.begin(9600);
    initI2C();
    setPinModes();
    initOLED();
     
}

void calibrateSensor(){
    qtra.calibrate();
}

void calibrateSensorsWhile(switchPin, calibrationDisplayPin){
    calibrationTimer = millis();
    calibrations = 0;
    while(digitalRead(switchPin) == HIGH){
        calibrateSensor();
        calibrations++;
        delay(10);
    }
}

void loop() {

}