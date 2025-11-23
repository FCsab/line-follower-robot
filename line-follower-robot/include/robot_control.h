#pragma once

#include <Arduino.h>

// Pin assignments
inline constexpr uint8_t BUTTON_PIN = 13;
inline constexpr uint8_t PWMB_PIN = 14;
inline constexpr uint8_t BIN2_PIN = 27;
inline constexpr uint8_t BIN1_PIN = 26;
inline constexpr uint8_t STBY_PIN = 25;
inline constexpr uint8_t AIN1_PIN = 33;
inline constexpr uint8_t AIN2_PIN = 32;
inline constexpr uint8_t PWMA_PIN = 23;
inline constexpr uint8_t SENSOR_COUNT = 5;
inline constexpr uint8_t SENSOR_PINS[SENSOR_COUNT] = {4, 16, 17, 5, 18};
inline constexpr int SENSOR_MIN_VALUE = 0;
inline constexpr int SENSOR_MAX_VALUE = 4095;
inline constexpr uint8_t BUTTON_SAMPLE_COUNT = 5;
inline constexpr unsigned int BUTTON_SAMPLE_DELAY_US = 500;

struct SensorReadings {
    int values[SENSOR_COUNT];
};

extern uint8_t BASE_SPEED;
extern uint8_t TURN_SPEED;
extern uint8_t CORRECTION_SPEED;
extern uint8_t REVERSE_SPEED;

void initRobot();
void moveForward(uint8_t speed = 255);
void moveBackward(uint8_t speed = 255);
void turnLeft(uint8_t speed = 255);
void turnRight(uint8_t speed = 255);
void stopMoving();
void driveMotors(int leftSpeed, int rightSpeed);
SensorReadings readAllSensors();
int readSensor(uint8_t index);
bool sensorValueValid(int value);
bool sensorsHealthy(const SensorReadings &readings);
bool isButtonPressed();
void emergencyStop();
