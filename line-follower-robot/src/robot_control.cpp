#include "robot_control.h"

uint8_t BASE_SPEED = 160;
uint8_t TURN_SPEED = 200;
uint8_t CORRECTION_SPEED = 20;
uint8_t REVERSE_SPEED = 140;

namespace {
constexpr uint8_t PWM_CHANNEL_A = 0;
constexpr uint8_t PWM_CHANNEL_B = 1;
constexpr uint32_t PWM_FREQUENCY = 15000;
constexpr uint8_t PWM_RESOLUTION = 8;
bool driverActive = false;

void enableDriver(bool enable) {
    digitalWrite(STBY_PIN, enable ? HIGH : LOW);
    driverActive = enable;
}

void driveMotor(uint8_t pwmChannel, uint8_t in1, uint8_t in2, int speed) {
    speed = constrain(speed, -255, 255);
    if (speed >= 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        ledcWrite(pwmChannel, speed);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        ledcWrite(pwmChannel, -speed);
    }
}

void configureMotorPins() {
    pinMode(AIN1_PIN, OUTPUT);
    pinMode(AIN2_PIN, OUTPUT);
    pinMode(BIN1_PIN, OUTPUT);
    pinMode(BIN2_PIN, OUTPUT);
    pinMode(STBY_PIN, OUTPUT);

    ledcSetup(PWM_CHANNEL_A, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(PWMA_PIN, PWM_CHANNEL_A);
    ledcSetup(PWM_CHANNEL_B, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(PWMB_PIN, PWM_CHANNEL_B);
}

void configureSensors() {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        pinMode(SENSOR_PINS[i], INPUT);
    }
}
}

void initRobot() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    configureMotorPins();
    configureSensors();
    enableDriver(false);
    stopMoving();
}

void moveForward(uint8_t speed) {
    driveMotors(speed, speed);
}

void moveBackward(uint8_t speed) {
    driveMotors(-speed, -speed);
}

void turnLeft(uint8_t speed) {
    driveMotors(speed / 2, speed);
}

void turnRight(uint8_t speed) {
    driveMotors(speed, speed / 2);
}

void stopMoving() {
    driveMotors(0, 0);
}

void driveMotors(int leftSpeed, int rightSpeed) {
    if (leftSpeed == 0 && rightSpeed == 0) {
        driveMotor(PWM_CHANNEL_A, AIN1_PIN, AIN2_PIN, 0);
        driveMotor(PWM_CHANNEL_B, BIN1_PIN, BIN2_PIN, 0);
        enableDriver(false);
        return;
    }
    if (!driverActive) {
        enableDriver(true);
    }
    driveMotor(PWM_CHANNEL_A, AIN1_PIN, AIN2_PIN, leftSpeed);
    driveMotor(PWM_CHANNEL_B, BIN1_PIN, BIN2_PIN, rightSpeed);
}

SensorReadings readAllSensors() {
    SensorReadings readings{};
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        readings.values[i] = analogRead(SENSOR_PINS[i]);
    }
    return readings;
}

int readSensor(uint8_t index) {
    if (index >= SENSOR_COUNT) {
        return -1;
    }
    int value = analogRead(SENSOR_PINS[index]);
    return sensorValueValid(value) ? value : -1;
}

bool sensorValueValid(int value) {
    return value >= SENSOR_MIN_VALUE && value <= SENSOR_MAX_VALUE;
}

bool sensorsHealthy(const SensorReadings &readings) {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        if (!sensorValueValid(readings.values[i])) {
            return false;
        }
    }
    return true;
}

bool isButtonPressed() {
    uint8_t positiveSamples = 0;
    for (uint8_t i = 0; i < BUTTON_SAMPLE_COUNT; ++i) {
        if (digitalRead(BUTTON_PIN) == LOW) {
            ++positiveSamples;
        }
        delayMicroseconds(BUTTON_SAMPLE_DELAY_US);
    }
    return positiveSamples > BUTTON_SAMPLE_COUNT / 2;
}

void emergencyStop() {
    driveMotors(0, 0);
}
