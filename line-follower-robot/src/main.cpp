#include <Arduino.h>
#include "robot_control.h"

namespace {
constexpr int SENSOR_THRESHOLD = 2000;
constexpr int SENSOR_WEIGHTS[SENSOR_COUNT] = {-2, -1, 0, 1, 2};
constexpr unsigned long LINE_LOSS_TIMEOUT_MS = 1500;
constexpr unsigned long SEARCH_REVERSE_MS = 30;
constexpr unsigned long SEARCH_SPIN_MS = 80;
constexpr uint8_t SENSOR_FAULT_LIMIT = 3;
constexpr uint8_t MAX_ERROR_MAGNITUDE = 2;
unsigned long lastLineSeen = 0;
int lastError = 0;
uint8_t sensorFaultCounter = 0;

int computeLineError(const SensorReadings &readings, bool &detected) {
    int weighted = 0;
    int hits = 0;
    detected = false;
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        if (!sensorValueValid(readings.values[i])) {
            continue;
        }
        bool onLine = readings.values[i] < SENSOR_THRESHOLD;
        if (onLine) {
            weighted += SENSOR_WEIGHTS[i];
            ++hits;
            detected = true;
        }
    }
    if (!detected || hits == 0) {
        return lastError;
    }
    return weighted / hits;
}

bool handleSensorFaults(const SensorReadings &readings) {
    if (!sensorsHealthy(readings)) {
        if (++sensorFaultCounter >= SENSOR_FAULT_LIMIT) {
            emergencyStop();
            return false;
        }
    } else {
        sensorFaultCounter = 0;
    }
    return true;
}

void followLineStep() {
    SensorReadings readings = readAllSensors();
    if (!handleSensorFaults(readings)) {
        return;
    }

    bool detected = false;
    int error = computeLineError(readings, detected);
    unsigned long now = millis();
    if (detected) {
        lastLineSeen = now;
    }

    if (!detected) {
        if (now - lastLineSeen > LINE_LOSS_TIMEOUT_MS) {
            emergencyStop();
            return;
        }
        driveMotors(-REVERSE_SPEED, -REVERSE_SPEED);
        delay(SEARCH_REVERSE_MS);
        int direction = lastError >= 0 ? 1 : -1;
        driveMotors(-direction * TURN_SPEED, direction * TURN_SPEED);
        delay(SEARCH_SPIN_MS);
        return;
    }

    lastError = error;
    int base = (abs(error) >= MAX_ERROR_MAGNITUDE) ? TURN_SPEED : BASE_SPEED;
    int correction = error * CORRECTION_SPEED;
    int left = constrain(base - correction, -255, 255);
    int right = constrain(base + correction, -255, 255);
    driveMotors(left, right);
}
}

void setup() {
    initRobot();
    lastLineSeen = millis();
}

void loop() {
    if (!isButtonPressed()) {
        emergencyStop();
        lastLineSeen = millis();
        lastError = 0;
        sensorFaultCounter = 0;
        delay(10);
        return;
    }
    followLineStep();
    delay(10);
}