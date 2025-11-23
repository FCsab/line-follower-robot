# Line Follower Robot

Firmware scaffold for an ESP32-based line follower that drives a TB6612FNG dual H-bridge and reads a 5-way TCRT5000 reflectance array. The code exposes movement helpers, sensor utilities, and a failsafe-heavy line-follow routine that you can further tune for your chassis.

## Hardware

- ESP32 development board (tested with GENERIC_ESP32 in PlatformIO)
- TB6612FNG dual motor driver
- TCRT5000 five-channel line sensor
- Two DC gear motors + chassis + battery pack
- Momentary push button (run/stop)

### Pin Mapping

| Function        | ESP32 GPIO |
|-----------------|------------|
| Button          | 13         |
| PWMB / Motor B  | 14         |
| BIN2            | 27         |
| BIN1            | 26         |
| STBY            | 25         |
| AIN1            | 33         |
| AIN2            | 32         |
| PWMA / Motor A  | 23         |
| Sensor OUT1     | 4          |
| Sensor OUT2     | 16         |
| Sensor OUT3     | 17         |
| Sensor OUT4     | 5          |
| Sensor OUT5     | 18         |

## Firmware Features

- Central header (`include/robot_control.h`) describing every pin, shared speed variables, and helper APIs.
- Motor helpers (`moveForward`, `turnLeft`, `driveMotors`, etc.) built on ESP32 LEDC PWM channels.
- Sensor utilities to read all five reflectance channels or a single index.
- Line-follow loop that weights each sensor, steers proportionally, and performs search maneuvers when the line is lost.
- Failsafes for button debounce, sensor faults, and line-loss timeouts. `emergencyStop()` is called whenever inputs look unsafe.

## Building and Uploading

1. Install [PlatformIO](https://platformio.org/) and open this workspace (`line-follower-robot/line-follower-robot`).
2. Confirm the correct ESP32 environment inside `platformio.ini` (default: `board = esp32dev`).
3. Connect the ESP32 over USB.
4. Build: `pio run`
5. Upload: `pio run -t upload`
6. Monitor serial output if you add logging: `pio device monitor`

## Customization Tips

- Adjust `BASE_SPEED`, `TURN_SPEED`, `CORRECTION_SPEED`, and `REVERSE_SPEED` in `robot_control.cpp` to fit your motors.
- Tweak `SENSOR_THRESHOLD` in `src/main.cpp` once you measure raw values over the line vs. background.
- The failsafe constants (`LINE_LOSS_TIMEOUT_MS`, etc.) can be tuned per track difficulty.
- Use the provided helpers to build more advanced behaviors (PID, lap timing, button toggles) without touching lower-level motor code.
