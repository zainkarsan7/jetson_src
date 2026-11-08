# hb_robot_hardware â€” ROS 2 Humble

Arduino Nano Every + GY-521 (MPU-6050 accelerometer and gyro) + camera protection flap servo.

## Wiring

Use a bidirectional I²C level shifter between the 5 V Nano Every and the 3.3 V sensor bus. Connect HV to Nano 5V, LV to Nano 3V3 and all grounds together.

| GY-521 pin | Connection |
|---|---|
| VCC | Nano 3V3 for a module rated for 3.3 V input, including the Joy-IT SEN-MPU6050. Check the supply rating of your breakout. |
| GND | Common ground |
| SDA | Shifter LV1, with HV1 to Nano A4/SDA |
| SCL | Shifter LV2, with HV2 to Nano A5/SCL |
| AD0 | Ground for address 0x68, or leave open if the board already pulls it low |
| XDA, XCL, INT | Leave unconnected. The firmware polls data-ready over I²C. |

[Joy-IT module manual](https://joy-it.net/files/files/Produkte/SEN-MPU6050/SEN-MPU6050_Manual_2024-04-15.pdf)

- Servo signal -> D9; servo power -> separate appropriately rated supply; connect grounds. Do not power a load-bearing servo from the Nano 5 V rail.
- Mechanically check OPEN=0Â°, CLOSED=90Â° before installing. Camera flap is not a safety-rated protective device.

## Install

1. In Arduino IDE select **Arduino Nano Every**, install Arduino megaAVR boards and the bundled Servo library. Upload `firmware/nano_every_bridge/nano_every_bridge.ino`.
2. Place `hb_robot_hardware` under `~/ros2_ws/src/`.
3. `sudo apt install python3-serial` (or install `pyserial` for the Python environment running ROS).
4. `cd ~/ros2_ws && colcon build --symlink-install --packages-select hb_robot_hardware && source install/setup.bash`
5. Identify the stable port: `ls -l /dev/serial/by-id/`.
6. `ros2 launch hb_robot_hardware hardware.launch.py port:=/dev/serial/by-id/YOUR_NANO`

## Topics and service

- `/mpu6050/imu` (`sensor_msgs/Imu`): acceleration in m/sÂ² in raw sensor axes; gravity included. Angular velocity is published in rad/s. Orientation is unavailable. No TF is published; add a measured static transform in hb_robot_description.
- `/mpu6050/sample_rate_hz` (`std_msgs/Float32`): exponentially averaged rate based on MCU micros(), not a guarantee of sample freshness.
- `/mpu6050/sequence` (`std_msgs/UInt32`): sequence number for gap detection.
- `/hardware/online` (`std_msgs/Bool`): recent samples received.
- `/camera_flap/closed` (`std_msgs/Bool`): **command acknowledged**, not physical feedback.
- `/camera_flap/set_closed` (`std_srvs/SetBool`): true = CLOSED; false = OPEN.

Examples:

```bash
ros2 topic hz /mpu6050/imu
ros2 topic echo /mpu6050/sample_rate_hz
ros2 service call /camera_flap/set_closed std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera_flap/set_closed std_srvs/srv/SetBool '{data: false}'
```

The cut profile server calls the flap service for each torch-enabled cut. After a close acknowledgement it waits `flap_travel_ms` (default 500 ms) before requesting ignition. Set this startup parameter to the measured loaded servo travel time with margin. After confirmed shutdown and return to idle it commands open and waits for travel again. An unconfirmed shutdown leaves the flap commanded closed. Motion dry runs do not command the flap.

## Serial protocol

ASCII newline-terminated `$payload*XX` where XX is uppercase 2-digit XOR checksum over payload bytes.

- MCU -> host: `M,sequence,micros,ax,ay,az,gx,gy,gz`.
- Host -> MCU: `FLAP,command_id,angle_deg`.
- MCU -> host: `ACK,command_id,angle_deg`.

The configured ranges are ±2 g (16384 counts/g) and ±250 °/s (131 counts per °/s). Flash the sketch and update the bridge together. Device `micros` wraps every ~71.6 minutes; wrap is handled for sample-rate calculation. ROS header uses host receipt time, **not synchronized measurement time**. Serial disconnect triggers automatic reconnection. No servo position sensor is provided. Servo is commanded open on MCU startup, so do not rely on flap position across resets.

## Engineering notes

- The firmware targets 200 Hz with the on-chip low-pass filter enabled. Log `/mpu6050/sequence` and compare observed `ros2 topic hz` with `/mpu6050/sample_rate_hz`.
- The skills launch uses `platform_accel_topic` (default `/mpu6050/imu`) alongside the platform lidar IMU. Both streams must have fresh, quiet windows. The stability monitor uses acceleration from both sensors and angular velocity from the lidar. The MPU gyro is also available on the raw IMU topic. Camera-only settling checks continue to use the Kinect IMU.
- The sensor is not calibrated by this package: run a six-orientation calibration and estimate stationary variance before trusting thresholds.
- For plasma operation, add physical limit switches or position sensing for flap confirmation, hardware interlocks, shielded cabling, electrical isolation and watchdog/fail-safe behavior. The provided servo service is a prototype actuator interface, not a verified protection interlock.
- Sample timing is host-receipt based. For accurate MPU6050/UniLiDAR phase comparisons, synchronize clocks or estimate latency; a future protocol revision can expose MCU timestamps with a host clock-offset estimator.
