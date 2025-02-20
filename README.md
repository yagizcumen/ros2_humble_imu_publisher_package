# IMU Publisher ROS2 Package

This package reads IMU data from a serial port, processes it, and publishes it as a ROS2 topic.

## Features
- Reads IMU data from a serial device (e.g., Arduino, MPU6050)
- Uses `imufusion` for sensor fusion
- Publishes IMU data as a ROS2 `sensor_msgs/Imu` message
- Supports gyro calibration

## Dependencies
Ensure you have the following installed:
- ROS2 Humble (or compatible version)
- `numpy`
- `imufusion`
- `ahrs`
- `pyserial`

You can install dependencies using:
```bash
pip install numpy imufusion ahrs pyserial
```

## Installation
Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository-url>
cd ~/ros2_ws
colcon build --symlink-install
source ~/.bashrc
```

## Usage
To run the IMU publisher:
```bash
ros2 run imu_publisher imu_publisher
```

## Topic
The IMU data is published to the following topic:
```
/imu/data_raw (sensor_msgs/Imu)
```

## Configuration
Modify the `port` variable in `imu_publisher.py` to match your serial device:
```python
port = "/dev/ttyUSB0"
```
## Credit
- The Arduino firmware and serial reading with calibration sequence code used in this project was provided by @Albaryan.

## Author
[Yagiz Cumen]
