# ESP32-ROS-WIFI

This app is meant to get readings from the MPU6050 sensor and to stream them over WIFI to a remote ROS server.

Wiring:

| MPU6050 | ESP32 Wroom |
| ------- | ----------- |
| VCC | 3.3V |
| GND | GND |
| SDA | G22 |
| SCL | G21 |
| INT | G32 |

By setting the sampling frequency it is possible to regulate the rate of data publishing.
The app itself is just a benchmark, it may contain errors.

**INSTALLATION INSTRUCTIONS**

1. Clone this repo  
2. Copy the folders ros_lib, mpu6050 and I2Cdev in Arduino/libraries  
3. Open the main.ino file and flash it to the ESP32

**USAGE**
1. Open a terminal and launch ``roscore``  
2. Open another terminal window and launch ```rosrun rosserial_python serial_node.py tcp``  
3. You can now see the logged data through ``rostopic echo /esp32/imuData``
