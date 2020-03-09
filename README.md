# ESP32-ROS-WIFI

This app is meant to get readings from the MPU6050 sensor and to stream them over WIFI to a remote ROS server.

Wiring:

MPU6050  <------>  ESP32 Wroom
VCC      <------>  3.3V
GND      <------>  GND
SDA      <------>  G22
SCL      <------>  G21
INT      <------>  G32

By setting the sampling frequency it is possible to regulate the rate of data publishing.
The app itself is just a benchmark, it may contain errors.
