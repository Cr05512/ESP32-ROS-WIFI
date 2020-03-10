#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include <WiFi.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <Time.h>

// ROS nodes //
ros::NodeHandle nh;
sensor_msgs::Imu imuData;

ros::Publisher pub_imuData("/esp32/imuData", &imuData);
char base_link[] = "/base_link";
uint16_t freqPub = 25;  //Publishing frequency
uint16_t waitTime = 1000.0 / freqPub;
unsigned long t0, t1;

//WIFI Setup

const char* ssid = "yourWIFINetwork";
const char* password = "yourWIFIPassword";

IPAddress server(192,168,*,***);      // Set the rosserial socket ROSCORE SERVER IP address
const uint16_t serverPort = 11411;    // Set the rosserial socket server port
void setupWiFi();                  // connect to ROS server as as a client
#define DEBUG 1

#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 accel;         // [x, y, z]            accel sensor measurements
VectorFloat gravity;
VectorInt16 gyro;         // [x, y, z]            gyro sensor measurements
int16_t gx, gy, gz;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(21, 22, 400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);
 
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (ESP32 pin 32"));
    pinMode(32, INPUT_PULLUP);
    Serial.println(F(")..."));
    attachInterrupt(32, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  setupWiFi();
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  imuData.header.frame_id = base_link;
  nh.advertise(pub_imuData);
  t1 = millis();
  t0 = t1;
}

void loop() {
  if(nh.connected()){
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      imuData.orientation.w = q.w;
      imuData.orientation.x = q.x;
      imuData.orientation.y = q.y;
      imuData.orientation.z = q.z;

      mpu.getRotation(&gx, &gy, &gz);
      imuData.angular_velocity.x = degToRad(gx / 131.0);
      imuData.angular_velocity.y = degToRad(gy / 131.0);
      imuData.angular_velocity.z = degToRad(gz / 131.0);

      mpu.dmpGetGravity(&gravity, &q);
      imuData.linear_acceleration.x = gravity.x;
      imuData.linear_acceleration.y = gravity.y;
      imuData.linear_acceleration.z = gravity.z;
  
    }
  }
  t1 = millis();
  if(t1 - t0 > waitTime){
    imuData.header.stamp = nh.now();
    pub_imuData.publish(&imuData);
    t0 = millis();
  }
  nh.spinOnce();
}

void setupWiFi() { 
  if(DEBUG){
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
  }
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  if(DEBUG){
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
}
