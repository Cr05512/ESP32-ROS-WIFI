#include <SparkFunMPU9250-DMP.h>
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

const char* ssid = "yourSSID";
const char* password = "yourPass";

IPAddress server(192,168,*,***);      // Set the rosserial socket ROSCORE SERVER IP address
const uint16_t serverPort = 11411;    // Set the rosserial socket server port
void setupWiFi();                  // connect to ROS server as as a client
#define DEBUG 1

#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

MPU9250_DMP mpu;

void setup() {
  Serial.begin(115200);

  if (mpu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(5000);
    }
  }


  mpu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
               50); // Set DMP FIFO rate to 50 Hz

  
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
    
    if ( mpu.fifoAvailable() ){
      
      if ( mpu.dmpUpdateFifo() == INV_SUCCESS){
        imuData.orientation.w = mpu.calcQuat(mpu.qw);
        imuData.orientation.x = mpu.calcQuat(mpu.qx);
        imuData.orientation.y = mpu.calcQuat(mpu.qy);
        imuData.orientation.z = mpu.calcQuat(mpu.qz);

        mpu.update(UPDATE_ACCEL | UPDATE_GYRO);
        imuData.angular_velocity.x = degToRad(mpu.calcGyro(mpu.gx) / mpu.getGyroSens());     //The gyro full scale range is set to 0, that is we need to divide by the LSB sensitivity of 131.0
        imuData.angular_velocity.y = degToRad(mpu.calcGyro(mpu.gy) / mpu.getGyroSens());
        imuData.angular_velocity.z = degToRad(mpu.calcGyro(mpu.gz) / mpu.getGyroSens());

        imuData.linear_acceleration.x = mpu.calcAccel(mpu.ax) / mpu.getAccelSens();    // The accel range is set to 0 (2g); that means the dividing factor will be twice the LSB sensitivity, that is 2*8192 = 16384, if we want
        imuData.linear_acceleration.y = mpu.calcAccel(mpu.ay) / mpu.getAccelSens();    // to map the accelerations between -1g and +1g
        imuData.linear_acceleration.z = mpu.calcAccel(mpu.az) / mpu.getAccelSens();
      }
 
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
