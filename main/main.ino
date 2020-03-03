#include <WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <ros/time.h>

#define DEBUG 1

// ROS nodes //
ros::NodeHandle nh;
std_msgs::String str_msg;
std_msgs::Int16 int_msg;

ros::Publisher pub_string("/esp32/chatter", &str_msg);

const char* ssid = "WebCube4-4NN5";
const char* password = "ELG67HAH";
IPAddress server(192,168,1,107);      // Set the rosserial socket ROSCORE SERVER IP address
const uint16_t serverPort = 11411;    // Set the rosserial socket server port

void setupWiFi();                  // connect to ROS server as as a client

void setup() {
  if(DEBUG) Serial.begin(115200);
  setupWiFi();
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.advertise(pub_string);
  str_msg.data = "Test";
}

ros::Time current_time = nh.now();
ros::Time last_time = current_time;

void loop() {
  if (nh.connected()) {
    pub_string.publish(&str_msg);
  
  }
  nh.spinOnce();
  // Loop aprox. every  
  delay(200);  // milliseconds

}

void setupWiFi() {                    // connect to ROS server as as a client
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
