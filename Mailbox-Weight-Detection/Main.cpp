#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include "HX711.h"

#define DT 3
#define SCK 4

HX711 scale;
ros::NodeHandle nh;

std_msgs::Float32 weight_msg;
std_msgs::String status_msg;

ros::Publisher weight_pub("mailbox_weight", &weight_msg);
ros::Publisher status_pub("mailbox_status", &status_msg);

float prevWeight = 0;
float threshold = 1.0; // grams

char status_buffer[20];

void setup() {
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(weight_pub);
  nh.advertise(status_pub);

  scale.begin(DT, SCK); 
  scale.set_scale(-7050); 
  scale.tare();
}

void loop() {
  // Check if connected to roscore
  if (!nh.connected()) {
    // Optional: Blink LED or hold until reconnected
    delay(100);
    nh.spinOnce();
    return; // Skip running until roscore is back
  }

  float weight = scale.get_units();

  weight_msg.data = weight;
  weight_pub.publish(&weight_msg);

  // detect change > threshold
  if (abs(weight - prevWeight) >= threshold) {
    sprintf(status_buffer, "Mail detected");
    status_msg.data = status_buffer;
    status_pub.publish(&status_msg);

    prevWeight = weight;
  }

  nh.spinOnce();
  delay(500);
}