#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include<geometry_msgs/Vector3.h>
#include <Servo.h>

ros::NodeHandle nh;
geometry_msgs::Vector3 rec;
std_msgs::Int16 gri;

int servo_base;
  int servo_shoulder;
  int servo_elbow;
  int servo_gripper;
  
void servomsg(const geometry_msgs::Vector3& theta){
  int servo_base= theta.x;
  int servo_shoulder= theta.y;
  int servo_elbow=theta.z;
  rec.x=theta.x;
rec.y=theta.y;
rec.z=theta.z;
}
void grippermsg(const std_msgs::Int16& gripper){
 servo_gripper= gripper.data;
 gri.data=gripper.data;
 
}

ros::Subscriber<geometry_msgs::Vector3> joint_move("/servo_joints", &servomsg);
ros::Subscriber<std_msgs::Int16> gripper_move("/servo_gripper", &grippermsg);
ros::Publisher getreadings("getreadings",&rec);
ros::Publisher gripper_reading("/gripper_read",&gri);


Servo base;
Servo shoulder;
Servo elbow;
Servo gripper;

int min_base = 120; //------
int min_shoulder = 70; //--------
int min_elbow = 140; //-------
int min_gripper = 50;

int max_base = 10; //------
int max_shoulder = 135;
int max_elbow = 100; //----------
int max_gripper = 90;

int home_base = 60; //-----
int home_shoulder = 90; //------
int home_elbow = 140; //--------


void setup() {
Serial.begin(57600);
  base.attach(9);
  shoulder.attach(8);
  elbow.attach(10);
  gripper.attach(7);

  nh.initNode();
  nh.advertise(getreadings);
  nh.advertise(gripper_reading);
  nh.subscribe(joint_move);
  nh.subscribe(gripper_move);

}

void loop() {
   shoulder.write(rec.y);
  elbow.write(rec.z);
  base.write(rec.x);
  gripper.write(gri.data);


  getreadings.publish(&rec);
  gripper_reading.publish(&gri);
  // shoulder.write(min_shoulder);
  //elbow.write(min_elbow);
  //base.write(min_base);
  //gripper.write(min_gripper);
  // delay(6000);

  //shoulder.write(home_shoulder);
  //elbow.write(home_elbow);
  //base.write(home_base);
  //gripper.write(min_gripper);

  //shoulder.write(max_shoulder);
  // elbow.write(max_elbow);
  // base.write(max_base);
  // gripper.write(max_gripper);
  // delay(3000);
nh.spinOnce();
delay(200);

}
