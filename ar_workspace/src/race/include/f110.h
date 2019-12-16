#pragma once

#include <race/MotorStateList.h>
#include <race/MotorCommand.h>

#include <ros/ros.h>
#include <vector>
#include <map>
#include <angles/angles.h>
#include <string>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using namespace std;

#define MAX_STEERING_ANGLE    45  //left
#define MIN_STEERING_ANGLE    -45 //right
#define DEFAULT_SPEED   0.0
#define STEERING    1
#define MOTOR   0

struct motor_data {
  string name;
  int pulse;
  double radians;
  double degrees;
};

extern motor_data motor_steering_states[2];
extern std_msgs::Float64 steering_plant_msg, steering_effort_msg, steering_setpoint_msg;

ros::NodeHandle n;
ros::Subscriber sensor_sub;
ros::Subscriber motor_sub;
ros::Publisher motor_pub;

ros::Publisher steering_plant_pub;
ros::Publisher steering_setpoint_pub;
ros::Subscriber steering_effort_sub;
ros::Publisher steering_pid_enable_pub;

ros::Publisher motor_plant_pub;
ros::Publisher motor_setpoint_pub;
ros::Subscriber motor_effort_sub;

ros::Publisher debug_pub;

ros::Subscriber imu_orientation_sub;

typedef sensor_msgs::Imu ImuMsg;

bool initialize()
double get_rate_hz();
void motor_state_callback(const MotorStateList::ConstPtr& msg);
void imu_callback(const ImuMsg::ConstPtr& imu_msg);
void steering_effort_callback(const std_msgs::Float64::ConstPtr& msg);
void publish_steering_setpoint();

int khbit();
void nonblock(int state);
bool keyState(int key); //Use ASCII table
