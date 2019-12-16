#include <race/f110.h>
#include <termios.h>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "race");
    if(initialize()) {
        ROS_INFO("Failed to initialize race node");
        return EXIT_FAILURE;
    }else {
        ROS_INFO("Successfully initialized race node");
    }
    ros::Rate rate(controller.get_rate_hz());

    ros::NodeHandle np("~");
    double speed_test;
    np.getParam("speed_test", speed_test);

    //pololu
    motor_sub = n.subscribe("pololu/motor_states", 1, motor_state_callback, this);
    motor_pub = n.advertise<MotorCommand>("pololu/command", 1);

    //pid
    steering_plant_pub = n.advertise<std_msgs::Float64>("steering_plant", 1);
    steering_setpoint_pub = n.advertise<std_msgs::Float64>("steering_setpoint", 1);
    steering_effort_sub = n.subscribe("steering_effort", 1, steering_effort_callback, this);
    steering_pid_enable_pub = n.advertise<std_msgs::Bool>("steering_pid_enable", 1);

    //imu
    imu_orientation_sub = n.subscribe("imu/data", 1, imu_callback, this);

    //debugging broadcast
    debug_pub = n.advertise<std_msgs::String>("race/debug", 1);

    //publish initial speed
    MotorCommand motor_command;
    motor_command.joint_name = "drive";
    motor_command.position = 0; //speed
    motor_command.speed = 5; //accel
    motor_command.acceleration = 0; //jerk
    motor_pub.publish(motor_command);

    while(ros::ok()) {
	      nonblock(1);
        if (keyState(32)) { //32 in ASCII table correspond to Space Bar
            nonblock(0);
            break;
        }

        //steering_plant from motor_steering_states[1]
        steering_plant_msg.data = steering_plant;
        if(!isnan(steering_plant_msg.data)){
      	   steering_plant_pub.publish(steering_plant_msg);
        }

        //steering_setpoint_pub.data  from follow the gap

        ros::spinOnce();
        rate.sleep();
	  }

	  ROS_INFO("Exited race");
	  ros::shutdown();

    return 0;
}

bool initialize() {
    return true;
}

double get_rate_hz() {
    //get this from params
    double rate_hz = 70.0;
    return rate_hz;
}

void steering_effort_callback(const std_msgs::Float64::ConstPtr& msg) {
    MotorCommand motor_command;

    steering_effort_msg.data = deg_to_rad(msg->data);
    motor_command.joint_name = "steering";
    motor_command.position = steering_effort_msg.data;
    publish_motor_command(motor_command);
}

void publish_motor_command(MotorCommand motor_command) {
    motor_pub.publish(motor_command);
}

void publish_steering_setpoint() {
    steering_setpoint_pub.publish(steering_setpoint_msg);
}

void motor_state_callback(const MotorStateList::ConstPtr& msg) {
    motor_steering_states[MOTOR].name = msg->motor_states[MOTOR].name;
    motor_steering_states[MOTOR].pulse = msg->motor_states[MOTOR].pulse;
    motor_steering_states[MOTOR].radians = msg->motor_states[MOTOR].radians;
    motor_steering_states[MOTOR].degrees = msg->motor_states[MOTOR].degrees;
    //ROS_INFO("motor_state:%d",latest_motor_state[MOTOR].pulse);

    motor_steering_states[STEERING].name = msg->motor_states[STEERING].name;
    motor_steering_states[STEERING].pulse = msg->motor_states[STEERING].pulse;
    motor_steering_states[STEERING].radians = msg->motor_states[STEERING].radians;
    motor_steering_states[STEERING].degrees = msg->motor_states[STEERING].degrees;
    //ROS_INFO("Steering state callback radians:%lf",latest_motor_state[STEERING].radians);
}

void imu_callback(const ImuMsg::ConstPtr& imu_msg) {
    double q0,q1,q2,q3;
    double t;
    double t_interval;

    q0 = imu_msg->orientation.w;
    q1 = imu_msg->orientation.x;
    q2 = imu_msg->orientation.y;
    q3 = imu_msg->orientation.z;
    x_accel = imu_msg->linear_acceleration.x;
    y_accel = imu_msg->linear_acceleration.y;

    tf2::Matrix3x3(tf2::Quaternion(q1,q2,q3,q0)).getRPY(roll, pitch, yaw);
    if(yaw < 0){
      yaw = -yaw;
    }
    else{
      yaw = M_PI + abs(M_PI-yaw);
    }
    if(!yaw_found){
      yaw_zero = yaw;
      yaw_found = true;
    }
    //ROS_INFO("yaw: %lf", yaw);
    //ROS_INFO("forward/reverse accel: %lf", x_accel);
}

double deg_to_rad(double angle){
    double angle_rad = angle * M_PI / 180;
    return angle_rad;
}
