#include <race/f110.h>

int main(int argc, char **argv)
{
    ros::Rate rate(controller.get_rate_hz());

    motor_pub = n.advertise<MotorCommand>("pololu/command", 1);

    //publish initial speed
    MotorCommand motor_command;
    motor_command.joint_name = "drive";
    motor_command.position = DEFAULT_SPEED; //speed
    motor_command.speed = 5; //accel
    motor_command.acceleration = 0; //jerk

    while(ros::ok()) {
        nonblock(1);
        if (keyState(13)) { //32 in ASCII table correspond to return/enter
            ROS_INFO("Speed: %d", motor_command.position);
            nonblock(0);
            break;
        }
        if (keyState(45)) { //45 in ASCII table correspond to -
            motor_pub.postion = motor_pub.position - .05;
            nonblock(0);
        }
        if (keyState(61)) { //61 in ASCII table correspond to =
            motor_pub.postion = motor_pub.position + .05;
            nonblock(0);
        }
        motor_pub.publish(motor_command);

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Exited race calibration");
    ros::shutdown();

    ros::init(argc, argv, "race");

    return 0;
}
