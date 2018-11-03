#include <ros/ros.h>
#include <std_msgs/Int16.h>

#include <wiringPi.h>
#include <softPwm.h>

// Pin number declarations. We're using the Broadcom chip pin numbers.
const int pwmPin = 18; // PWM LED - Broadcom pin 18, P1 pin 12

void motorControlCallback(const std_msgs::Int16::ConstPtr & msg) {
    //ROS_INFO("I heard: [%c]", msg->data);
    unsigned char rightMotor = msg->data & 0x00FF;
    unsigned char leftMotor = msg->data >> 8;
	ROS_INFO("%d", rightMotor & 0x40);
    if(rightMotor & 0x40 == 0){
        softPwmWrite(26, rightMotor & 0x3F);
        softPwmWrite(27, 0);
		std::cout << "Should not be here" <<std::endl;
    } else {
        softPwmWrite(26, 0);
        softPwmWrite(27, rightMotor & 0x3F);
    }

    if(leftMotor & 0x40 == 0){
        softPwmWrite(28, leftMotor & 0x3F);
        softPwmWrite(29, 0);
    } else {
        softPwmWrite(28, 0);
        softPwmWrite(29, leftMotor & 0x3F);
    }
}

int main(int argc, char **argv) {
    // Setup ROS node listener
    ros::init(argc, argv, "motor_control");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("motor_commands", 1000, motorControlCallback);
    // Setup stuff:
    wiringPiSetup(); // Initialize wiringPi -- using Broadcom pin numbers

    softPwmCreate(26, 0, 64);
    softPwmCreate(27, 0, 64);
    softPwmCreate(28, 0, 64);
    softPwmCreate(29, 0, 64);

    ROS_INFO("Loop is running");
    ros::spin();

    // Loop (while(1)):
    int motorValue = 50;
    while(1)
    {
        motorValue += 10;
        if(motorValue > 80){
            motorValue = 30;
        }
        std::cout << motorValue << std::endl;
        softPwmWrite(28, motorValue);
        softPwmWrite(29, 0);
        delay(2000); // Wait 75ms
        softPwmWrite(29, motorValue);
        softPwmWrite(28, 0);
        delay(2000);
    }
    return 0;
}
