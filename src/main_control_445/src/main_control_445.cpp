#include <ros/ros.h>

#include <std_msgs/Int16.h>

#include <iostream>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::Int16>("motor_commands", 1000);

    ros::Rate loop_rate(1);

    int count = 0;
    unsigned char motorValue = 32;
    while (ros::ok())
    {
        if(motorValue == 55){
            motorValue = 32;
        }
        std_msgs::Int16 msg;
        unsigned int motorCmd = 0;
        motorCmd |= motorValue;
        motorCmd <<= 8;
        motorCmd |= 0xC0;
        motorCmd |= motorValue;

        msg.data = motorCmd;

        //ROS_INFO("%d", msg.data);
		std::cout << std::hex << msg.data << std::endl;

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
        motorValue++;
    }
    return 0;
}
