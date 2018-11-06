#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <RF24/RF24.h>
#include <RF24Network/RF24Network.h>
#include <iostream>
#include <ctime>
#include <stdio.h>
#include <time.h>


// Setup for GPIO 22 CE and CE1 CSN with SPI Speed @ 8Mhz
RF24 radio(RPI_V2_GPIO_P1_15, BCM2835_SPI_CS0, BCM2835_SPI_SPEED_8MHZ);  

RF24Network network(radio);

// Address of our node in Octal format
const uint16_t this_node = 0;

// Address of the other node in Octal format (01,021, etc)
const uint16_t other_node = 1;

const unsigned long interval = 2000; //ms  // How often to send 'hello world to the other unit

unsigned long last_sent;             // When did we last send?
unsigned long packets_sent;          // How many have we sent already


struct payload_t {                  // Structure of our payload
  unsigned long ms;
  unsigned long counter;
};

int main(int argx, char **argv) {
  ros::init(argc, argv, "rf_comms");
  
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
