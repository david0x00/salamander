#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <RF24/RF24.h>
#include <RF24Network/RF24Network.h>
#include <RF24Mesh/RF24Mesh.h>
#include <iostream>
#include <ctime>
#include <stdio.h>
#include <time.h>
#include <fstream>
#include <string>

// Setup for GPIO 22 CE and CE1 CSN with SPI Speed @ 8Mhz
RF24 radio(RPI_V2_GPIO_P1_15, BCM2835_SPI_CS0, BCM2835_SPI_SPEED_8MHZ);  

RF24Network network(radio);
RF24Mesh mesh(radio,network);

bool amMaster = false;

int main(int argc, char **argv) {
	// get node ID from file
	std::ifstream nodeIDFile;
	std::string line;
	int nodeId = 0;
	nodeIDFile.open("/home/pi/.salamanderNodeId");
	if(nodeIDFile.is_open()){
		std::getline(nodeIDFile, line);
		nodeId = std::stoi(line);
		nodeIDFile.close();
	}else{
		std::cout << "Make sure to run setNodeId.sh" << std::endl;
		return 1;
	}
	amMaster = (nodeId == 0);
	// Setup RF24Mesh
	mesh.setNodeID(nodeId);
	mesh.begin();
	radio.printDetails();

	ros::init(argc, argv, "rf_comms");

	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<std_msgs::Int64>("rf_received", 1000);

	ros::Rate loop_rate(200);

	while (ros::ok())
	{
		mesh.update();
		if(amMaster){
			mesh.DHCP();
		}


		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}
