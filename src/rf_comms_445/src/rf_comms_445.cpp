// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rf_comms_445/RFPayload.h>

// RF24
#include <RF24/RF24.h>
#include <RF24Network/RF24Network.h>
#include <RF24Mesh/RF24Mesh.h>

// STL
#include <iostream>
#include <ctime>
#include <stdio.h>
#include <time.h>
#include <fstream>
#include <string>

struct payload {
	char messageType;
	unsigned int seqNum;
	float x;
	float y;
	float orientation;
	bool taskDone;
};


// Setup for GPIO 22 CE and CE1 CSN with SPI Speed @ 8Mhz
RF24 radio(RPI_V2_GPIO_P1_15, BCM2835_SPI_CS0, BCM2835_SPI_SPEED_8MHZ);  

RF24Network network(radio);
RF24Mesh mesh(radio,network);

bool amMaster = false;
std::string rfMessageToSend;

void rfSendCallback(const rf_comms_445::RFPayload::ConstPtr & msg) {
	//rfMessageToSend = msg->data;
}

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

	ros::Publisher rfPub = n.advertise<rf_comms_445::RFPayload>("rf_received", 1000);
	ros::Subscriber rfSub = n.subscribe("rf_to_send", 1000, rfSendCallback);

	ros::Rate loop_rate(200);

	while (ros::ok())
	{
		mesh.update();
		if(amMaster){
			mesh.DHCP();
		}
		while (network.available()){
			RF24NetworkHeader header;
		}

		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}
