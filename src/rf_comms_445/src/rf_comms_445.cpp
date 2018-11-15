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

struct Payload {
	char messageType;
	unsigned int seqNum;
	float x;
	float y;
	float orientation;
	bool taskDone;
	unsigned short motorCmd;
	unsigned int motorDuration;
};


// Setup for GPIO 22 CE and CE1 CSN with SPI Speed @ 8Mhz
RF24 radio(RPI_V2_GPIO_P1_15, BCM2835_SPI_CS0, BCM2835_SPI_SPEED_8MHZ);  

RF24Network network(radio);
RF24Mesh mesh(radio,network);

bool amMaster = false;
bool isPayloadToSend = false;
unsigned int payloadDestination = 0;
Payload rfMessageToSend;

void rfSendCallback(const rf_comms_445::RFPayload::ConstPtr & msg) {
	rfMessageToSend.messageType = msg->message_type;
	rfMessageToSend.seqNum = msg->seq_num;
	rfMessageToSend.x = msg->x;
	rfMessageToSend.y = msg->y;
	rfMessageToSend.orientation = msg->orientation;
	rfMessageToSend.taskDone = msg->task_done;
	rfMessageToSend.motorCmd = msg->motor_cmd;
	rfMessageToSend.motorDuration = msg->motor_duration;
	isPayloadToSend = true;
	payloadDestination = msg->destination;
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
		// Check to receive
		if (network.available()){
			RF24NetworkHeader header;
			network.peek(header);

			Payload payloadRecv;
			switch(header.type){
				case 'M':
				{
					network.read(header, &payloadRecv, sizeof(Payload));
					std::cout << "Recevied from " << header.from_node << " seq num " << payloadRecv.seqNum << std::endl;
				} break;
				default:
					break;
			}
		}

		// Get ROS messages
		ros::spinOnce();

		// Check to send
		if(isPayloadToSend){
			if(mesh.write(&rfMessageToSend, 'M', sizeof(Payload), payloadDestination)){
				// Send okay
				std::cout << "Sent okay to " << payloadDestination << std::endl;
			}else{
				// Send failed
				// Check connectivity with the mesh network
				if(!mesh.checkConnection()){
					std::cout << "Mesh renewing address" << std::endl;
					mesh.renewAddress();
				}else{
					std::cout << "Send failed to " << payloadDestination << std::endl;
				}
			}
			isPayloadToSend = false;
		}

		loop_rate.sleep();
	}
	return 0;
}
