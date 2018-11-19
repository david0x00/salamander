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
rf_comms_445::RFPayload rfMessageToSend;

void rfSendCallback(const rf_comms_445::RFPayload::ConstPtr & msg) {
	rfMessageToSend.message_type = msg->message_type;
	rfMessageToSend.seq_num = msg->seq_num;
	rfMessageToSend.x = msg->x;
	rfMessageToSend.y = msg->y;
	rfMessageToSend.orientation = msg->orientation;
	rfMessageToSend.task_done = msg->task_done;
	rfMessageToSend.motor_cmd = msg->motor_cmd;
	rfMessageToSend.motor_duration = msg->motor_duration;
	rfMessageToSend.destination = msg->destination;
	isPayloadToSend = true;
	payloadDestination = msg->destination;
}

int main(int argc, char **argv) {
	// get node ID from file
	std::ifstream nodeIDFile;
	std::string line;
	unsigned int nodeId = 0;
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
	std::cout << "Node ID " << nodeId <<std::endl;
	// Setup RF24Mesh
	mesh.setNodeID((uint8_t)nodeId);
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

			rf_comms_445::RFPayload payloadRecv;
			switch(header.type){
				case 'M':
				{
					network.read(header, &payloadRecv, sizeof(payloadRecv));
					std::cout << "Recevied from " << header.from_node << " seq num " << payloadRecv.seq_num << std::endl;
					rfPub.publish(payloadRecv);
				} break;
				default:
					break;
			}
		}

		// Get ROS messages
		ros::spinOnce();

		// Check to send
		if(isPayloadToSend){
			if(mesh.write(&rfMessageToSend, 'M', sizeof(rfMessageToSend), payloadDestination)){
				// Send okay
				std::cout << "Sent okay to " << payloadDestination << std::endl;
			}else{
				// Send failed
				// Check connectivity with the mesh network
				if(!mesh.checkConnection()){
					std::cout << "Mesh renewing address" << std::endl;
					//mesh.renewAddress();
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
