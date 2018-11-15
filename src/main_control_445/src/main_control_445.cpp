#include <ros/ros.h>

#include <std_msgs/Int16.h>
#include <rf_comms_445/RFPayload.h>

#include <iostream>
#include <fstream>

bool amMaster;

bool newPayload = false;
rf_comms_445::RFPayload rfMessageReceived;
void rfReceivedCallback(const rf_comms_445::RFPayload::ConstPtr & msg) {
	rfMessageReceived.destination = msg->destination;
	rfMessageReceived.message_type = msg->message_type;
	rfMessageReceived.seq_num = msg->seq_num;
	rfMessageReceived.x = msg->x;
	rfMessageReceived.y = msg->y;
	rfMessageReceived.orientation = msg->orientation;
	rfMessageReceived.task_done = msg->task_done;
	rfMessageReceived.motor_cmd = msg->motor_cmd;
	rfMessageReceived.motor_duration = msg->motor_duration;
	newPayload = true;
}

int main(int argc, char **argv)
{
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

    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::Int16>("motor_commands", 1000);
	ros::Publisher rfToSendPub = n.advertise<rf_comms_445::RFPayload>("rf_to_send", 1000);
	ros::Subscriber rfReceivedSub = n.subscribe("rf_received", 1000, rfReceivedCallback);

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

        chatter_pub.publish(msg);

		// Master Test send
		if(amMaster){
			rf_comms_445::RFPayload payloadToSend;
			payloadToSend.destination = 2;
			payloadToSend.seq_num = count;
			payloadToSend.x = 0;
			payloadToSend.y = 0;
			payloadToSend.orientation = 0;
			payloadToSend.task_done = false;
			payloadToSend.motor_cmd = motorCmd;
			payloadToSend.motor_duration = 100;
			rfToSendPub.publish(payloadToSend);
		}

        ros::spinOnce();

		if(newPayload){
			// There is a new payload from another node
			// TODO act on it
			std::cout << "Main control received payload with seqNum " << rfMessageReceived.seq_num << std::endl;
			newPayload = false;
		}

        loop_rate.sleep();
        ++count;
        motorValue++;
    }
    return 0;
}
