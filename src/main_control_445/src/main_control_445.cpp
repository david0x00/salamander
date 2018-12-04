#include <ros/ros.h>

#include <std_msgs/Int64.h>
#include <rf_comms_445/RFPayload.h>

#include <iostream>
#include <fstream>
#include <string>

bool amMaster;
bool newPayload = false;
rf_comms_445::RFPayload rfMessageReceived;

bool motorDone = false;

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

void motorDoneCallback(const std_msgs::Int64::ConstPtr & motorDoneMsg) {
	motorDone = true;
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
	bool *taskSent = new bool[10];
	for(int i = 0; i < 10; ++i){
		taskSent[i] = false;
	}

    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher motorPub = n.advertise<std_msgs::Int64>("motor_commands", 1000);
	ros::Subscriber motorSub = n.subscribe("motor_responses", 1000, motorDoneCallback);
	ros::Publisher rfToSendPub = n.advertise<rf_comms_445::RFPayload>("rf_to_send", 1000);
	ros::Subscriber rfReceivedSub = n.subscribe("rf_received", 1000, rfReceivedCallback);

    ros::Rate loop_rate(1);

    int count = 0;
    unsigned char motorValue = 48;
    while (ros::ok())
    {
        //if(motorValue == 55){
           // motorValue = 32;
        //}

		// Master Test send
		if(amMaster){
			std::string inputCmd = "w2";
			std::cin >> inputCmd;
			uint8_t tmpDest = 0;
			tmpDest = inputCmd[1] - 48;
			char moveCmd = inputCmd[0];
			bool leftForward = true;
			bool rightForward = true;
			switch (moveCmd) {
				case 'w':
					leftForward = true;
					rightForward = true;
					break;
				case 's':
					leftForward = false;
					rightForward = false;
					break;
				case 'a':
					leftForward = true;
					rightForward = false;
					break;
				case 'd':
					leftForward = false;
					rightForward = true;
					break;
				default:
					break;
			}
			//std::cout << moveCmd << tmpDest << std::endl;

			if(taskSent[tmpDest] == false){
		        unsigned int motorCmd = 0;
				if(!rightForward){
					motorCmd |= 0xC0;
				}
				motorCmd |= motorValue;
		        motorCmd <<= 8;

				if(!leftForward){
					motorCmd |= 0xC0;
				}
		        motorCmd |= motorValue;
				rf_comms_445::RFPayload payloadToSend;
				payloadToSend.source = 0;
				payloadToSend.destination = tmpDest;
				payloadToSend.seq_num = count;
				payloadToSend.x = 0;
				payloadToSend.y = 0;
				payloadToSend.orientation = 0;
				payloadToSend.task_done = false;
				payloadToSend.motor_cmd = motorCmd;
				payloadToSend.motor_duration = 500;
				rfToSendPub.publish(payloadToSend);
				//taskSent[tmpDest] = true;
			}
		}

        ros::spinOnce();

		if(newPayload){
			// There is a new payload from another node
			// TODO act on it
			std::cout << "Main control received payload with seqNum " << rfMessageReceived.seq_num << std::endl;
			if(rfMessageReceived.task_done == true){
				std::cout << "Task is done" << std::endl;
				taskSent[rfMessageReceived.source] = false;
			}

			if(rfMessageReceived.motor_duration != 0){

        		std_msgs::Int64 motorMsg;
		        uint64_t motorData = rfMessageReceived.motor_duration << 16;
				motorData |= rfMessageReceived.motor_cmd;
		        motorMsg.data = motorData;
		        motorPub.publish(motorMsg);
			}
			newPayload = false;
		}

		if(motorDone){
			motorDone = false;

			rf_comms_445::RFPayload payloadToSend;
			payloadToSend.source = nodeId;
			payloadToSend.destination = 0;
			payloadToSend.seq_num = count;
			payloadToSend.x = 0;
			payloadToSend.y = 0;
			payloadToSend.orientation = 0;
			payloadToSend.task_done = true;
			payloadToSend.motor_cmd = 0;
			payloadToSend.motor_duration = 0;
			rfToSendPub.publish(payloadToSend);
		}

        loop_rate.sleep();
        ++count;
    }
	delete taskSent;
    return 0;
}
