#include <ros/ros.h>
#include <wiringPi.h>
#include <softPwm.h>

// Pin number declarations. We're using the Broadcom chip pin numbers.
const int pwmPin = 18; // PWM LED - Broadcom pin 18, P1 pin 12

const int pwmValue = 75; // Use this to set an LED brightness

int main(int argc, char **argv) {
  // Setup stuff:
  wiringPiSetup(); // Initialize wiringPi -- using Broadcom pin numbers

  softPwmCreate(28, 75, 100);
  softPwmCreate(29, 75, 100);

  ROS_INFO("Loop is running");

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
