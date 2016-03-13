/******* output 3axis_posture_angle[deg] *******/
#include "ros/ros.h"
#include "posture_sensor.h"
#include <stdio.h>

void chatterCallback(const openreroc_posturesensor::posture_sensor msg)
{
  printf("x:%f\n",msg.roll);
  printf("y:%f\n",msg.pitch);
  printf("z:%f\n",msg.yaw);
}

int main(int argc, char  **argv)
{
	ros::init(argc, argv, "sample_output_posture");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("posture_sensor_value", 1000, chatterCallback);
	ros::spin();
  	return 0;
}
