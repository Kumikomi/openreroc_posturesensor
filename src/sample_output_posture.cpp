#include "ros/ros.h"
#include "posture_sensor.h"
#include <stdio.h>

void chatterCallback(const openreroc_posturesensor::posture_sensor msg)
{
  //printf("x:%f\n",msg_g.real_gx);
  // printf("y:%f\n",msg_g.real_gy);
  // printf("z:%f\n",msg_g.real_gz);
  printf("x:%ftest\n",msg.roll);
  printf("y:%ftest\n",msg.pitch);
  printf("z:%ftest\n",msg.yaw);
}

int main(int argc, char  **argv)
{
	ros::init(argc, argv, "sample_output_posture");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("posture_sensor_value", 1000, chatterCallback);
	ros::spin();
  	return 0;
}
