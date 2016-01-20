#include "ros/ros.h"
#include "gyro_sensor.h"
#include <stdio.h>

void chatterCallback(const openreroc_posturesensor::gyro_sensor msg_g)
{
  //printf("gyro_x:%f\n",msg_g.real_gx);
  // printf("gyro_y:%f\n",msg_g.real_gy);
  // printf("gyro_z:%f\n",msg_g.real_gz);
}

int main(int argc, char  **argv)
{
  ros::init(argc, argv, "sample_output_gyro");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("gyro_sensor_value", 1000, chatterCallback);
  ros::spin();
  return 0;
}
