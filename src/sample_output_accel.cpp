/*******output 3axis_acceleration[G]*******/
#include "ros/ros.h"
#include "accel_sensor.h"
#include <stdio.h>

void chatterCallback(const openreroc_posturesensor::accel_sensor msg_a)
{
  printf("accel_x:%f ",msg_a.real_ax);
  printf("accel_y:%f ",msg_a.real_ay);
  printf("accel_z:%f\n",msg_a.real_az);
}

int main(int argc, char  **argv)
{
  ros::init(argc, argv, "sample_output_accel");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("accel_sensor_value", 1000, chatterCallback);
  ros::spin();
  return 0;
}
