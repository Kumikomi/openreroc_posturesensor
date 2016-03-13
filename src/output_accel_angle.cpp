/*******output 3axis_acceleration[G]*******/
#include "ros/ros.h"
#include "accel_angle.h"
#include <stdio.h>

void chatterCallback(const openreroc_posturesensor::accel_angle msg_a_ang)
{
  printf("accel_x:%f\n ", msg_a_ang.ac_x_ang);
  printf("accel_y:%f\n", msg_a_ang.ac_y_ang);
  printf("accel_z:%f\n", msg_a_ang.ac_z_ang);
}

int main(int argc, char  **argv)
{
  ros::init(argc, argv, "output_accel_angle");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("accel_angle_value", 1000, chatterCallback);
  ros::spin();
  return 0;
}
