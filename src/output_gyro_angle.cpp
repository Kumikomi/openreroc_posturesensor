/*******output 3axis_acceleration[G]*******/
#include "ros/ros.h"
#include "gyro_angle.h"
#include <stdio.h>

void chatterCallback(const openreroc_posturesensor::gyro_angle msg_g_ang)
{
  printf("gyro_x:%f\n",msg_g_ang.gy_x_ang);
  printf("gyro_y:%f\n",msg_g_ang.gy_y_ang);
  printf("gyro_z:%f\n",msg_g_ang.gy_z_ang);
}

int main(int argc, char  **argv)
{
  ros::init(argc, argv, "output_gyro_angle");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("gyro_angle_value", 1000, chatterCallback)\
    ;
  ros::spin();
  return 0;
}
