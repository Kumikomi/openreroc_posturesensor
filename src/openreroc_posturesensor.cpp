#include "ros/ros.h"
#include "posture_sensor.h"
#include "accel_sensor.h"
#include "gyro_sensor.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

typedef struct AccelSensor_Data
{
	int ax;
	int ay;
	int az;
} accelsensor_data;

typedef struct GyroSensor_Data
{
	int gx;
	int gy;
	int gz;
} gyrosensor_data;

int main(int argc, char **argv)
{
  int fd_32;
  int rc;
  
  int gyro_x;
  int gyro_y;
  int gyro_z;
  int accel_x;
  int accel_y;
  int accel_z;
  int gyro_x_signed;
  int gyro_y_signed;
  int gyro_z_signed;
  int accel_x_signed;
  int accel_y_signed;
  int accel_z_signed;
  float real_gx;
  float real_gy;
  float real_gz;
  float real_ax;
  float real_ay;
  float real_az;

  fd_32 = open("/dev/xillybus_read_32", O_RDONLY);

  ros::init(argc, argv, "openreroc_posturesensor");
  ros::NodeHandle n;
  ros::Publisher pub_openreroc_posturesensor = n.advertise<openreroc_posturesensor::posture_sensor>("posture_sensor_value", 1000);
  ros::Publisher pub_openreroc_accelsensor = n.advertise<openreroc_posturesensor::accel_sensor>("accel_sensor_value",1000);
  ros::Publisher pub_openreroc_gyrosensor = n.advertise<openreroc_posturesensor::gyro_sensor>("gyro_sensor_value",1000);
  ros::Rate loop_rate(1);

  openreroc_posturesensor::posture_sensor msg;
  openreroc_posturesensor::accel_sensor msg_a;
  openreroc_posturesensor::gyro_sensor msg_g;

  gyrosensor_data cur_g;
  accelsensor_data cur_a;

  while (ros::ok())
  {
    rc = read(fd_32, &accel_x, sizeof(accel_x));
    rc = read(fd_32, &accel_y, sizeof(accel_y));
    rc = read(fd_32, &accel_z, sizeof(accel_z));
    rc = read(fd_32, &gyro_x, sizeof(gyro_x));
    rc = read(fd_32, &gyro_y, sizeof(gyro_y));
    rc = read(fd_32, &gyro_z, sizeof(gyro_z));

	
    if(cur_g.gx != gyro_x && cur_g.gy != gyro_y && cur_g.gz != gyro_z){
	if(cur_a.ax != accel_x && cur_a.ay != accel_y && cur_a.az != accel_z){
      
	  accel_x_signed = (accel_x > 32768)? (accel_x-65535) : accel_x;
      accel_y_signed = (accel_y > 32768)? (accel_y-65535) : accel_y;
      accel_z_signed = (accel_z > 32768)? (accel_z-65535) : accel_z;  
	  gyro_x_signed = (gyro_x > 32768)? (gyro_x-65535) : gyro_x;
      gyro_y_signed = (gyro_y > 32768)? (gyro_y-65535) : gyro_y;
      gyro_z_signed = (gyro_z > 32768)? (gyro_z-65535) : gyro_z;    

      //msg.roll = accel_x_signed / 16384.0;
      // msg.pitch = accel_y_signed / 16384.0;
      // msg.yaw = accel_z_signed / 16384.0;
      msg_a.real_ax = accel_x_signed / 16384.0;
      msg_a.real_ay = accel_y_signed / 16384.0;
      msg_a.real_az = accel_z_signed / 16384.0;
      msg_g.real_gx = gyro_x_signed / 131.0;
      msg_g.real_gy = gyro_y_signed / 131.0;
      msg_g.real_gz = gyro_z_signed / 131.0;
		
	  
	  
      //printf("x:%d\n",msg.gx);
      //printf("y:%d\n",msg.gy);
      //printf("z:%d\n",msg.gz);
      printf("raw_ax:%d\n",accel_x);
      printf("raw_ay:%d\n",accel_y);
      printf("raw_az:%d\n",accel_z);
      printf("raw_gx:%d\n",gyro_x);
      printf("raw_gy:%d\n",gyro_y);
      printf("raw_gz:%d\n",gyro_z);

      pub_openreroc_posturesensor.publish(msg);
      pub_openreroc_accelsensor.publish(msg_a);
      pub_openreroc_gyrosensor.publish(msg_g);
    }
	}

	
    cur_a.ax = accel_x;
    cur_a.ay = accel_y;
    cur_a.az = accel_z;
    cur_g.gx = gyro_x;
    cur_g.gy = gyro_y;
    cur_g.gz = gyro_z;

    ros::spinOnce();
    // loop_rate.sleep();
  }

  close(fd_32);
  return 0;
}
