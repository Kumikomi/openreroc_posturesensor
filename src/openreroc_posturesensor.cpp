//flat
#include "ros/ros.h"
#include "posture_sensor.h"
#include "accel_sensor.h"
#include "gyro_sensor.h"
#include "accel_angle.h"
#include "gyro_angle.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#define M_PI 3.14159265358979 /* pi */
#define rad2deg(a) ( (a) / M_PI * 180.0 ) /* rad to deg */
#include <sys/time.h>
#define N 10    //for gyro HPF
#define M 100    //for offset
int ptr = 0;    //for gyro HPF
int count = 0;  //for offset
int sum_x =0, sum_y=0, sum_z=0;


double get_dtime(void){
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return ((double)(tv.tv_sec) + (double)(tv.tv_usec) * 0.001 * 0.001);
}

struct sensor_data_i
{
  int x;
  int y;
  int z;
}; 

 struct sensor_data_f
{
  float x;
  float y;
  float z;
}; 


struct sensor_data_d
{
  double x;
  double y;
  double z;
};

///************************************************************///

struct sensor_data_f offset_calc(sensor_data_i signed_gy)
{ 
  struct sensor_data_f offset_gy_temp;
  int i,j;
  int buff_x[M];
  int buff_y[M];
  int buff_z[M];

  if(count <= M-1){   //fill in buff[0]~buff[N]
    buff_x[count] = signed_gy.x;
    buff_y[count] = signed_gy.y;
    buff_z[count] = signed_gy.z;
    sum_x += signed_gy.x;
    sum_y += signed_gy.y;
    sum_z += signed_gy.z;
    count += 1;
    offset_gy_temp.x = 0.0;
    offset_gy_temp.y = 0.0;
    offset_gy_temp.z = 0.0;
    if(count == M-1){
        printf("offset_calc_finish\n");
    }
  }

  else {
    offset_gy_temp.x = (float)signed_gy.x - ((float)sum_x / (float)M);
    offset_gy_temp.y = (float)signed_gy.y - ((float)sum_y / (float)M);
    offset_gy_temp.z = (float)signed_gy.z - ((float)sum_z / (float)M);
  }

  return offset_gy_temp;
}

struct sensor_data_f accel_lpf(sensor_data_f lpf_acc, openreroc_posturesensor::accel_sensor *msg_a)
{
  struct sensor_data_f lpf_acc_temp;
  lpf_acc_temp.x = lpf_acc.x*0.9 + (msg_a->real_ax)*0.1;
  lpf_acc_temp.y = lpf_acc.y*0.9 + (msg_a->real_ay)*0.1;
  lpf_acc_temp.z = lpf_acc.z*0.9 + (msg_a->real_az)*0.1;
  return lpf_acc_temp;
}

struct sensor_data_f gyro_hpf(openreroc_posturesensor::gyro_sensor *msg_g)
{
  struct sensor_data_f hpf_gy_temp; 
  int i,j;
  float sum_x, sum_y, sum_z;
  float buff_x[N];
  float buff_y[N];
  float buff_z[N];

  if(ptr <= N-1){   //fill in buff[0]~buff[N]
    buff_x[ptr] = msg_g->real_gx;
    buff_y[ptr] = msg_g->real_gy;
    buff_z[ptr] = msg_g->real_gz;
    sum_x += msg_g->real_gx;
    sum_y += msg_g->real_gy;
    sum_z += msg_g->real_gz;
    ptr += 1;
    hpf_gy_temp.x = 0;
    hpf_gy_temp.y = 0;
    hpf_gy_temp.z = 0;
  }

  else {
    sum_x += (float)(msg_g->real_gx - buff_x[0]);
    sum_y += (float)(msg_g->real_gy - buff_y[0]);
    sum_z += (float)(msg_g->real_gz - buff_z[0]);

    for(i=0; i<=N-1; i++){
       buff_x[i] = buff_x[i+1];
       buff_y[i] = buff_y[i+1];
       buff_z[i] = buff_z[i+1];
    }
    buff_x[N-1] = msg_g->real_gx;    //new data
    buff_y[N-1] = msg_g->real_gy;
    buff_z[N-1] = msg_g->real_gz;

    hpf_gy_temp.x = msg_g->real_gx - (sum_x / (float)N);
    hpf_gy_temp.y = msg_g->real_gy - (sum_y / (float)N);
    hpf_gy_temp.z = msg_g->real_gz - (sum_z / (float)N);
 }
 return hpf_gy_temp;
}

struct sensor_data_f accel_angle(sensor_data_f lpf_acc)
{
    struct sensor_data_f angle_acc_temp;
//1axis : θx = asin(y / √(z^2 + y^2))
    //angle_acc_temp.x =  asin(lpf_acc.y / sqrt(lpf_acc.z * lpf_acc.z + lpf_acc.y * lpf_acc.y));
  //2axis : θx = atan2(y,z)
    //angle_acc_temp.x = atan2(lpf_acc.y, lpf_acc.z);
  //3axis : θx = atan2(x,y)
    angle_acc_temp.x = atan2(lpf_acc.x, sqrt(lpf_acc.y*lpf_acc.y + lpf_acc.z*lpf_acc.z));

  //1axis : θy = asin(x / √(z^2 + x^2))
    //angle_acc_temp.y =  asin(lpf_acc.x / sqrt(lpf_acc.z * lpf_acc.z + lpf_acc.x * lpf_acc.x));
  //2axis : θy = atan2(x,z)
    //angle_acc_temp.y = atan2(lpf_acc.x, lpf_acc.z);
  //3axis : θy = acos(z / √(x^2 + y^2 + z^2))
    angle_acc_temp.y = atan2(lpf_acc.y, sqrt(lpf_acc.x*lpf_acc.x + lpf_acc.z*lpf_acc.z));

  //3axis : 0z = atan2((x^2+y^2)/z)
    angle_acc_temp.z = atan2(sqrt(lpf_acc.x*lpf_acc.x + lpf_acc.y*lpf_acc.y), lpf_acc.z);
   
   angle_acc_temp.x = rad2deg(angle_acc_temp.x);      //radian to degree
   angle_acc_temp.y = rad2deg(angle_acc_temp.y);      //radian to degree
  angle_acc_temp.z = rad2deg(angle_acc_temp.z);      //radian to degree

   return angle_acc_temp;
}

struct sensor_data_d gyro_angle(double &delta_t, openreroc_posturesensor::gyro_sensor *msg_g)
{
     struct sensor_data_d gyro_angle_temp;    
     gyro_angle_temp.x = (double)(msg_g->real_gx) * (delta_t);
     gyro_angle_temp.y = (double)(msg_g->real_gy) * (delta_t);
     gyro_angle_temp.z = (double)(msg_g->real_gz) * (delta_t);
     
     return gyro_angle_temp;
}


double complement_filter(double &gyro_ang, float &acc_ang, double &posture_angle)
{
    posture_angle = 0.95*(posture_angle + gyro_ang) + 0.05*acc_ang;
    return posture_angle;
}


///**********************************************************************///

int main(int argc, char **argv)
{
  int fd_32;
  int rc;
  
  double new_time = 0;
  double old_time = 0;
  double t1,t2,t3,t4 , t_sump= 0;
  double sampling_interval = 0;

  //double time_sum = 0;//for debug
  double time_buff =0 ;

  fd_32 = open("/dev/xillybus_read_32", O_RDONLY);

  ros::init(argc, argv, "openreroc_posturesensor");
  ros::NodeHandle n;
  ros::Publisher pub_openreroc_posturesensor = n.advertise<openreroc_posturesensor::posture_sensor>("posture_sensor_value", 1000);
  ros::Publisher pub_openreroc_accelsensor = n.advertise<openreroc_posturesensor::accel_sensor>("accel_sensor_value",1000);
  ros::Publisher pub_openreroc_gyrosensor = n.advertise<openreroc_posturesensor::gyro_sensor>("gyro_sensor_value",1000);
//for debug
  ros::Publisher pub_openreroc_accel_ang = n.advertise<openreroc_posturesensor::accel_angle>("accel_angle_value", 1000);
  ros::Publisher pub_openreroc_gyro_ang = n.advertise<openreroc_posturesensor::gyro_angle>("gyro_angle_value", 1000);
 // ros::Rate loop_rate(1000);

  openreroc_posturesensor::posture_sensor msg;
  openreroc_posturesensor::accel_sensor msg_a;
  openreroc_posturesensor::gyro_sensor msg_g;
//
  openreroc_posturesensor::accel_angle msg_a_ang;
  openreroc_posturesensor::gyro_angle msg_g_ang;
  

  sensor_data_i raw_acc;
  sensor_data_i raw_gy;
  sensor_data_i cur_gy;     //before raw gyro data
  sensor_data_i cur_acc;    //before raw accel data
  sensor_data_i signed_acc; //signed accel data
  sensor_data_i signed_gy;  //signed gyro data
  sensor_data_f offset_gy;  //after offset_calc
  sensor_data_f lpf_acc;    //LPF to accel data
  sensor_data_f angle_acc;  //angle from LPF accel data
  sensor_data_f hpf_gy;     //HPF to gyro data
  sensor_data_d angle_gy;   //(double) angle from HPF gyro data

  while (ros::ok())
  {
    t1 = get_dtime();
    rc = read(fd_32, &raw_acc.x, sizeof(raw_acc.x));
    rc = read(fd_32, &raw_acc.y, sizeof(raw_acc.y)); 
    rc = read(fd_32, &raw_acc.z, sizeof(raw_acc.z));
    rc = read(fd_32, &raw_gy.x, sizeof(raw_gy.x));
    rc = read(fd_32, &raw_gy.y, sizeof(raw_gy.y));
    rc = read(fd_32, &raw_gy.z, sizeof(raw_gy.z)); 
    t2 = get_dtime();

    //for sampling_interval 
    //old_time = new_time;
    //new_time = get_dtime();
    //sampling_interval = new_time - old_time;
   
int same_value  = 1;

  if(cur_gy.x != raw_gy.x && cur_gy.y != raw_gy.y && cur_gy.z != raw_gy.z){   
    if(cur_acc.x != raw_acc.x && cur_acc.y != raw_acc.y && cur_acc.z != raw_acc.z){



      same_value = 0;

      old_time = new_time;
      new_time = get_dtime();
      sampling_interval = new_time - old_time;
   

      time_buff = t_sump;
      t_sump = get_dtime();
 
      signed_acc.x = (raw_acc.x > 0x8000)? (raw_acc.x-0xFFFF) : raw_acc.x;
      signed_acc.y = (raw_acc.y > 0x8000)? (raw_acc.y-0xFFFF) : raw_acc.y;
      signed_acc.z = (raw_acc.z > 0x8000)? (raw_acc.z-0xFFFF) : raw_acc.z;  
      signed_gy.x = (raw_gy.x > 0x8000)? (raw_gy.x-0xFFFF) : raw_gy.x;
      signed_gy.y = (raw_gy.y > 0x8000)? (raw_gy.y-0xFFFF) : raw_gy.y;
      signed_gy.z = (raw_gy.z > 0x8000)? (raw_gy.z-0xFFFF) : raw_gy.z;    

      offset_gy = offset_calc(signed_gy);

    //physical quantitiy conversion
      msg_a.real_ax = (float)signed_acc.x / 16384.0;
      msg_a.real_ay = (float)signed_acc.y / 16384.0;
      msg_a.real_az = (float)signed_acc.z / 16384.0;
      msg_g.real_gx = offset_gy.x / 131.0;
      msg_g.real_gy = offset_gy.y / 131.0;
      msg_g.real_gz = offset_gy.z / 131.0;
      

    //accel lpf
        lpf_acc = accel_lpf(lpf_acc, &msg_a);
    //accel angle conversion
        angle_acc = accel_angle(lpf_acc);
    //gyro HPF (1-LPF) 
        hpf_gy = gyro_hpf(&msg_g);
    //gyro angle conversion 
        angle_gy = gyro_angle(sampling_interval, &msg_g);
    //complement_filter
      msg.roll  = complement_filter(angle_gy.x, angle_acc.x, msg.roll);
      msg.pitch = complement_filter(angle_gy.y, angle_acc.y, msg.pitch);
      msg.yaw = complement_filter(angle_gy.z, angle_acc.z, msg.yaw);
      //msg.yaw   += angle_gy.z;   //can't calc yaw
      
      printf("angle: %f %f %f \n", msg.roll, msg.pitch, msg.yaw); //for debug print
     

      t3 = get_dtime();
      pub_openreroc_posturesensor.publish(msg);
      pub_openreroc_accelsensor.publish(msg_a);
      pub_openreroc_gyrosensor.publish(msg_g);

      msg_g_ang.gy_x_ang += angle_gy.x;
      msg_g_ang.gy_y_ang += angle_gy.y;
      msg_g_ang.gy_z_ang += angle_gy.z;
      msg_a_ang.ac_x_ang = angle_acc.x;
      msg_a_ang.ac_y_ang = angle_acc.y;
      msg_a_ang.ac_z_ang = angle_acc.z;

      pub_openreroc_gyro_ang.publish(msg_g_ang);
      pub_openreroc_accel_ang.publish(msg_a_ang);

      t4 = get_dtime();
      //printf("2-3:%.8f, 3-4:%.8f same:%d\n", t3-t2, t4-t3, same_value);

    }
  }

      //printf("1-2:%.8f, 2-3:%.8f 3-4%.8f, all:%.8f, same:%d \n", 
      //t2-t1, t3-t2, t4-t3, t4-t1, same_value);  


    cur_acc.x = raw_acc.x;
    cur_acc.y = raw_acc.y;
    cur_acc.z = raw_acc.z;
    cur_gy.x = raw_gy.x;
    cur_gy.y = raw_gy.y;
    cur_gy.z = raw_gy.z;

    ros::spinOnce();
     //loop_rate.sleep();
  }

  close(fd_32);
    return 0;
}
