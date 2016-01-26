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

// accel data temp
typedef struct Accel_Temp_Data
{
  float acc_x;
  float acc_y;
  float acc_z;
} accel_temp_data;

// gyro data temp
typedef struct Gyro_Temp_Data
{
  float gyro_x;
  float gyro_y;
  float gyro_z;
} gyro_temp_data;

typedef struct Gyro_Temp_Data_f
{
  double gyro_x;
  double gyro_y;
  double gyro_z;
} gyro_temp_data_f;

///************************************************************///

void offset_calc(float &off_x,float &off_y,float &off_z, int &signed_x, int &signed_y, int &signed_z)
{  int i,j;

  int buff_x[M];
  int buff_y[M];
  int buff_z[M];

  if(count <= M-1){   //fill in buff[0]~buff[N]
    buff_x[count] = signed_x;
    buff_y[count] = signed_y;
    buff_z[count] = signed_z;
    sum_x += signed_x;
    sum_y += signed_y;
    sum_z += signed_z;
    count += 1;
    off_x = 0.0;
    off_y = 0.0;
    off_z = 0.0;
    if(count == M-1){
        printf("offset_calc_finish\n");
    }
  }

  else {
    off_x = (float)signed_x - ((float)sum_x / (float)M);
    off_y = (float)signed_y - ((float)sum_y / (float)M);
    off_z = (float)signed_z - ((float)sum_z / (float)M);
  }
}

void accel_lpf(float &accel_x_lpf, float &accel_y_lpf, float &accel_z_lpf, float &acc_x, float &acc_y, float &acc_z)
{
  accel_x_lpf = accel_x_lpf*0.9 + acc_x*0.1;
  accel_y_lpf = accel_y_lpf*0.9 + acc_y*0.1;
  accel_z_lpf = accel_z_lpf*0.9 + acc_z*0.1;
}

void gyro_hpf(float &gy_x_hpf, float &gy_y_hpf, float &gy_z_hpf, float &real_gx, float &real_gy, float real_gz)
{
  int i,j;
  float sum_x, sum_y, sum_z;
  float buff_x[N];
  float buff_y[N];
  float buff_z[N];

  if(ptr <= N-1){   //fill in buff[0]~buff[N]
    buff_x[ptr] = real_gx;
    buff_y[ptr] = real_gy;
    buff_z[ptr] = real_gz;
    sum_x += real_gx;
    sum_y += real_gx;
    sum_z += real_gx;
    ptr += 1;
    gy_x_hpf = 0;
    gy_y_hpf = 0;
    gy_z_hpf = 0;
  }

  else {
    sum_x += (float)(real_gx - buff_x[0]);
    sum_y += (float)(real_gy - buff_y[0]);
    sum_z += (float)(real_gz - buff_z[0]);

    for(i=0; i<=N-1; i++){
       buff_x[i] = buff_x[i+1];
       buff_y[i] = buff_y[i+1];
       buff_z[i] = buff_z[i+1];
    }
    buff_x[N-1] = real_gx;    //new data
    buff_y[N-1] = real_gy;
    buff_z[N-1] = real_gz;

    gy_x_hpf = real_gx - (sum_x / (float)N);
    gy_y_hpf = real_gy - (sum_y / (float)N);
    gy_z_hpf = real_gz - (sum_z / (float)N);
 }
}

void accel_angle(float &acc_x_lpf, float &acc_y_lpf, float &acc_z_lpf, float &acc_x_ang, float &acc_y_ang, float &acc_z_ang)
{
  //1axis : θx = asin(y / √(z^2 + y^2))
    //acc_x_ang =  asin(acc_y_lpf / sqrt(acc_z_lpf * acc_z_lpf + acc_y_lpf * acc_y_lpf));
  //2axis : θx = atan2(y,z)
    acc_x_ang = atan2(acc_y_lpf, acc_z_lpf);
  //3axis : θx = atan2(x,y)
    //acc_x_ang = atan2(acc_x_lpf, acc_y_lpf);

  //1axis : θy = asin(x / √(z^2 + x^2))
    //acc_y_ang =  asin(acc_x_lpf / sqrt(acc_z_lpf * acc_z_lpf + acc_x_lpf * acc_x_lpf));
  //2axis : θx = atan2(x,z)
    acc_y_ang = atan2(acc_x_lpf, acc_z_lpf);
  //3axis : θx = acos(z / √(x^2 + y^2 + z^2))
    //acc_y_ang = acos(acc_z_lpf / (sqrt(acc_x_lpf*acc_x_lpf + acc_y_lpf*acc_y_lpf + acc_z_lpf*acc_z_lpf )) );
   acc_x_ang = rad2deg(acc_x_ang);      //radian to degree
   acc_y_ang = rad2deg(acc_y_ang);      //radian to degree
}

void gyro_angle(double &delta_t, float &real_gx, float &real_gy, float &real_gz, double &gy_x_ang, double &gy_y_ang, double &gy_z_ang)
{    
     gy_x_ang = (double)(real_gx) * (delta_t);
     gy_y_ang = (double)(real_gy) * (delta_t);
     gy_z_ang += (double)(real_gz) * (delta_t);
     
}

void complement_filter(double &gyro_ang, float &acc_ang, double &posture_angle)
{
    posture_angle = 0.95*(posture_angle + gyro_ang) + 0.05*acc_ang;
}


///**********************************************************************///

int main(int argc, char **argv)
{
  int fd_32;
  int rc;
  
  int gyro_x; //raw gyro data
  int gyro_y;
  int gyro_z;
  int accel_x; //raw acccel data
  int accel_y;
  int accel_z;

  double new_time = 0;
  double old_time = 0;
  double sampling_interval = 0;

  fd_32 = open("/dev/xillybus_read_32", O_RDONLY);

  ros::init(argc, argv, "openreroc_posturesensor");
  ros::NodeHandle n;
  ros::Publisher pub_openreroc_posturesensor = n.advertise<openreroc_posturesensor::posture_sensor>("posture_sensor_value", 1000);
  ros::Publisher pub_openreroc_accelsensor = n.advertise<openreroc_posturesensor::accel_sensor>("accel_sensor_value",1000);
  ros::Publisher pub_openreroc_gyrosensor = n.advertise<openreroc_posturesensor::gyro_sensor>("gyro_sensor_value",1000);
  //ros::Rate loop_rate(1);

  openreroc_posturesensor::posture_sensor msg;
  openreroc_posturesensor::accel_sensor msg_a;
  openreroc_posturesensor::gyro_sensor msg_g;

  gyrosensor_data cur_g;      //before raw gyro data
  accelsensor_data cur_a;     //before raw accel data
  accelsensor_data signed_a;  //signed accel data
  gyrosensor_data signed_g;   //signed gyro data
  gyro_temp_data offset_g;    //after offset_calc
  accel_temp_data lpf_a;      //LPF to accel data
  accel_temp_data angle_a;    //angle from LPF accel data
  gyro_temp_data hpf_g;       //HPF to gyro data
  gyro_temp_data_f angle_g;   //(double) angle from HPF gyro data

  while (ros::ok())
  {
    rc = read(fd_32, &accel_x, sizeof(accel_x));
    rc = read(fd_32, &accel_y, sizeof(accel_y));
    rc = read(fd_32, &accel_z, sizeof(accel_z));
    rc = read(fd_32, &gyro_x, sizeof(gyro_x));
    rc = read(fd_32, &gyro_y, sizeof(gyro_y));
    rc = read(fd_32, &gyro_z, sizeof(gyro_z));
    
    //for sampling_interval 
    old_time = new_time;
    new_time = get_dtime();
    sampling_interval = new_time - old_time;
	
  if(cur_g.gx != gyro_x && cur_g.gy != gyro_y && cur_g.gz != gyro_z){   
	  if(cur_a.ax != accel_x && cur_a.ay != accel_y && cur_a.az != accel_z){
      
	    signed_a.ax = (accel_x > 32768)? (accel_x-65535) : accel_x;
      signed_a.ay = (accel_y > 32768)? (accel_y-65535) : accel_y;
      signed_a.az = (accel_z > 32768)? (accel_z-65535) : accel_z;  
	    signed_g.gx = (gyro_x > 32768)? (gyro_x-65535) : gyro_x;
      signed_g.gy = (gyro_y > 32768)? (gyro_y-65535) : gyro_y;
      signed_g.gz = (gyro_z > 32768)? (gyro_z-65535) : gyro_z;    

     offset_calc(offset_g.gyro_x, offset_g.gyro_y, offset_g.gyro_z, signed_g.gx, signed_g.gy, signed_g.gz);

    //physical quantitiy conversion
      msg_a.real_ax = (float)signed_a.ax / 16384.0;
      msg_a.real_ay = (float)signed_a.ay / 16384.0;
      msg_a.real_az = (float)signed_a.az / 16384.0;
      msg_g.real_gx = offset_g.gyro_x / 131.0;
      msg_g.real_gy = offset_g.gyro_y / 131.0;
      msg_g.real_gz = offset_g.gyro_z / 131.0;
      

    //accel lpf
      accel_lpf(lpf_a.acc_x, lpf_a.acc_y, lpf_a.acc_z, msg_a.real_ax, msg_a.real_ay, msg_a.real_az);
    //accel angle conversion
      accel_angle(lpf_a.acc_x, lpf_a.acc_y, lpf_a.acc_z, angle_a.acc_x, angle_a.acc_y, angle_a.acc_z);//accel_x_angle, accel_y_angle, accel_z_angle);    
    //gyro HPF (1-LPF) 
      gyro_hpf(hpf_g.gyro_x, hpf_g.gyro_y, hpf_g.gyro_z, msg_g.real_gx, msg_g.real_gy, msg_g.real_gz);
    //gyro angle conversion 
      gyro_angle(sampling_interval, msg_g.real_gx, msg_g.real_gy, msg_g.real_gz, angle_g.gyro_x, angle_g.gyro_y, angle_g.gyro_z);
    //complement_filter
      complement_filter(angle_g.gyro_x, angle_a.acc_x, msg.roll);
      complement_filter(angle_g.gyro_y, angle_a.acc_y, msg.pitch);
      msg.yaw = angle_g.gyro_z;   //can't calc yaw
      
      printf("angle: %f %f %f \n", msg.roll, msg.pitch, msg.yaw); //for debug print

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
