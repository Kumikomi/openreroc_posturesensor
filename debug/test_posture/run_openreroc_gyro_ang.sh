#!/bin/sh
export timestamp=$(date +%H%M%s)
echo $timestamp
rosrun openreroc_posturesensor output_gyro_angle > log/gyro_${timestamp}.txt