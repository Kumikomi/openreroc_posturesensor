#!/bin/sh
export timestamp=$(date +%H%M%s)
echo $timestamp
rosrun openreroc_posturesensor openreroc_posturesensor > log/${timestamp}.txt