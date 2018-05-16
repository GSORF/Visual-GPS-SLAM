#!/bin/bash
#Shebang

#Start roscore and ZED wrapper
#-----------------------------
#roscore&
roslaunch zed_wrapper zed.launch&
Process1=$!
sleep 10s

# Start DSO
#-----------------------------
source runDSOOnline.sh &
Process2=$!

wait $Process1 $Process2
