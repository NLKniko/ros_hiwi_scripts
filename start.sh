#!/usr/bin/bash

var=`roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx250s use_nav:=true use_lidar:=true rtabmap_args:=-d enable_pipeline:=true use_perception:=true
var=`gnome-terminal &`
var=`cd workspace/ros_hiwi_scripts/`
var=`python3 put_in_bag.py`