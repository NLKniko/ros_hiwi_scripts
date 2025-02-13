#!/usr/bin/bash

var=`roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx250s enable_pipeline:=true use_perception:=true use_base:=true`
var=`gnome-terminal &`
var=`cd workspace/ros_hiwi_scripts/`
var=`python3 perception.py`