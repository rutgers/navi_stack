#!/bin/bash
rosbag record -o cameras /tf /vision/camera/{left,middle,right,wheel/left,wheel/right}/{image,camera_info}
