#!/bin/bash
rosservice call /finish_trajectory 0
sleep 5
rosservice call /write_state "{filename: '${HOME}/data/cartographer/map/$1.pbstream'}"
rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/data/cartographer/map/$1 -pbstream_filename=${HOME}/data/cartographer/map/$1.pbstream  -resolution=0.05
echo "succeed save map!"
