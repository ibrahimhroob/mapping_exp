#!/bin/bash

######### BASH TERMINAL COLORS ################################################
# Black        0;30     Dark Gray     1;30
# Red          0;31     Light Red     1;31
# Green        0;32     Light Green   1;32
# Brown/Orange 0;33     Yellow        1;33
# Blue         0;34     Light Blue    1;34
# Purple       0;35     Light Purple  1;35
# Cyan         0;36     Light Cyan    1;36
# Light Gray   0;37     White         1;37

RED='\033[0;31m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

printf "${PURPLE}***Script ID: Mapping experiment***${NC}\n"

######### DIRECTORIES & FILES #################################################
CATKIN_WS=/home/computing/neptune/catkin_ws
FASTLIO_PCD_DIR=${CATKIN_WS}/src/FAST_LIO/PCD

FILTER_SCRIPT=/home/computing/neptune/inference_model/stability_filter.py

MAP_ODOM_TOPIC=/Odometry

GT_TOPIC=/odometry/gps
IMU_TOPIC=/os_cloud_node/imu 
POINTS_TOPIC=/os_cloud_node/points

PLAY_RATE=0.2

# Set the IP address of the remote machine
REMOTE_IP=10.5.37.139

BAGS_REMOTE_DIR=/mnt/2564a3b0-8e42-43a4-b917-2b1af0c78052/bacchus/dataset/bags

source $CATKIN_WS/devel/setup.bash

bags=("june_1" "june_8" "june_29" "july_13")

for bag in "${bags[@]}"; do
    printf "Mapping: ${BLUE}${bag}${NC} ...\n"
    exp_dir=$bag
    pcd_dir=$exp_dir/pcd
    map_odom_dir=$exp_dir/map_odom

    mkdir -p $pcd_dir
    mkdir -p $map_odom_dir 

    odom_bag=$map_odom_dir/$bag
    printf "${CYAN}Recording ${MAP_ODOM_TOPIC} to ${odom_bag}.bag${NC}\n"
    rosbag record -O $odom_bag $MAP_ODOM_TOPIC &>$exp_dir/rosbag_record.txt& 
    
    #running the filter
    python $FILTER_SCRIPT --model ktima &>$exp_dir/filter.txt&

    printf "${CYAN}Running fast-lio background ... ${NC}\n"
    roslaunch fast_lio mapping_ouster16_filter.launch &>$exp_dir/mapping_log.txt& 
    sleep 5   

    bag_dir=$BAGS_REMOTE_DIR/$bag.bag
    ssh ibrahim@$REMOTE_IP "source /opt/ros/melodic/setup.bash && rosbag play ${bag_dir} \
                            --topics $IMU_TOPIC $POINTS_TOPIC -r ${PLAY_RATE} --clock"

    rosnode kill -a # kill all the nodes 
    killall -9 roscore
    killall -9 rosmaster
    sleep 5

    printf "${CYAN}Saving ${MAP_ODOM_TOPIC} as .tum format${NC}\n"
    evo_traj bag $odom_bag.bag $MAP_ODOM_TOPIC --save_as_tum
    mv Odometry.tum $map_odom_dir/$bag.tum
    rm -r $odom_bag.bag

    pcd_map=$pcd_dir/$bag.pcd
    printf "${CYAN}Moving pcd map to ${pcd_map}${NC}\n"
    mv ${FASTLIO_PCD_DIR}/scans.pcd ${pcd_map} #move map to the raw pcd folder 

    printf "======================================================\n"

done

