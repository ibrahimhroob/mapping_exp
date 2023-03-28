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

# Source the ROS environment setup file
source $CATKIN_WS/devel/setup.bash

# Define a function for error messages
function error() {
  printf "${RED}Error: ${1}${NC}\n"
  exit 1
}

# Check that the necessary dependencies are installed
command -v rosbag >/dev/null 2>&1 || error "rosbag command not found"
command -v roslaunch >/dev/null 2>&1 || error "roslaunch command not found"
command -v python >/dev/null 2>&1 || error "python command not found"

# Define an array of bags to process
bags=("june_1" "june_8" "june_29" "july_13")

for bag in "${bags[@]}"; do
    printf "Mapping: ${BLUE}${bag}${NC} ...\n"
    exp_dir=$bag
    log_dir=$exp_dir/log
    pcd_dir=$exp_dir/pcd
    map_odom_dir=$exp_dir/map_odom

    # Kill all ROS nodes
    printf "${CYAN}Killing all ROS nodes${NC}\n"
    rosnode kill -a >/dev/null 2>&1 || true # Ignore errors if there are no nodes to kill

    mkdir -p $log_dir
    mkdir -p $pcd_dir
    mkdir -p $map_odom_dir 

    odom_bag=$map_odom_dir/$bag
    printf "${CYAN}Recording ${MAP_ODOM_TOPIC} to ${odom_bag}.bag${NC}\n"
    rosbag record -O $odom_bag $MAP_ODOM_TOPIC &>$log_dir/rosbag_record.txt &
    
    # Run the filter in the background
    printf "${CYAN}Running filter ...${NC}\n"
    nohup python $FILTER_SCRIPT --model ktima &>$log_dir/filter.txt &
    sleep 5
    
    # Run fast_lio in the background
    printf "${CYAN}Running fast-lio background ... ${NC}\n"
    nohup roslaunch fast_lio mapping_ouster16_filter.launch &>$log_dir/mapping_log.txt & 
    sleep 5   

    bag_path=$BAGS_REMOTE_DIR/$bag.bag

    # Check if the bag file exists on the remote machine
    if ! ssh ibrahim@$REMOTE_IP "[ -f ${bag_path} ]"; then
        error "Bag file not found on remote machine: ${bag_path}"
    fi


    printf "${CYAN}Playing ${bag_path} on remote machine${NC}\n"
    ssh ibrahim@$REMOTE_IP "source /opt/ros/melodic/setup.bash && rosbag play ${bag_path} \
                            --topics $IMU_TOPIC $POINTS_TOPIC -r ${PLAY_RATE} --clock     \
                            --duration=640 " || error "Failed to play rosbag on remote machine"

    # Kill all ROS nodes
    printf "${CYAN}Killing all ROS nodes${NC}\n"
    rosnode kill -a >/dev/null 2>&1 || true # Ignore errors if there are no nodes to kill

    # Kill all background processes
    printf "${CYAN}Killing all background processes${NC}\n"
    kill $(jobs -p) >/dev/null 2>&1 || true # Ignore errors if there are no background processes to kill
    sleep 5

    # Convert the odometry topic to .tum format
    printf "${CYAN}Converting ${MAP_ODOM_TOPIC} to .tum format${NC}\n"
    evo_traj bag $odom_bag.bag $MAP_ODOM_TOPIC --save_as_tum || error "Failed to convert odometry topic to .tum format"
    mv Odometry.tum $map_odom_dir/$bag.tum
    rm -f $odom_bag.bag

    # Move the PCD map to the raw PCD folder
    pcd_map=$pcd_dir/$bag.pcd
    printf "${CYAN}Moving PCD map to ${pcd_map}${NC}\n"
    mv ${FASTLIO_PCD_DIR}/scans.pcd ${pcd_map} || error "Failed to move PCD map to ${pcd_map}"

    printf "======================================================\n"

done
