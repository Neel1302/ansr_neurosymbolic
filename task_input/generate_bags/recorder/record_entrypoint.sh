#!/bin/bash
echo "Using thread $1 with timeout $TIMEOUT"
source /home/performer/dev_ws/install/setup.bash
rm -rf /bag_root/bags/
if [[ $1 == "perception_thread" ]]
then
    # Record perception topics
    timeout $TIMEOUT ros2 bag record --storage=mcap -o /bag_root/bags /adk_node/input/perception	/adk_node/ground_truth/perception /adk_node/SimpleFlight/odom_local_ned
elif [[ $1 == "maneuver_thread" ]]
then
    # Record maneuver topics
    timeout $TIMEOUT ros2 bag record --storage=mcap -o /bag_root/bags /adk_node/SimpleFlight/odom_local_ned /adk_node/ground_truth/perception
else
    echo Requires thread argument!
    return 1
fi