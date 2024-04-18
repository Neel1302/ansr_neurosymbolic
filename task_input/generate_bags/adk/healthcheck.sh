#!/bin/bash
source ~/AirSimExtensions/ros2/install/setup.bash
[ $(ros2 node list | grep /adk_node | wc -l) -gt 0 ]