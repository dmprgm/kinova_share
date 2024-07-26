#!/bin/bash

# redirect all output to a log file for debugging purposes
exec &> /home/sharer/hw_test_ws/src/rec_rep/scripts/run_srgui.log
set -x

# source ROS environment setup script
source /opt/ros/noetic/setup.bash
source /home/sharer/hw_test_ws/devel/setup.bash

# run python script
python3 /home/sharer/hw_test_ws/src/rec_rep/scripts/start_recording.py
