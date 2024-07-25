#!/usr/bin/env python

import os
import sys
import rospy
import signal
import subprocess
import time

def signal_handler(sig, frame):
    rospy.loginfo("Signal received, shutting down...")
    sys.exit(0)

def list_bag_files(directory):
    return sorted([f for f in os.listdir(directory) if f.endswith('.bag')])

def kill_node(node_name):
    try:
        subprocess.call(['rosnode', 'kill', node_name])
        rospy.loginfo(f"Successfully killed node: {node_name}")
        time.sleep(2)  # Give some time for the node to be terminated
    except Exception as e:
        rospy.logerr(f"Failed to kill node {node_name}: {e}")

def main():
    rospy.init_node('select_and_launch', anonymous=True)

    # Set up signal handlers for shutdown (fix for unresponsive Ctrl-C)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

     # Launch the hardware node
    with open("/tmp/roslaunch_kortex_hardware.log", "w") as log_file:
        hardware_launch = subprocess.Popen(
            ["roslaunch", "kortex_hardware", "gen3_hardware.launch", "ip_address:=192.168.1.10", "dof:=7"],
            stdout=log_file,
            stderr=log_file
        )
    
    # Wait for a moment to ensure the hardware node starts properly
    time.sleep(1)

    
    while not rospy.is_shutdown():
        # Directory containing the bag files
        bags_directory = '/home/sharer/hw_test_ws/src/rec_rep/bags'
        bag_files = list_bag_files(bags_directory)

        print("Available bag files:")
        for idx, bag_file in enumerate(bag_files):
            print(f"{idx}: {bag_file}")

        bag_index = int(input("Select a bag file index: "))
        bag_path = f"{bags_directory}/{bag_files[bag_index]}"

        # Get the mode from command line arguments
        if len(sys.argv) < 2:
            print("Usage: select_and_launch.py <mode>")
            sys.exit(1)

        mode = sys.argv[1]

        # Launch the bag_to_point.py script with the selected bag file
        os.system(f"rosrun rec_rep bag_to_point.py {mode} {bag_path}")
      

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

