#!/usr/bin/env python

import os
import sys
import subprocess
import time
import re
import glob
import rospy
import signal
import numpy as np
import tf2_ros
from controller_manager_msgs.srv import SwitchController
from kortex_hardware.srv import ModeService
from moveit_msgs.msg import CartesianTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def signal_handler(sig, frame):
	rospy.loginfo("Signal received, shutting down...")
	sys.exit(0)

def kill_node(node_name):
    try:
        subprocess.call(['rosnode', 'kill', node_name])
        rospy.loginfo(f"Successfully killed node: {node_name}")
        time.sleep(2)  # Give some time for the node to be terminated
    except Exception as e:
        rospy.logerr(f"Failed to kill node {node_name}: {e}")

def get_next_bag_prefix(bags_directory):
    bag_files = glob.glob(os.path.join(bags_directory, '*.bag'))
    if not bag_files:
        return "000"

    prefix_pattern = re.compile(r'^(\d{3})')
    prefixes = []
    for bag_file in bag_files:
        basename = os.path.basename(bag_file)
        match = prefix_pattern.match(basename)
        if match:
            prefixes.append(int(match.group(1)))

    if not prefixes:
        return "000"

    next_prefix = max(prefixes) + 1
    return f"{next_prefix:03d}"

def record_rosbag(bag_name):
    bags_directory = '/home/sharer/hw_test_ws/src/rec_rep/bags'
    full_bag_path = os.path.join(bags_directory, bag_name)
    return subprocess.Popen(
        ['rosbag', 'record', '-O', full_bag_path, '/joint_states'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

def start_process(command):
    rospy.loginfo(f"Executing command: {command}")
    return subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

def log_process_output(process, process_name):
    while process.poll() is None:
        output = process.stdout.readline()
        if output:
            rospy.loginfo(f"[{process_name}] {output.strip().decode('utf-8')}")
        error = process.stderr.readline()
        if error:
            rospy.logerr(f"[{process_name}] {error.strip().decode('utf-8')}")

if __name__ == "__main__":
    print("hi i'm gonna try to launch the controller.launch file") # reword

    # check command line argument for the control type
    if len(sys.argv) != 2 or sys.argv[1] not in ['joint', 'task']:
        print("Usage: python your_script.py <controller_type>")
        print("<controller_type> must be 'joint' or 'task'")
        sys.exit(1)

    controller_type = sys.argv[1]

    # Launch the gen3_compliant_controllers node with the specified controller type
    with open("/tmp/roslaunch_gen3_compliant_controllers.log", "w") as log_file:
        hardware_launch = subprocess.Popen(
            ["roslaunch", "gen3_compliant_controllers", "controller.launch", 
            "ip_address:=192.168.1.10", "dof:=7", f"controller_type:={controller_type}"],
            
            stdout=log_file,
            stderr=log_file
        )

    # Wait for a moment to ensure the hardware node starts properly
    print("time for a casual breathing exercise")
    time.sleep(2)

    #unlike bag_to_point, this currently has no implementation to check if the node is running.
    print("or not lol")
    rospy.init_node('start_recording')

    # Set up signal handlers for shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # get command line arguments for control mode
    controller_space = sys.argv[1]
    mode = "effort"
    topic = ""
    if controller_space == "task":
        topic = "/task_space_compliant_controller/command"
    elif controller_space == "joint":
        topic = "/joint_space_compliant_controller/command"
    else:
        print("Invalid control mode")
        sys.exit(0)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    print("Switching to", controller_space, "space controller. The robot might jerk a bit as it is switching to effort mode.")
    rospy.wait_for_service("controller_manager/switch_controller")
    rospy.wait_for_service("set_control_mode")
    switch_controller = rospy.ServiceProxy(
        "controller_manager/switch_controller", SwitchController
    )
    mode_change = rospy.ServiceProxy("set_control_mode", ModeService)

    try:
        if controller_space == "task":
            resp = switch_controller(
                ["task_space_compliant_controller"],
                ["velocity_controller", "joint_space_compliant_controller"],
                1,
                0,
                5,
            )
        else:
            resp = switch_controller(
                ["joint_space_compliant_controller"],
                ["velocity_controller", "task_space_compliant_controller"],
                1,
                0,
                5,
            )
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    try:
        resp1 = mode_change(mode)
        print("Mode change response: ", resp1)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    if mode == "stop":
        sys.exit(0)

    print("Switch complete...")
    try:
        cmd_pub = None
        if controller_space == "task":
            cmd_pub = rospy.Publisher(topic, CartesianTrajectoryPoint, queue_size=1)
        else:
            cmd_pub = rospy.Publisher(topic, JointTrajectoryPoint, queue_size=1)
        rate = rospy.Rate(100.0)

        # Setup bag file to record
        bags_directory = '/home/sharer/hw_test_ws/src/rec_rep/bags'
        next_prefix = get_next_bag_prefix(bags_directory)
        print(f"Prefix: {next_prefix}")
        suffix = input(f"Enter the suffix for the bag file (e.g., 'recording'): ")
        bag_name = f"{next_prefix}{suffix}.bag"

        # Start recording
        print(f"Starting rosbag recording to {bag_name}...")
        rosbag_proc = record_rosbag(bag_name)
        rospy.loginfo("Recording started. Use Ctrl-C to stop recording...")

        while not rospy.is_shutdown():
            # wait for message from joint_states
            cmd = None
            if controller_space == "task":
                cmd = CartesianTrajectoryPoint()
                # get current end effector position using tf
                transform = None
                while True:
                    try:
                        transform = tfBuffer.lookup_transform('base_link', 'end_effector_link', rospy.Time())
                        break
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        rate.sleep()
                        continue
                print(
                    "Current end effector position: ", transform.transform.translation
                )
                # use static values for x, y, z
                #cmd.point.pose.position.x = 0.4566529
                #cmd.point.pose.position.y = 0.0013146
                #cmd.point.pose.position.z = 0.4336706
                cmd.point.pose.position.x = float(input("Ctrl-C to end recording... Do NOT type any numbers."))
                cmd.point.pose.position.y = float(input())
                cmd.point.pose.position.z = float(input())
                cmd.point.pose.orientation.x = transform.transform.rotation.x
                cmd.point.pose.orientation.y = transform.transform.rotation.y
                cmd.point.pose.orientation.z = transform.transform.rotation.z
                cmd.point.pose.orientation.w = transform.transform.rotation.w
            else:
                # get current joint state from joint_states
                joint_states = rospy.wait_for_message(
                    "/joint_states", JointState, timeout=None
                )
                tmp = np.array(joint_states.position)[1:]
                print("Current joint state: ", tmp)
                cmd = JointTrajectoryPoint()
                # take input from user
                dof = int(input("Ctrl-C to end recording... Do NOT type any numbers."))
                value = float(input())
                tmp[dof - 1] = value
                cmd.positions = tmp.tolist()
                cmd.velocities = [0] * len(tmp)
            cmd_pub.publish(cmd)
            input("press enter to continue... (DON'T)")

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
        rospy.loginfo("Stopping rosbag recording...")
        rosbag_proc.terminate()
        rosbag_proc.wait()
        rospy.loginfo("Rosbag recording stopped.")
        kill_node("kortex_hardware")
    except KeyboardInterrupt:
        rospy.loginfo("Stopping rosbag recording...")
        rosbag_proc.terminate()
        rosbag_proc.wait()
        rospy.loginfo("Rosbag recording stopped.")
        kill_node("kortex_hardware")
