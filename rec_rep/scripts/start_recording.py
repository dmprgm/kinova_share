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
from trajectory_msgs.msg import JointTrajectoryPoint
from PyQt5.QtWidgets import (QApplication, QWidget, QLabel, QLineEdit, QPushButton,
                             QVBoxLayout, QHBoxLayout, QRadioButton, QButtonGroup, QMessageBox)

rosbag_proc = None
cmd_pub = None
controller_space = None
tfBuffer = None

def signal_handler(sig, frame):
    rospy.loginfo("Signal received, shutting down...")
    stop_recording()
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

def update_position(msg):
    tmp = np.array(msg.position)[1:]
    cmd = JointTrajectoryPoint()
    dof = 7
    value = np.pi/2
    tmp[dof - 1] = value
    cmd.positions = tmp.tolist()
    cmd.velocities = [0] * len(tmp)
    cmd_pub.publish(cmd)
    
def start_recording():
    global rosbag_proc, cmd_pub, controller_space, tfBuffer

    controller_space = 'task' if taskRadio.isChecked() else 'joint'
    
    mode = "effort"
    topic = ""
    if controller_space == "task":
        topic = "/task_space_compliant_controller/command"
    elif controller_space == "joint":
        topic = "/joint_space_compliant_controller/command"

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.wait_for_service("controller_manager/switch_controller")
    rospy.wait_for_service("set_control_mode")
    switch_controller = rospy.ServiceProxy("controller_manager/switch_controller", SwitchController)
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

    try:
        if controller_space == "task":
            cmd_pub = rospy.Publisher(topic, CartesianTrajectoryPoint, queue_size=1)
        else:
            cmd_pub = rospy.Publisher(topic, JointTrajectoryPoint, queue_size=1)
        rospy.Subscriber('joint_states', JointState, update_position)

        bags_directory = '/home/sharer/hw_test_ws/src/rec_rep/bags'
        next_prefix = get_next_bag_prefix(bags_directory)
        suffix = suffixInput.text()
        bag_name = f"{next_prefix}{suffix}.bag"

        rosbag_proc = record_rosbag(bag_name)
        rospy.loginfo("Recording started. Use Ctrl-C to stop recording...")

        QMessageBox.information(window, 'Recording Started', f'Started recording to {bag_name}')

        startButton.setEnabled(False)
        stopButton.setEnabled(True)
        #rospy.spin()
        while not rospy.is_shutdown():
            joint_states = rospy.wait_for_message("/joint_states", JointState, timeout=None)
            tmp = np.array(joint_states.position)[1:]
            print("Current joint state: ", tmp)
            cmd = JointTrajectoryPoint()
		    # take input from user
            dof = 7
            value = 1
            tmp[dof - 1] = value
            cmd.positions = tmp.tolist()
            cmd.velocities = [0] * len(tmp)
            cmd_pub.publish(cmd)
            rospy.sleep(.5)
            
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
        stop_recording()
    except KeyboardInterrupt:
        stop_recording()




def stop_recording():
    global rosbag_proc
    if rosbag_proc:
        rospy.loginfo("Stopping rosbag recording...")
        rosbag_proc.terminate()
        rosbag_proc.wait()
        rospy.loginfo("Rosbag recording stopped.")
        kill_node("kortex_hardware")
        rosbag_proc = None

        QMessageBox.information(window, 'Recording Stopped', 'Recording has been stopped.')

        startButton.setEnabled(True)
        stopButton.setEnabled(False)
        sys.exit(0)

def run_gui():
    global window, startButton, stopButton, jointRadio, taskRadio, suffixInput

    app = QApplication(sys.argv)
    window = QWidget()
    window.setWindowTitle('Record Robot Motion')

    taskRadio = QRadioButton('Task')
    jointRadio = QRadioButton('Joint')
    taskRadio.setChecked(True)
    controllerTypeGroup = QButtonGroup()
    controllerTypeGroup.addButton(taskRadio)
    controllerTypeGroup.addButton(jointRadio)

    hbox1 = QHBoxLayout()
    hbox1.addWidget(taskRadio)
    hbox1.addWidget(jointRadio)

    suffixLabel = QLabel('Bag File Suffix:')
    suffixInput = QLineEdit()

    hbox2 = QHBoxLayout()
    hbox2.addWidget(suffixLabel)
    hbox2.addWidget(suffixInput)

    startButton = QPushButton('Start Recording')
    startButton.clicked.connect(start_recording)
    stopButton = QPushButton('Stop Recording')
    stopButton.clicked.connect(stop_recording)
    stopButton.setEnabled(False)

    hbox3 = QHBoxLayout()
    hbox3.addWidget(startButton)
    hbox3.addWidget(stopButton)

    vbox = QVBoxLayout()
    vbox.addLayout(hbox1)
    vbox.addLayout(hbox2)
    vbox.addLayout(hbox3)

    window.setLayout(vbox)
    window.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    with open("/tmp/roslaunch_gen3_compliant_controllers.log", "w") as log_file:
        hardware_launch = subprocess.Popen(
            ["roslaunch", "gen3_compliant_controllers", "controller.launch", 
             "ip_address:=192.168.1.10", "dof:=7", "controller_type:=joint"], #type will always be joint
            stdout=log_file,
            stderr=log_file
        )
    time.sleep(3)

    print("attempted to start controller")

    rospy.init_node('start_recording')
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    run_gui()
