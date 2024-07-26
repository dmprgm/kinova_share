#!/usr/bin/env python
import tkinter as tk
from tkinter import filedialog, messagebox
import os # for user input
import sys
import rospy
import rosbag
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController
from kortex_hardware.srv import ModeService
import signal
import subprocess
import time
import glob # for user input

def signal_handler(sig, frame):
	rospy.loginfo("Signal received, shutting down...")
	sys.exit(0)

def list_bag_files(directory):
    return sorted([f for f in os.listdir(directory) if f.endswith('.bag')])

def kill_node(node_name):
    try:
        subprocess.call(['rosnode', 'kill', node_name])
        rospy.loginfo(f"Successfully killed node: {node_name}")
        time.sleep(1)  # Give some time for the node to be terminated
    except Exception as e:
        rospy.logerr(f"Failed to kill node {node_name}: {e}")
        
def run_gui():
    def start_processing():
        mode = mode_var.get()
        selected_index = bag_file_listbox.curselection()

        if mode not in ["position", "velocity", "effort"]:
            messagebox.showerror("Invalid Mode", "Please select a valid mode: 'position', 'velocity', or 'effort'.")
            return

        if not selected_index:
            messagebox.showerror("No Bag File", "Please select a bag file.")
            return

        bag_file = bag_files[selected_index[0]]
        bag_file = os.path.join('/home/sharer/hw_test_ws/src/rec_rep/bags', bag_file)

        print(f"Selected bag file: {bag_file}")  # Debug print statement


        root.destroy()  # Close the GUI

        # Call the main function with the selected mode and bag file
        main(mode, bag_file)

    root = tk.Tk()
    root.title("Motion Replay")

    tk.Label(root, text="Select Mode:").grid(row=0, column=0, padx=10, pady=10)
    mode_var = tk.StringVar(value="velocity")  # Default to velocity
    tk.OptionMenu(root, mode_var, "position", "velocity", "effort").grid(row=0, column=1, padx=10, pady=10)
    
    tk.Label(root, text="Select Recording:").grid(row=1, column=0, padx=10, pady=10)
    bag_files = list_bag_files('/home/sharer/hw_test_ws/src/rec_rep/bags')
    bag_file_listbox = tk.Listbox(root, selectmode=tk.SINGLE, width=40, height=10)
    for i, bag_file in enumerate(bag_files):
        bag_file_listbox.insert(tk.END, f"{i+1}. {bag_file}")
    bag_file_listbox.grid(row=1, column=1, padx=10, pady=10)

    tk.Button(root, text="Start", command=start_processing).grid(row=2, column=0, columnspan=2, pady=20)

    root.mainloop()

def main(mode, bag_file):
    # Launch the hardware node
    print("hi i'm gonna try to launch the kortex_hardware node wml") #reword this eventually
    with open("/tmp/roslaunch_kortex_hardware.log", "w") as log_file:
        hardware_launch = subprocess.Popen(
            ["roslaunch", "kortex_hardware", "gen3_hardware.launch", "ip_address:=192.168.1.10", "dof:=7"],
            
            stdout=log_file,
            stderr=log_file
        )
    #give node time to start up
    print("just gonna wait here a second. or five.")
    time.sleep(3) # wait time - MUST be at or above 2
    #check if nodes are active
    print("i'm checking if nodes are active!")
    try:
        active_nodes = subprocess.check_output(["rosnode", "list"]).decode("utf-8")
        if "/kortex_hardware" in active_nodes:
            print("Kortex hardware is up and running.")
        else:
            print("Kortex hardware is not running. Check /tmp/roslaunch_kortex_hardware.log for details.")
            sys.exit(1)
    except subprocess.CalledProcessError as e:
        print(f"Error checking active nodes: {e}")
        sys.exit(1)

    topic = ""
    if mode == "position":
        topic = "/position_controller/command"
    elif mode == "velocity":
        topic = "/velocity_controller/command"
    elif mode == "effort":
        topic = "/effort_controller/command"
    else:
        print("Invalid mode. Use 'position', 'velocity', or 'effort'.")
        sys.exit(1)

    rospy.loginfo("Starting bag_to_point node")

    # Wait for necessary services
    rospy.wait_for_service('controller_manager/switch_controller')
    rospy.wait_for_service('set_control_mode')
    switch_controller = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
    mode_change = rospy.ServiceProxy('set_control_mode', ModeService)
    try:
        if mode == "position":
            resp = switch_controller(['position_controller'], ['velocity_controller', 'effort_controller'], 1, 0, 5)
        elif mode == "effort":
            resp = switch_controller(['effort_controller'], ['velocity_controller', 'position_controller'], 1, 0, 5)
        elif mode == "velocity":
            resp = switch_controller(['velocity_controller'], ['position_controller', 'effort_controller'], 1, 0, 5)
        rospy.loginfo("Successfully switched controller to %s mode", mode)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

    try:
        resp1 = mode_change(mode)
        rospy.loginfo("Mode change response: %s", resp1)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

    if mode == "stop":
        sys.exit(0)
        
    # kortex hardware should be up and running at this point. moving on
    rospy.init_node('bag_to_point')

    # Set up signal handlers for shutdown (fix for unresponsive Ctrl-C)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Publishers for position and velocity controllers
    if mode == "position":
        position_pub = rospy.Publisher(topic, Float64MultiArray, queue_size=10)
    elif mode == "velocity":
        velocity_pub = rospy.Publisher(topic, JointTrajectory, queue_size=10)

    try:
        # Open the selected rosbag
        bag = rosbag.Bag(bag_file)
        rospy.loginfo(f"Opened bag file: {bag_file}")
        
        # Print the current working directory for debugging
        print(f"Current working directory: {os.getcwd()}")

        for topic, msg, t in bag.read_messages(topics=['/joint_states']):
            if topic == '/joint_states':
                if mode == "position":
                    # Create Float64MultiArray message for position
                    position_msg = Float64MultiArray()
                    position_msg.data = list(msg.position[1:8])
                    position_pub.publish(position_msg)
                    rospy.loginfo(f"Published position message with positions {msg.position}")
                elif mode == "velocity":
                    # Create JointTrajectory message for velocity
                    trajectory_msg = JointTrajectory()
                    trajectory_msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
                    point = JointTrajectoryPoint()
                    point.positions = list(msg.position[1:8])
                    point.velocities = list(msg.velocity[1:8])
                    point.time_from_start = rospy.Duration(0.005)  # Adjust as needed
                    trajectory_msg.points = [point]
                    velocity_pub.publish(trajectory_msg)
                    rospy.loginfo(f"Published velocity message with positions {msg.position} and velocities {msg.velocity}")
                # Sleep to simulate real-time publishing
                rospy.sleep(0.0005)  # Adjust the sleep time as needed

        bag.close()
        rospy.logerr("Finished processing bag file.")
    except rospy.ROSException as e:
        rospy.logerr(f"Error processing the bag file: {e}")

    kill_node("kortex_hardware")
    kill_node("compliant_controller_spawner_started")
    kill_node("controller_spawner_started")
    kill_node("controller_spawner_stopped")

if __name__ == '__main__':
    run_gui()

