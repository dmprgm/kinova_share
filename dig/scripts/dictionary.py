#! /usr/bin/env python

import os
import rospy
import rosbag
import sensor_msgs.msg
# from sensor_msgs.msg import JointState
import signal
import numpy as np
import csv


def compile_joint_states(directory):
    all_joint_states = []
    for file in os.listdir(directory):
        if file.endswith('.bag'):
            bag_file = os.path.join(directory, file)
            bag = rosbag.Bag(bag_file)
            for topic, msg, t in bag.read_messages(topics=['/joint_states']):
                all_joint_states.append((t.to_sec(), msg.position, msg.velocity, msg.effort))
            bag.close()
    return all_joint_states


def compute_attributes(joint_states):
    times = np.array([state[0] for state in joint_states])
    positions = np.array([state[1] for state in joint_states])
    velocities = np.array([state[2] for state in joint_states])

    # Compute velocity if not provided
    if np.isnan(velocities).any():
        velocities = np.gradient(positions, times, axis=0)
    return times, positions, velocities


def save_to_csv(filename, times, positions, velocities):
    with open(filename, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(['Time', 'Position', 'Velocity'])
        for t, pos, vel in zip(times, positions, velocities):
            csvwriter.writerow([t, pos, vel])


def main(directory, output_csv):
    all_joint_states = compile_joint_states(directory)
    times, positions, velocities = compute_attributes(all_joint_states)
    save_to_csv(output_csv, times, positions, velocities)
    print(f"Data saved to {output_csv}")
    

if __name__ == "__main__":
    directory = '/home/sharer/hw_test_ws/src/kinova_share/rec_rep/bags'
    output_csv = 'joint_states_data.csv'
    main(directory, output_csv)
