#! /usr/bin/env python

import os
import rospy
import rosbag
import re
import numpy as np
import csv

# processes messages inside the bag file
# starts recording messages after the start idle period
# afterwards, backpedals until motion detected again to remove end idle period
def compile_joint_states(bag_file, position_threshold=0.01, velocity_threshold=0.01): #adjust these thresholds if needed?
    joint_states = []
    bag = rosbag.Bag(bag_file)
    
    prev_positions = None
    prev_velocities = None
    movement_started = False

    for topic, msg, t in bag.read_messages(topics=['/joint_states']):
        current_positions = np.array(msg.position)
        current_velocities = np.array(msg.velocity)
        
        if prev_positions is not None and prev_velocities is not None:
            # Calculate the difference between current and previous positions/velocities
            position_diff = np.abs(current_positions - prev_positions)
            velocity_diff = np.abs(current_velocities - prev_velocities)

            # Check if the movement has started
            if not movement_started:
                if np.any(position_diff > position_threshold) or np.any(velocity_diff > velocity_threshold):
                    movement_started = True
        
        # Start recording data once movement is detected
        if movement_started:
            joint_states.append((t.to_sec(), current_positions, current_velocities, msg.effort))

        # Update previous positions and velocities
        prev_positions = current_positions
        prev_velocities = current_velocities

    bag.close()
    
    # Filter any end idle period
    joint_states = filter_end_idle_period(joint_states, position_threshold, velocity_threshold)
    
    return joint_states


def filter_end_idle_period(joint_states, position_threshold, velocity_threshold):
    if not joint_states:
        return joint_states
    
    # Reverse iterate over joint_states (find when the last movement occurred)
    last_movement_index = len(joint_states) - 1
    for i in reversed(range(1, len(joint_states))):
        current_positions = np.array(joint_states[i][1])
        previous_positions = np.array(joint_states[i-1][1])
        
        current_velocities = np.array(joint_states[i][2])
        previous_velocities = np.array(joint_states[i-1][2])
        
        position_diff = np.abs(current_positions - previous_positions)
        velocity_diff = np.abs(current_velocities - previous_velocities)
        
        # Check if there's a significant movement at this point
        if np.any(position_diff > position_threshold) or np.any(velocity_diff > velocity_threshold):
            last_movement_index = i
            break
    
    # Return the list up to the last movement point
    return joint_states[:last_movement_index + 1]

# pulls time stamps and joint positions
def compute_attributes(joint_states):
    times = np.array([state[0] for state in joint_states])
    positions = np.array([state[1] for state in joint_states])
    velocities = np.array([state[2] for state in joint_states])

    # Compute velocity if not provided
    if np.isnan(velocities).any():
        velocities = np.gradient(positions, times, axis=0)
    return times, positions, velocities

# saves data to a csv file
def save_to_csv(filename, times, positions, velocities):
    with open(filename, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(['Time', 'Position', 'Velocity'])
        for t, pos, vel in zip(times, positions, velocities):
            csvwriter.writerow([t, pos, vel])


def process_bag_files(study_bags_directory, csv_output_directory):
    for subdir in os.listdir(study_bags_directory):
        subdir_path = os.path.join(study_bags_directory, subdir)
        if os.path.isdir(subdir_path):  # Only process directories like P0, P1, etc.
            for file in os.listdir(subdir_path):
                if file.endswith('.bag'):
                    bag_file = os.path.join(subdir_path, file)

                    # Remove any leading numeral prefix
                    cleaned_file = re.sub(r'^\d+', '', os.path.splitext(file)[0])
                    csv_filename = f"{subdir}_{cleaned_file}.csv"
                    output_csv = os.path.join(csv_output_directory, csv_filename)
                    
                    joint_states = compile_joint_states(bag_file)
                    times, positions, velocities = compute_attributes(joint_states)
                    save_to_csv(output_csv, times, positions, velocities)
                    
                    print(f"Data saved to {output_csv}")


if __name__ == "__main__":
    study_bags_directory = '/home/sharer/hw_test_ws/src/rec_rep/study_bags'
    csv_output_directory = '/home/sharer/hw_test_ws/src/rec_rep/study_csv'
    process_bag_files(study_bags_directory, csv_output_directory)
