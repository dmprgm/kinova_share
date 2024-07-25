import rospy
import rosbag
import os

def main():
    rospy.init_node('troubleshooter')

    bag_path = rospy.get_param('~bag_path', '/rec_rep/bags/posi_tracker.bag')
    rospy.loginfo(f"Using bag file path: {bag_path}")

    # Print the current working directory
    cwd = os.getcwd()
    rospy.loginfo(f"Current working directory: {cwd}")

    # Verify the absolute path
    absolute_bag_path = os.path.abspath(bag_path)
    rospy.loginfo(f"Absolute bag file path: {absolute_bag_path}")

    # Check if the file exists
    if os.path.exists(absolute_bag_path):
        rospy.loginfo(f"File {absolute_bag_path} exists and is accessible.")
    else:
        rospy.logerr(f"File {absolute_bag_path} does not exist or is not accessible.")

    # Rest of your code...

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

