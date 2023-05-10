#!/usr/bin/env python3

import rospy
import subprocess

def main():
    rospy.init_node("node_creator", anonymous=True)
    
    package_name = "husky_control"
    launch_file = "teleop.launch"
    
    rospy.loginfo(f"Creating node: /joy_teleop/joy_node")
    subprocess.Popen(["roslaunch", package_name, launch_file])

    rospy.spin()  # Keep the script running to prevent the node from being terminated

if __name__ == "__main__":
    main()
