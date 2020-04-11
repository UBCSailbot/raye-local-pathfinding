#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64

if __name__ == "__main__":
    # Create sailbot ROS object that subscribes to relevant topics
    sailbot = Sailbot(nodeName='testing')

    while not rospy.is_shutdown():
        print("HI")
