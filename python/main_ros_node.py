#!/usr/bin/env python
import rospy
from local_pathfinding.msg import wind


def callback(data):
    rospy.loginfo(data)

def listener():
    rospy.init_node('local_pathfinder', anonymous=True)
    rospy.Subscriber("MOCK_wind", AIS, callback)
    pub = rospy.Publisher("heading", float)
    r = rospy.Rate(1) #1hz

    while not rospy.is_shutdown():
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    listener()
