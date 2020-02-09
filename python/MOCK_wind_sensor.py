#!/usr/bin/env python
import rospy
import math

from local_pathfinding.msg import wind

def talker():
    pub = rospy.Publisher('MOCK_wind', wind, queue_size=4)
    rospy.init_node('MOCK_wind_talker', anonymous=True)
    r = rospy.Rate(1) #0.1 HZ
    msg = wind()
    msg.direction = math.radians(90)
    msg.speed = 10.0

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
