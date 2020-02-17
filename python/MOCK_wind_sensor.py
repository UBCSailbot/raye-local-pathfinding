#!/usr/bin/env python
import rospy
import math

from local_pathfinding.msg import windSensor, GPS

# Global variables for tracking boat speed
boatHeadingDegrees = 0
boatSpeedKmph = 0
def gpsCallback(data):
    global boatHeadingDegrees
    global boatSpeedKmph
    boatHeadingDegrees = data.headingDegrees
    boatSpeedKmph = data.speedKmph

def talker():
    global boatHeadingDegrees
    global boatSpeedKmph

    pub = rospy.Publisher('windSensor', windSensor, queue_size=4)
    rospy.init_node('MOCK_wind_talker', anonymous=True)
    r = rospy.Rate(1) #1 HZ
    msg = windSensor()

    while not rospy.is_shutdown():
        # Set windSensor message such that the global wind is 10km/h in 90 degrees direction
        globalWindSpeedKmph = 10
        globalWindDirectionDegrees = 90

        measuredWindXGlobalFrame = globalWindSpeedKmph * math.cos(math.radians(globalWindDirectionDegrees)) - boatSpeedKmph * math.cos(math.radians(boatHeadingDegrees))
        measuredWindYGlobalFrame = globalWindSpeedKmph * math.sin(math.radians(globalWindDirectionDegrees)) - boatSpeedKmph * math.sin(math.radians(boatHeadingDegrees))
        measuredWindXBoatFrame = measuredWindXGlobalFrame * math.sin(math.radians(boatHeadingDegrees)) - measuredWindYGlobalFrame * math.cos(math.radians(boatHeadingDegrees))
        measuredWindYBoatFrame = measuredWindXGlobalFrame * math.cos(math.radians(boatHeadingDegrees)) + measuredWindYGlobalFrame * math.sin(math.radians(boatHeadingDegrees))

        msg.measuredDirectionDegrees = math.degrees(math.atan2(measuredWindYBoatFrame, measuredWindXBoatFrame))
        msg.measuredSpeedKmph = (measuredWindYBoatFrame**2 + measuredWindXBoatFrame**2)**0.5
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        rospy.Subscriber("GPS", GPS, gpsCallback)
        talker()
    except rospy.ROSInterruptException: pass
