#!/usr/bin/env python
import rospy
import math
import random

import local_pathfinding.msg as msg

class MOCK_ControllerAndSailbot: 
    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon
        self.heading = math.radians(180)
        self.speed = 0

        rospy.init_node('Sailbot_Listener', anonymous=True)
        self.publisher = rospy.Publisher("MOCK_GPS", msg.GPS)
        rospy.Subscriber("MOCK_desired_heading", msg.desired_heading, self.desiredHeadingCallback)

    def move(self):
        # TODO: Change this so that the boat
        # moves at a reasonable speed
        speed_coeff = 0.001
        
        dx = math.cos(self.heading)*speed_coeff
        dy = math.sin(self.heading)*speed_coeff

        self.lon = self.lon + dx
        self.lat = self.lat + dy

        # TODO: translate to m/s
        self.speed = math.sqrt(dx**2 + dy**2)

    def desiredHeadingCallback(self, data):
        rospy.loginfo(data)
        self.heading = data.heading + random.gauss(0, 0.1)

if __name__ == '__main__':
    MOCK_ctrl_sailbot = MOCK_ControllerAndSailbot(48.5, -124.8)
    r = rospy.Rate(1) #hz

    while not rospy.is_shutdown():
        MOCK_ctrl_sailbot.move()
        data = msg.GPS()
        data.lat = MOCK_ctrl_sailbot.lat
        data.lon = MOCK_ctrl_sailbot.lon
        data.heading = MOCK_ctrl_sailbot.heading
        data.speed = MOCK_ctrl_sailbot.speed
        MOCK_ctrl_sailbot.publisher.publish(data)
        r.sleep()
