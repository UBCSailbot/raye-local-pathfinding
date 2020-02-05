#!/usr/bin/env python
import rospy
import math

import local_pathfinding.msg as msg

def create_path(init, goal):
    path = []

    # Insert the initial position
    init_wp = msg.latlon()
    init_wp.lat = init[0]
    init_wp.lon = init[1]
    path.append(init_wp)

    # Just do some linear interpolation
    num_global_waypoints = 25
    for i in range(1, num_global_waypoints):
        coeff = float(i)/(num_global_waypoints)
        lat = (1 - coeff)*init[0] + coeff*goal[0]
        lon = (1 - coeff)*init[1] + coeff*goal[1]
        print(lat, lon)
        wp = msg.latlon()
        wp.lat = lat
        wp.lon = lon
        path.append(wp)

    # Insert the goal
    last_wp = msg.latlon()
    last_wp.lat = goal[0]
    last_wp.lon = goal[1]
    path.append(last_wp)
    return path

def MOCK_global():
    init = [48.5, -124.0] # Port Renfrew (ish)
    goal = [20.0, -156.0] # Maui (ish)
    path = create_path(init, goal)

    rospy.init_node('MOCK_global_planner', anonymous=True)
    pub = rospy.Publisher("MOCK_global_path", msg.path)
    publish_period = 60 # Seconds. TODO: set this rate
    r = rospy.Rate(float(1) / publish_period)

    while not rospy.is_shutdown():
        pub.publish(msg.path(path))
        r.sleep()

if __name__ == '__main__':
    MOCK_global()
