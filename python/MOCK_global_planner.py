#!/usr/bin/env python
import rospy
import math

from local_pathfinding.msg import latlon, global_path

def create_path(init, goal):
    path = []

    # Insert the initial position
    init_wp = latlon()
    init_wp.lat = init[0]
    init_wp.lon = init[1]
    path.append(init_wp)

    dist = int(math.sqrt((init[0] - goal[0])**2 + (init[1] - goal[1])**2))

    # Just do some linear interpolation
    for i in range(1, 10*dist):
        coeff = float(i)/(10*dist)
        lat = (1 - coeff)*init[0] + coeff*goal[0]
        lon = (1 - coeff)*init[1] + coeff*goal[1]
        print(lat, lon)
        wp = latlon()
        wp.lat = lat
        wp.lon = lon
        path.append(wp)

    # Insert the goal
    last_wp = latlon()
    last_wp.lat = goal[0]
    last_wp.lon = goal[1]
    path.append(last_wp)
    return path

def MOCK_global():
    init = [48.5, -124.0] # Port Renfrew (ish)
    goal = [20.0, -156.0] # Maui (ish)
    path = create_path(init, goal)

    rospy.init_node('MOCK_global_planner', anonymous=True)
    pub = rospy.Publisher("MOCK_global_path", global_path)
    r = rospy.Rate(1.0) # TODO: set this rate

    while not rospy.is_shutdown():
        pub.publish(global_path(path))
        r.sleep()

if __name__ == '__main__':
    MOCK_global()
