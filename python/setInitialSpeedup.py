#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64

MIN_NUM_SPEEDUP_SUBSCRIBERS_BEFORE_STARTING = 5

if __name__ == "__main__":
    rospy.init_node('setInitialSpeedup', anonymous=True)
    initial_speedup = rospy.get_param('initial_speedup', default=1.0)
    speedupPublisher = rospy.Publisher('speedup', Float64, queue_size=4)

    # Wait for other nodes before publishing
    while speedupPublisher.get_num_connections() < MIN_NUM_SPEEDUP_SUBSCRIBERS_BEFORE_STARTING:
        rospy.loginfo("Waiting for at least {} speedup subscribers before publishing initial speedup".format(
            MIN_NUM_SPEEDUP_SUBSCRIBERS_BEFORE_STARTING))
        time.sleep(0.1)
    rospy.loginfo("{} speedup subscribers found. Publishing initial speedup = {}".format(
        speedupPublisher.get_num_connections(), initial_speedup))

    speedupPublisher.publish(initial_speedup)
