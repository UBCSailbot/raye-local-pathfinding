#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64

MIN_NUM_SPEEDUP_SUBSCRIBERS_BEFORE_STARTING = 6

if __name__ == "__main__":
    rospy.init_node('setInitialSpeedup', anonymous=True)
    initial_speedup = rospy.get_param('initial_speedup', default=1.0)
    speedupPublisher = rospy.Publisher('speedup', Float64, queue_size=4)

    # Wait for other nodes before publishing
    numConnections = speedupPublisher.get_num_connections()
    while numConnections < MIN_NUM_SPEEDUP_SUBSCRIBERS_BEFORE_STARTING:
        rospy.loginfo("{} speedup subscribers found. Waiting for at least {} speedup subscribers "
                      "before publishing initial speedup"
                      .format(numConnections, MIN_NUM_SPEEDUP_SUBSCRIBERS_BEFORE_STARTING))
        time.sleep(0.1)

        # Calculate number of connections at the end of each loop
        numConnections = speedupPublisher.get_num_connections()

    rospy.loginfo("{} speedup subscribers found, which is greater than or equal to the minimum required {}. "
                  "Publishing initial speedup of {} now."
                  .format(numConnections, MIN_NUM_SPEEDUP_SUBSCRIBERS_BEFORE_STARTING, initial_speedup))
    speedupPublisher.publish(initial_speedup)
