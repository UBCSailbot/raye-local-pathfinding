#! /usr/bin/env python
import rospy
import time
import sys
from utilities import measuredWindToGlobalWind
from sailbot_msg.msg import GPS, windSensor, globalWind

GLOBAL_WIND_PUSH_PERIOD_SECONDS = 1.0


class GlobalWind:
    def __init__(self):
        self.isFirstRun = True

        self.measuredWindSpeed = None
        self.measuredWindDirection = None
        self.boatSpeed = None
        self.headingDegrees = None

        rospy.init_node('global_wind', anonymous=True)
        rospy.Subscriber("GPS", GPS, self.GPSCallback)
        rospy.Subscriber("windSensor", windSensor, self.windSensorCallback)

    def GPSCallback(self, data):
        self.headingDegrees = data.headingDegrees
        self.boatSpeed = data.speedKmph

    def windSensorCallback(self, data):
        self.measuredWindDirection = data.measuredDirectionDegrees
        self.measuredWindSpeed = data.measuredSpeedKmph

    def measuredWindToGlobalWind(self):
        '''Calculate the global wind based on the measured wind and the boat velocity

        Returns:
           float, float pair representing the globalWindSpeed (same units as input speed), globalWindDirectionDegrees
        '''
        if self.isFirstRun:
            self.waitForFirstSensorData()
            self.isFirstRun = False

        return measuredWindToGlobalWind(measuredWindSpeed=self.measuredWindSpeed,
                                        measuredWindDirectionDegrees=self.measuredWindDirection,
                                        boatSpeed=self.boatSpeed,
                                        headingDegrees=self.headingDegrees)

    def waitForFirstSensorData(self):
        """Waits until first sensor data have been received by subscribers"""
        while self.isMissingFirstSensorData():
            # Exit if shutdown
            if rospy.is_shutdown():
                rospy.loginfo("rospy.is_shutdown() is True. Exiting")
                sys.exit()
            else:
                rospy.loginfo("Waiting for sailbot to receive first sensor data")
                rospy.loginfo("self.measuredWindSpeed is None? {}. self.measuredWindDirection is None? {}. "
                              "self.boatSpeed is None? {}. self.headingDegrees is None? {}. "
                              .format(self.measuredWindSpeed is None, self.measuredWindDirection is None,
                                      self.boatSpeed is None, self.headingDegrees is None))
                time.sleep(1)

        rospy.loginfo("First sensor data received")

    def isMissingFirstSensorData(self):
        """Checks if the subscribers have all received their first messages

        Returns:
           bool True iff the subscribers have all received their first messages
        """
        return (self.measuredWindSpeed is None or self.measuredWindDirection is None or
                self.boatSpeed is None or self.headingDegrees is None)


if __name__ == '__main__':
    global_wind_conversion = GlobalWind()
    global_wind_pub = rospy.Publisher('global_wind', globalWind, queue_size=4)
    rate = rospy.Rate(1.0 / GLOBAL_WIND_PUSH_PERIOD_SECONDS)

    while not rospy.is_shutdown():
        speed, direction = global_wind_conversion.measuredWindToGlobalWind()
        global_wind_pub.publish(globalWind(directionDegrees=direction, speedKmph=speed))
        rate.sleep()
