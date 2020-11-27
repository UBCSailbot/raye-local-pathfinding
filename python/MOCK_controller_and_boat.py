#!/usr/bin/env python
import rospy
import random
import json

from std_msgs.msg import Float64
import sailbot_msg.msg as msg
import geopy.distance
from utilities import headingToBearingDegrees, PORT_RENFREW_LATLON

# Constants
GPS_PUBLISH_PERIOD_SECONDS = 0.1  # Keep below 1.0 for smoother boat motion
BOAT_SPEED_KMPH = 14.4  # Boat should move at about 4m/s = 14.4 km/h
TURNING_GAIN = 0.25  # Proportional term in the heading controller
SPEED_CHANGE_GAIN = 0.05  # The same, but for the speed

# Bring this value to about 1.0 for limited deviation. 20.0 for quite large deviations.
HEADING_DEGREES_STANDARD_DEVIATION = 1.0

# TODO: Move this to utilities
# Smallest signed angle
def ssa(angle):
    return ((angle + 180) % 360) - 180



class MOCK_ControllerAndSailbot:
    def __init__(self, lat, lon, headingDegrees, speedKmph):
        self.lat = lat
        self.lon = lon
        self.speedup = 1.0
        self.headingDegrees = headingDegrees
        self.headingSetpoint = headingDegrees
        self.speedKmph = speedKmph
        self.speedSetpoint = speedKmph
        self.publishPeriodSeconds = GPS_PUBLISH_PERIOD_SECONDS

        rospy.init_node('MOCK_Sailbot_Listener', anonymous=True)
        self.publisher = rospy.Publisher("GPS", msg.GPS, queue_size=4)
        rospy.Subscriber("desiredHeading", msg.heading, self.desiredHeadingCallback)
        rospy.Subscriber("changeGPS", msg.GPS, self.changeGPSCallback)
        rospy.Subscriber('speedup', Float64, self.speedupCallback)

    def move(self):
        # Travel greater distances with speedup
        kmTraveledPerPeriod = self.speedKmph * self.publishPeriodSeconds / 3600.0 * self.speedup
        distanceTraveled = geopy.distance.distance(kilometers=kmTraveledPerPeriod)
        destination = distanceTraveled.destination(point=(self.lat, self.lon),
                                                   bearing=headingToBearingDegrees(self.headingDegrees))

        self.lon = destination.longitude
        self.lat = destination.latitude

        # Heading
        error = ssa(self.headingSetpoint - self.headingDegrees)
        gain = min(1, TURNING_GAIN * self.publishPeriodSeconds * self.speedup)
        self.headingDegrees = (self.headingDegrees + gain * error) % 360

        # Speed
        error = self.speedSetpoint - self.speedKmph
        gain = min(1, SPEED_CHANGE_GAIN * self.publishPeriodSeconds * self.speedup)
        self.speedKmph += gain * error

    def desiredHeadingCallback(self, data):
        rospy.loginfo(data)
        self.headingSetpoint = data.headingDegrees
        error_mag = abs(ssa(self.headingSetpoint - self.headingDegrees))
        # Anything >= 135 has maximum effect (speed is divided by 3)
        # Anything <= 45 has no effect on speed
        self.speedKmph = self.speedKmph / max(1, min(3, error_mag / 45))

    def changeGPSCallback(self, data):
        rospy.loginfo("Received change GPS message = {}".format(data))
        self.lat = data.lat
        self.lon = data.lon
        self.headingDegrees = data.headingDegrees
        self.speedKmph = data.speedKmph

    def speedupCallback(self, data):
        self.speedup = data.data


if __name__ == '__main__':
    # Get gps_file  parameter
    gps_file = rospy.get_param('gps_file', default=None)

    if gps_file:
        with open(gps_file) as f:
            record = json.loads(f.read())
            lat = record[0]
            lon = record[1]
            MOCK_ctrl_sailbot = MOCK_ControllerAndSailbot(*record)
    else:
        # Boat should move at about 4m/s = 14.4 km/h
        MOCK_ctrl_sailbot = MOCK_ControllerAndSailbot(PORT_RENFREW_LATLON.lat, PORT_RENFREW_LATLON.lon, 180, 14.4)

    r = rospy.Rate(float(1) / MOCK_ctrl_sailbot.publishPeriodSeconds)

    while not rospy.is_shutdown():
        MOCK_ctrl_sailbot.move()
        data = msg.GPS()
        data.lat = MOCK_ctrl_sailbot.lat
        data.lon = MOCK_ctrl_sailbot.lon
        data.headingDegrees = MOCK_ctrl_sailbot.headingDegrees
        data.speedKmph = MOCK_ctrl_sailbot.speedKmph
        MOCK_ctrl_sailbot.publisher.publish(data)
        r.sleep()
