#!/usr/bin/env python
import rospy
import json
from simple_pid import PID

from std_msgs.msg import Float64
import sailbot_msg.msg as msg
import geopy.distance
from utilities import headingToBearingDegrees, PORT_RENFREW_LATLON

# Constants
GPS_PUBLISH_PERIOD_SECONDS = 0.1  # Keep below 1.0 for smoother boat motion
BOAT_SPEED_KMPH = 14.4  # Boat should move at about 4m/s = 14.4 km/h
INITIAL_HEADING_DEGREES = 180


class MOCK_ControllerAndSailbot:
    def __init__(self, lat, lon, headingDegrees, speedKmph):
        self.lat = lat
        self.lon = lon
        self.speedup = 1.0
        self.headingDegrees = headingDegrees
        self.speedKmph = speedKmph
        self.publishPeriodSeconds = GPS_PUBLISH_PERIOD_SECONDS

        self.headingPID = PID(1.0, 0, 0, setpoint=self.headingDegrees)  # TODO: Change magic numbers
        self.headingPID.output_limits = (-100, 100)

        rospy.init_node('MOCK_Sailbot_Listener', anonymous=True)
        self.publisher = rospy.Publisher("GPS", msg.GPS, queue_size=4)
        rospy.Subscriber("desiredHeading", msg.heading, self.desiredHeadingCallback)
        rospy.Subscriber("changeGPS", msg.GPS, self.changeGPSCallback)
        rospy.Subscriber('speedup', Float64, self.speedupCallback)

    def move(self):
        def findHeadingRepresentationClosestTo(headingDegrees, closeToThisDegrees):
            headingRepresentations = [headingDegrees, headingDegrees + 360, headingDegrees - 360]
            distances = [abs(heading - closeToThisDegrees) for heading in headingRepresentations]
            indexOfMinDistance = distances.index(min(distances))
            return headingRepresentations[indexOfMinDistance]

        self.headingDegrees = findHeadingRepresentationClosestTo(self.headingDegrees, self.headingPID.setpoint)
        turningFactor = self.headingPID(self.headingDegrees)
        self.headingDegrees += turningFactor * self.publishPeriodSeconds  # Need speedup factor?

        # Travel greater distances with speedup
        kmTraveledPerPeriod = self.speedKmph * self.publishPeriodSeconds / 3600.0 * self.speedup
        distanceTraveled = geopy.distance.distance(kilometers=kmTraveledPerPeriod)
        destination = distanceTraveled.destination(point=(self.lat, self.lon),
                                                   bearing=headingToBearingDegrees(self.headingDegrees))

        self.lon = destination.longitude
        self.lat = destination.latitude

    def desiredHeadingCallback(self, data):
        rospy.loginfo(data)
        self.headingPID.setpoint = data.headingDegrees

    def changeGPSCallback(self, data):
        rospy.loginfo("Received change GPS message = {}".format(data))
        self.lat = data.lat
        self.lon = data.lon
        self.headingDegrees = data.headingDegrees
        self.speedKmph = data.speedKmph

    def speedupCallback(self, data):
        self.speedup = data.data

    def publish(self):
        data = msg.GPS()
        data.lat = self.lat
        data.lon = self.lon
        data.headingDegrees = self.headingDegrees
        data.speedKmph = self.speedKmph
        self.publisher.publish(data)


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
        MOCK_ctrl_sailbot = MOCK_ControllerAndSailbot(PORT_RENFREW_LATLON.lat, PORT_RENFREW_LATLON.lon,
                                                      INITIAL_HEADING_DEGREES, BOAT_SPEED_KMPH)

    r = rospy.Rate(float(1) / MOCK_ctrl_sailbot.publishPeriodSeconds)

    while not rospy.is_shutdown():
        MOCK_ctrl_sailbot.move()
        MOCK_ctrl_sailbot.publish()
        r.sleep()
