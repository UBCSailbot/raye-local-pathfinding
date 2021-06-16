#!/usr/bin/env python
import rospy
import json

from std_msgs.msg import Float64
import sailbot_msg.msg as msg
import geopy.distance
from utilities import headingToBearingDegrees, PORT_RENFREW_LATLON, ssa_deg

# Constants
GPS_PUBLISH_PERIOD_SECONDS = 0.1  # Keep below 1.0 for smoother boat motion
BOAT_SPEED_KMPH = 14.4  # Boat should move at about 4m/s = 14.4 km/h
TURNING_GAIN = 0.25  # Proportional term in the heading controller
SPEED_CHANGE_GAIN = 0.05  # The same, but for the speed


class MOCK_ControllerAndSailbot:
    def __init__(self, lat, lon, headingDegrees, speedKmph):
        self.lat = lat
        self.lon = lon
        self.speedup = 1.0
        self.headingDegrees = headingDegrees
        self.headingSetpoint = headingDegrees
        self.speedKmph = speedKmph
        self.speedSetpoint = speedKmph
        self.origSpeedSetpoint = speedKmph
        self.publishPeriodSeconds = GPS_PUBLISH_PERIOD_SECONDS
        self.relativeWindDegrees = None

        rospy.init_node('MOCK_Sailbot_Listener', anonymous=True)
        self.publisher = rospy.Publisher("GPS", msg.GPS, queue_size=4)
        rospy.Subscriber("heading_degrees", msg.heading, self.desiredHeadingCallback)
        rospy.Subscriber("changeGPS", msg.GPS, self.changeGPSCallback)
        rospy.Subscriber('speedup', Float64, self.speedupCallback)
        rospy.Subscriber("windSensor", msg.windSensor, self.windSensorCallback)

    def move(self):
        # Travel based on boat speed
        kmTraveledPerPeriod = self.speedKmph * self.publishPeriodSeconds / 3600.0
        kmTraveledPerPeriod *= self.speedup  # Move greater distances with speedup
        distanceTraveled = geopy.distance.distance(kilometers=kmTraveledPerPeriod)
        destination = distanceTraveled.destination(point=(self.lat, self.lon),
                                                   bearing=headingToBearingDegrees(self.headingDegrees))
        self.lon = destination.longitude
        self.lat = destination.latitude

        # Travel based on ocean current
        oceanCurrentSpeedKmph = rospy.get_param('ocean_current_speed', default=0.0)
        oceanCurrentDirectionDegress = rospy.get_param('ocean_current_direction', default=0.0)
        oceanCurrentKmTraveledPerPeriod = oceanCurrentSpeedKmph * self.publishPeriodSeconds / 3600.0
        oceanCurrentKmTraveledPerPeriod *= self.speedup  # Move greater distances with speedup
        distanceTraveled = geopy.distance.distance(kilometers=oceanCurrentKmTraveledPerPeriod)
        destination = distanceTraveled.destination(point=(self.lat, self.lon),
                                                   bearing=headingToBearingDegrees(oceanCurrentDirectionDegress))
        self.lon = destination.longitude
        self.lat = destination.latitude

        if self.smoothChanges:
            # Heading
            error = ssa_deg(self.headingSetpoint - self.headingDegrees)
            gain = min(1, TURNING_GAIN * self.publishPeriodSeconds * self.speedup)
            self.headingDegrees = (self.headingDegrees + gain * error) % 360

            # Speed
            if self.relativeWindDegrees is not None:
                if abs(ssa_deg(self.relativeWindDegrees)) < 15 or abs(ssa_deg(self.relativeWindDegrees)) > 165:
                    self.speedSetpoint = self.origSpeedSetpoint / 4.0
                else:
                    self.speedSetpoint = self.origSpeedSetpoint
                print(self.speedSetpoint)

            error = self.speedSetpoint - self.speedKmph
            gain = min(1, SPEED_CHANGE_GAIN * self.publishPeriodSeconds * self.speedup)
            self.speedKmph += gain * error

    def windSensorCallback(self, data):
        rospy.loginfo(data)
        self.relativeWindDegrees = data.measuredDirectionDegrees

    def desiredHeadingCallback(self, data):
        rospy.loginfo(data)
        headingDegreesNewCoordinates = (-1) * data.headingDegrees + 90
        if self.smoothChanges:
            self.headingSetpoint = headingDegreesNewCoordinates
            error_mag = abs(ssa_deg(self.headingSetpoint - self.headingDegrees))
            # Heading change >= 135 has maximum effect (speed is divided by 3)
            # Heading change <= 45 has no effect on speed
            self.speedKmph = self.speedKmph / max(1, min(3, error_mag / 45))
        else:
            self.headingDegrees = headingDegreesNewCoordinates

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
    smooth_changes = rospy.get_param('smooth_changes', default=True)

    if gps_file:
        with open(gps_file) as f:
            record = json.loads(f.read())
            MOCK_ctrl_sailbot = MOCK_ControllerAndSailbot(*record)
    else:
        # Boat should move at about 4m/s = 14.4 km/h
        MOCK_ctrl_sailbot = MOCK_ControllerAndSailbot(PORT_RENFREW_LATLON.lat, PORT_RENFREW_LATLON.lon, 180, 14.4)

    MOCK_ctrl_sailbot.smoothChanges = smooth_changes

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
