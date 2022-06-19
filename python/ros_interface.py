#!/usr/bin/env python
import itertools
from collections import OrderedDict
import numpy as np
import rospy
from sailbot_msg.msg import Sensors, windSensor, GPS, globalWind
from utilities import bearingToHeadingDegrees, measuredWindToGlobalWind


# Constants
CHECK_PERIOD_SECONDS = 0.1  # How often fields are updated
KNOTS_TO_KMPH = 1.852

"""
Conversion Notes:
- Sensors data types are int32 or float32, GPS and windSensor are float64, so no loss of precision
- gps_lat_decimalDegrees and gps_lon_decimalDegrees: already in decimal degrees (minutes conversion done upstream)
- gps_heading_degrees: (0 is north, 90 is east - cw - degrees) -> (0 is east, 90 is north - ccw - degrees)
- measured_wind_direction: (0 is forward, 90 is right - cw - degrees) -> (0 is right, 90 is bow - ccw - degrees)
- note that same angle coordinate conversion is needed for both gps_heading_degrees and measured_wind_direction
- gps_speedKmph, measured_wind_speedKmph: knots -> km/h
"""


class RosInterface:
    def __init__(self):
        rospy.init_node('ros_interface', anonymous=True)
        rospy.Subscriber("sensors", Sensors, self.sensorsCallback)
        self.pubMeasuredWind = rospy.Publisher('windSensor', windSensor, queue_size=4)
        self.pubGlobalWind = rospy.Publisher('global_wind', globalWind, queue_size=4)
        self.pubGPS = rospy.Publisher('GPS', GPS, queue_size=4)

        # ROS topic data
        self.initialized = False
        self.sensors = None
        self.gps = OrderedDict()
        self.wind_sensor = OrderedDict()
        self.global_wind = OrderedDict()

    def sensorsCallback(self, data):
        if not self.initialized:
            self.initialized = True
        self.sensors = data

    def pub(self):
        if self.initialized:
            self.filter()
            self.pubGPS.publish(*itertools.chain(self.gps.values()))
            self.pubMeasuredWind.publish(*itertools.chain(self.wind_sensor.values()))
            self.get_global_wind()
            self.pubGlobalWind.publish(*itertools.chain(self.global_wind.values()))

            rospy.loginfo("Publishing to GPS, windSensor, and global_wind")
            rospy.loginfo("GPS fields: {}".format(dict(self.gps)))
            rospy.loginfo("windSensor fields: {}".format(dict(self.wind_sensor)))
            rospy.loginfo("global_wind fields: {}".format(dict(self.global_wind)))
        else:
            rospy.loginfo("Tried to publish sensor values, but not initialized yet. Waiting for first sensor message.")

    def filter(self):
        '''Filter data from multiple sensors into one input field, performing the necessary unit conversions.
        Note: the order of fields must match the msg file.'''
        # GPS sensor fields - CAN and AIS -> use CAN
        self.gps['lat_decimal_degrees'] = self.sensors.gps_can_latitude_degrees
        self.gps['lon_decimal_degrees'] = self.sensors.gps_can_longitude_degrees
        self.gps['heading_degrees'] = bearingToHeadingDegrees(self.sensors.gps_can_true_heading_degrees)
        self.gps['speed_kmph'] = self.sensors.gps_can_groundspeed_knots * KNOTS_TO_KMPH

        # Wind sensor fields - sensors 1, 2, and 3 -> use median for speed, sensor 1 for direction
        self.wind_sensor['measured_heading_degrees'] = bearingToHeadingDegrees(self.sensors.wind_sensor_1_angle_degrees)
        self.wind_sensor['measured_speed_kmph'] = \
            np.median([self.sensors.wind_sensor_1_speed_knots, self.sensors.wind_sensor_2_speed_knots,
                       self.sensors.wind_sensor_3_speed_knots]) * KNOTS_TO_KMPH

    def get_global_wind(self):
        speed, direction = measuredWindToGlobalWind(
            measuredWindSpeed=self.wind_sensor['measured_speed_kmph'],
            measuredWindDirectionDegrees=self.wind_sensor['measured_heading_degrees'],
            boatSpeed=self.gps['speed_kmph'],
            headingDegrees=self.gps['heading_degrees'])

        self.global_wind['heading_degrees'] = direction
        self.global_wind['speed_kmph'] = speed


if __name__ == "__main__":
    rosInterface = RosInterface()
    rate = rospy.Rate(1 / CHECK_PERIOD_SECONDS)
    while not rospy.is_shutdown():
        rosInterface.pub()
        rate.sleep()
