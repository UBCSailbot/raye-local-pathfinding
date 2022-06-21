#!/usr/bin/env python
import numpy as np
import rospy
from sailbot_msg.msg import Sensors, windSensor, GPS, globalWind
from std_msgs.msg import Bool
from utilities import bearingToHeadingDegrees, measuredWindToGlobalWind


# Constants
CHECK_PERIOD_SECONDS = 0.1  # How often fields are updated
KNOTS_TO_KMPH = 1.852
LOW_WIND_THRESHOLD_KMPH = 4.5
LOW_BOAT_SPEED_THRESHOLD_KMPH = 1.5
HYSTERESIS_MARGINE_WIND = 1.3
HYSTERESIS_MARGINE_BOATSPEED = 0.3

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
        self.pubLowWindConditions = rospy.Publisher('lowWindConditions', Bool, queue_size=4)
        self.windIsLow = False
        self.initialized = False
        self.data = None

        # Current state
        self.gps_lat_decimalDegrees = None
        self.gps_lon_decimalDegrees = None
        self.gps_headingDegrees = None
        self.gps_speedKmph = None
        self.measured_wind_speedKmph = None
        self.measured_wind_direction = None

    def sensorsCallback(self, data):
        if not self.initialized:
            self.initialized = True
        self.data = data

    def pub(self):
        if self.initialized:
            self.translate()
            self.pubGPS.publish(self.gps_lat_decimalDegrees, self.gps_lon_decimalDegrees,
                                self.gps_headingDegrees, self.gps_speedKmph)
            self.pubMeasuredWind.publish(self.measured_wind_direction, self.measured_wind_speedKmph)
            self.pubGlobalWind.publish(self.get_global_wind())
            self.updateWindIsLow()
            self.pubLowWindConditions.publish(self.windIsLow)

            rospy.loginfo("Publishing to GPS and windSensor with self.gps_lat_decimalDegrees = {}, "
                          "self.gps_lon_decimalDegrees = {}, self.gps_headingDegrees = {}, self.gps_speedKmph = {}, "
                          "self.measured_wind_direction = {}, self.measured_wind_speedKmph = {}"
                          .format(self.gps_lat_decimalDegrees, self.gps_lon_decimalDegrees, self.gps_headingDegrees,
                                  self.gps_speedKmph, self.measured_wind_direction, self.measured_wind_speedKmph))
        else:
            rospy.loginfo("Tried to publish sensor values, but not initialized yet. Waiting for first sensor message.")

    def translate(self):
        '''Translate data from multiple sensors into one input field, performing the necessary unit conversions.'''
        # GPS sensor fields - CAN and AIS -> use CAN
        self.gps_lat_decimalDegrees = self.data.gps_can_latitude_degrees
        self.gps_lon_decimalDegrees = self.data.gps_can_longitude_degrees
        self.gps_headingDegrees = bearingToHeadingDegrees(self.data.gps_can_true_heading_degrees)
        self.gps_speedKmph = self.data.gps_can_groundspeed_knots * KNOTS_TO_KMPH

        # Wind sensor fields - sensors 1, 2, and 3 -> use median for speed, sensor 1 for direction
        self.measured_wind_speedKmph = np.median([self.data.wind_sensor_1_speed_knots,
                                                  self.data.wind_sensor_2_speed_knots,
                                                  self.data.wind_sensor_3_speed_knots]) * KNOTS_TO_KMPH
        self.measured_wind_direction = bearingToHeadingDegrees(self.data.wind_sensor_1_angle_degrees)

        # Log inputs fields
        rospy.loginfo('gps_lat_decimalDegrees = {}'.format(self.gps_lat_decimalDegrees))
        rospy.loginfo('gps_lon_decimalDegrees = {}'.format(self.gps_lon_decimalDegrees))
        rospy.loginfo('gps_headingDegrees = {}'.format(self.gps_headingDegrees))
        rospy.loginfo('gps_speedKmph = {}'.format(self.gps_speedKmph))
        rospy.loginfo('measured_wind_speedKmph = {}'.format(self.measured_wind_speedKmph))
        rospy.loginfo('measured_wind_direction = {}'.format(self.measured_wind_direction))

    def updateWindIsLow(self):
        if self.windIsLow:
            rospy.logwarn_throttle(5, "Low wind conditions. " +
                                   "Measured Wind: {}km/h. ".format(self.measured_wind_speedKmph) +
                                   "Boat Speed: {}km/h".format(self.gps_speedKmph))

            # Already in low wind mode, so need boat gps speed OR apparent wind speed to be high enough to get out
            if((self.measured_wind_speedKmph > LOW_WIND_THRESHOLD_KMPH + HYSTERESIS_MARGINE_WIND)
               or (self.gps_speedKmph > LOW_BOAT_SPEED_THRESHOLD_KMPH + HYSTERESIS_MARGINE_BOATSPEED)):

                self.windIsLow = False

        else:
            # Global wind is not low: enter low wind mode if boat gps speed AND apparent wind speed become too low
            if((self.measured_wind_speedKmph < LOW_WIND_THRESHOLD_KMPH - HYSTERESIS_MARGINE_WIND)
               and (self.gps_speedKmph < LOW_BOAT_SPEED_THRESHOLD_KMPH - HYSTERESIS_MARGINE_BOATSPEED)):

                self.windIsLow = True

    def get_global_wind(self):
        speed, direction = measuredWindToGlobalWind(
                measuredWindSpeed=self.measured_wind_speedKmph,
                measuredWindDirectionDegrees=self.measured_wind_direction,
                boatSpeed=self.gps_speedKmph,
                headingDegrees=self.gps_headingDegrees)

        return globalWind(directionDegrees=direction, speedKmph=speed)


if __name__ == "__main__":
    rosInterface = RosInterface()
    rate = rospy.Rate(1 / CHECK_PERIOD_SECONDS)
    while not rospy.is_shutdown():
        rosInterface.pub()
        rate.sleep()
