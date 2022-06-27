#!/usr/bin/env python
import itertools
from collections import OrderedDict
import numpy as np
import rospy
from sailbot_msg.msg import Sensors, windSensor, GPS, globalWind
from std_msgs.msg import Bool
from utilities import bearingToHeadingDegrees, measuredWindToGlobalWind, vectorAverage, ewma, KNOTS_TO_KMPH


# Constants
CHECK_PERIOD_SECONDS = 0.1  # How often fields are updated
LOW_WIND_THRESHOLD_KMPH = 4.5
LOW_BOAT_SPEED_THRESHOLD_KMPH = 1.5
HYSTERESIS_MARGINE_WIND = 1.3
HYSTERESIS_MARGINE_BOATSPEED = 0.3

# Constants for GPS filtering
ACTIVE_GPS_SENSORS = ('can', 'ais')  # tuple of GPS names

# Constants for wind sensor filtering
ACTIVE_WIND_SENSORS = (1, 2, 3)  # tuple of sensor numbers
WIND_SENSOR_WEIGHTS = {1: 0.5, 2: 0.25, 3: 0.25}  # dict of sensor numbers mapping to their weights
WIND_EWMA_WEIGHTS = [0.075, 0.35]  # weights in [0, 1] (floats) in ascending order; see get_ewma_weight()
WIND_EWMA_UPPER_BOUNDS = [3.0]  # speed in knots (floats) in ascending order; see get_ewma_weight()


class RosInterface:
    def __init__(self):
        rospy.init_node('ros_interface', anonymous=True)
        rospy.Subscriber("sensors", Sensors, self.sensorsCallback)
        self.pubMeasuredWind = rospy.Publisher('windSensor', windSensor, queue_size=4)
        self.pubGlobalWind = rospy.Publisher('global_wind', globalWind, queue_size=4)
        self.pubGPS = rospy.Publisher('GPS', GPS, queue_size=4)
        self.pubLowWindConditions = rospy.Publisher('lowWindConditions', Bool, queue_size=4)
        self.windIsLow = False

        # derived constants
        self.active_gps_lats = ['gps_{}_latitude_degrees'.format(gps) for gps in ACTIVE_GPS_SENSORS]
        self.active_gps_lons = ['gps_{}_longitude_degrees'.format(gps) for gps in ACTIVE_GPS_SENSORS]
        self.active_gps_speeds = ['gps_{}_groundspeed_knots'.format(gps) for gps in ACTIVE_GPS_SENSORS]
        self.active_gps_headings = ['gps_{}_true_heading_degrees'.format(gps) for gps in ACTIVE_GPS_SENSORS]
        self.active_wind_sensor_weights = [WIND_SENSOR_WEIGHTS[num] for num in ACTIVE_WIND_SENSORS]
        self.active_wind_sensor_speeds = ['wind_sensor_{}_speed_knots'.format(num) for num in ACTIVE_WIND_SENSORS]
        self.active_wind_sensor_angles = ['wind_sensor_{}_angle_degrees'.format(num) for num in ACTIVE_WIND_SENSORS]

        # ROS topic data
        self.initialized = False
        self.sensors = None
        self.gps = OrderedDict()
        self.wind_sensor = OrderedDict()
        self.global_wind = OrderedDict()

        # initialize EWMA past values to None (signifying no past value)
        self.wind_sensor['measured_speed_knots'] = None
        self.wind_sensor['measured_bearing_degrees'] = None

    def sensorsCallback(self, data):
        if not self.initialized:
            self.initialized = True
        self.sensors = data

    def pub(self):
        if self.initialized:
            self.filter()
            self.convert()
            self.pubGPS.publish(*itertools.chain(self.gps.values()))
            self.pubMeasuredWind.publish(*itertools.chain(self.wind_sensor.values()))
            self.get_global_wind()
            self.pubGlobalWind.publish(*itertools.chain(self.global_wind.values()))
            self.updateWindIsLow()
            self.pubLowWindConditions.publish(self.windIsLow)

            rospy.loginfo("Publishing to GPS, windSensor, and global_wind")
            rospy.loginfo("GPS fields: {}".format(dict(self.gps)))
            rospy.loginfo("windSensor fields: {}".format(dict(self.wind_sensor)))
            rospy.loginfo("global_wind fields: {}".format(dict(self.global_wind)))
        else:
            rospy.loginfo("Tried to publish sensor values, but not initialized yet. Waiting for first sensor message.")

    def filter(self):
        '''Filter data from multiple sensors into one input field, performing the necessary unit conversions.
        Note: the order of fields must match the msg file (fields in filter() are before convert().'''
        # GPS sensor fields - CAN and AIS - take average
        self.gps['lat_decimal_degrees'] = np.mean([getattr(self.sensors, attr) for attr in self.active_gps_lats])
        self.gps['lon_decimal_degrees'] = np.mean([getattr(self.sensors, attr) for attr in self.active_gps_lons])
        self.gps['speed_knots'], self.gps['bearing_degrees'] = \
            vectorAverage([getattr(self.sensors, attr) for attr in self.active_gps_speeds],
                          [getattr(self.sensors, attr) for attr in self.active_gps_headings])

        # Wind sensor fields - sensors 1, 2, and 3 -> take EWMA (circular data like direction handled differently)
        speed_knots_sensor_data = [getattr(self.sensors, attr) for attr in self.active_wind_sensor_speeds]
        angle_degrees_sensor_data = [getattr(self.sensors, attr) for attr in self.active_wind_sensor_angles]
        speed_knots_averaged, angle_degrees_averaged = \
            vectorAverage(speed_knots_sensor_data, angle_degrees_sensor_data, self.active_wind_sensor_weights)
        ewma_weight = self.get_ewma_weight(speed_knots_averaged)
        rospy.loginfo('EWMA weight: {}'.format(ewma_weight))
        self.wind_sensor['measured_speed_knots'] = ewma(current_val=speed_knots_averaged,
                                                        past_ewma=self.wind_sensor['measured_speed_knots'],
                                                        weight=ewma_weight, is_angle=False)
        self.wind_sensor['measured_bearing_degrees'] = ewma(current_val=angle_degrees_averaged,
                                                            past_ewma=self.wind_sensor['measured_bearing_degrees'],
                                                            weight=ewma_weight, is_angle=True)

    def convert(self):
        '''Convert to conventions used by the pathfinding and controller. Conversion notes:
            - Sensors data types are int32 or float32, GPS and windSensor are float64, so no loss of precision
            - gps speed_kmph and wind_sensor measured_speed_kmph: knots -> km/h
            - gps heading_degrees and wind_sensor measured_heading_degrees: bearing to heading degrees
              (0 is north/forward, 90 is east/right - cw) -> (0 is east/right, 90 is north/forward - ccw)
        Note: the order of fields must match the msg file (fields in convert() are after filter().
        '''
        self.gps['speed_kmph'] = self.gps['speed_knots'] * KNOTS_TO_KMPH
        self.gps['heading_degrees'] = bearingToHeadingDegrees(self.gps['bearing_degrees'])
        self.wind_sensor['measured_speed_kmph'] = self.wind_sensor['measured_speed_knots'] * KNOTS_TO_KMPH
        self.wind_sensor['measured_heading_degrees'] = \
            bearingToHeadingDegrees(self.wind_sensor['measured_bearing_degrees'])

    def get_global_wind(self):
        speed, direction = measuredWindToGlobalWind(
            measuredWindSpeed=self.wind_sensor['measured_speed_kmph'],
            measuredWindDirectionDegrees=self.wind_sensor['measured_heading_degrees'],
            boatSpeed=self.gps['speed_kmph'],
            headingDegrees=self.gps['heading_degrees'])

        self.global_wind['heading_degrees'] = direction
        self.global_wind['speed_kmph'] = speed

    def get_ewma_weight(self, measured_speed_knots):
        '''Simple linear function to get EWMA weight.'''
        for i in range(len(WIND_EWMA_UPPER_BOUNDS)):
            if measured_speed_knots < WIND_EWMA_UPPER_BOUNDS[i]:
                return WIND_EWMA_WEIGHTS[i]
        return WIND_EWMA_WEIGHTS[-1]

    def updateWindIsLow(self):
        if self.windIsLow:
            rospy.logwarn_throttle(5, "Low wind conditions. Measured Wind: {}km/h. Boat Speed: {}km/h"
                                   .format(self.wind_sensor['measured_speed_kmph'], self.gps['speed_kmph']))
            # Already in low wind mode, so need boat gps speed OR apparent wind speed to be high enough to get out
            if((self.wind_sensor['measured_speed_kmph'] > LOW_WIND_THRESHOLD_KMPH + HYSTERESIS_MARGINE_WIND)
               or (self.gps['speed_kmph'] > LOW_BOAT_SPEED_THRESHOLD_KMPH + HYSTERESIS_MARGINE_BOATSPEED)):
                self.windIsLow = False
        else:
            # Global wind is not low: enter low wind mode if boat gps speed AND apparent wind speed become too low
            if((self.wind_sensor['measured_speed_kmph'] < LOW_WIND_THRESHOLD_KMPH - HYSTERESIS_MARGINE_WIND)
               and (self.gps['speed_kmph'] < LOW_BOAT_SPEED_THRESHOLD_KMPH - HYSTERESIS_MARGINE_BOATSPEED)):
                self.windIsLow = True


if __name__ == "__main__":
    rosInterface = RosInterface()
    rate = rospy.Rate(1 / CHECK_PERIOD_SECONDS)
    while not rospy.is_shutdown():
        rosInterface.pub()
        rate.sleep()
