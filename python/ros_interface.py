#!/usr/bin/env python
import itertools
from collections import OrderedDict
import numpy as np
import rospy
from sailbot_msg.msg import Sensors, windSensor, GPS, globalWind
from utilities import bearingToHeadingDegrees, measuredWindToGlobalWind, angleAverage


# Globals for wind filtering
WIND_SENSOR_WEIGHTS = {1: 0.5, 2: 0.25, 3: 0.25}
ACTIVE_WIND_SENSORS = (1, 2, 3)

# Constants
CHECK_PERIOD_SECONDS = 0.1  # How often fields are updated
KNOTS_TO_KMPH = 1.852


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
            self.convert()
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
        Note: the order of fields must match the msg file (fields in filter() are before convert().'''
        # GPS sensor fields - CAN and AIS - take average
        self.gps['lat_decimal_degrees'] = np.mean((self.sensors.gps_can_latitude_degrees,
                                                   self.sensors.gps_ais_latitude_degrees))
        self.gps['lon_decimal_degrees'] = np.mean((self.sensors.gps_can_longitude_degrees,
                                                   self.sensors.gps_ais_longitude_degrees))
        self.gps['speed_knots'] = np.mean((self.sensors.gps_can_groundspeed_knots,
                                           self.sensors.gps_ais_groundspeed_knots))
        self.gps['bearing_degrees'] = angleAverage((self.sensors.gps_can_true_heading_degrees,
                                                    self.sensors.gps_ais_true_heading_degrees))

        # Wind sensor fields - sensors 1, 2, and 3 -> take EWMA (circular data like direction handled differently)
        active_wind_sensor_weights = [WIND_SENSOR_WEIGHTS[num] for num in ACTIVE_WIND_SENSORS]
        speed_knots_sensor_data = [getattr(self.sensors, attr) for attr in ['wind_sensor_{}_speed_knots'.format(num)
                                                                            for num in ACTIVE_WIND_SENSORS]]
        self.wind_sensor['measured_speed_knots'] = np.average(speed_knots_sensor_data,
                                                              weights=active_wind_sensor_weights)
        angle_degrees_sensor_data = [getattr(self.sensors, attr) for attr in ['wind_sensor_{}_angle_degrees'.format(num)
                                                                              for num in ACTIVE_WIND_SENSORS]]
        self.wind_sensor['measured_bearing_degrees'] = angleAverage(angle_degrees_sensor_data,
                                                                    weights=active_wind_sensor_weights)
        rospy.logfatal(active_wind_sensor_weights)
        rospy.logwarn(speed_knots_sensor_data)

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


if __name__ == "__main__":
    rosInterface = RosInterface()
    rate = rospy.Rate(1 / CHECK_PERIOD_SECONDS)
    while not rospy.is_shutdown():
        rosInterface.pub()
        rate.sleep()
