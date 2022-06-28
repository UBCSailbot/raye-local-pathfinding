#!/usr/bin/env python
import numpy as np
import rospy
from sailbot_msg.msg import Sensors, windSensor, GPS, globalWind
from utilities import bearingToHeadingDegrees, measuredWindToGlobalWind, vectorAverage, ewma, KNOTS_TO_KMPH


# Constants
CHECK_PERIOD_SECONDS = 0.1  # How often fields are updated

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
        self.gps = GPS()
        self.wind_sensor = windSensor()
        self.global_wind = globalWind()

        # initialize EWMA past values to None (signifying no past value)
        self.wind_sensor.measuredSpeedKnots = None
        self.wind_sensor.measuredBearingDegrees = None

    def sensorsCallback(self, data):
        if not self.initialized:
            self.initialized = True
        self.sensors = data

    def pub(self):
        if self.initialized:
            self.filter()
            self.convert()
            self.pubGPS.publish(self.gps)
            self.pubMeasuredWind.publish(self.wind_sensor)
            self.get_global_wind()
            self.pubGlobalWind.publish(self.global_wind)

            rospy.loginfo("Publishing to GPS, windSensor, and global_wind")
            rospy.loginfo("GPS fields:\n{}".format(self.gps))
            rospy.loginfo("windSensor fields:\n{}".format(self.wind_sensor))
            rospy.loginfo("global_wind fields:\n{}".format(self.global_wind))
        else:
            rospy.loginfo("Tried to publish sensor values, but not initialized yet. Waiting for first sensor message.")

    def filter(self):
        '''Filter data from multiple sensors into one input field, performing the necessary unit conversions.
        Note: the order of fields must match the msg file (fields in filter() are before convert().'''
        # GPS sensor fields - CAN and AIS - take average
        self.gps.lat = np.mean([getattr(self.sensors, attr) for attr in self.active_gps_lats])
        self.gps.lon = np.mean([getattr(self.sensors, attr) for attr in self.active_gps_lons])
        self.gps.speedKnots, self.gps.bearingDegrees = \
            vectorAverage([getattr(self.sensors, attr) for attr in self.active_gps_speeds],
                          [getattr(self.sensors, attr) for attr in self.active_gps_headings])

        # Wind sensor fields - sensors 1, 2, and 3 -> take EWMA (circular data like direction handled differently)
        speed_knots_sensor_data = [getattr(self.sensors, attr) for attr in self.active_wind_sensor_speeds]
        angle_degrees_sensor_data = [getattr(self.sensors, attr) for attr in self.active_wind_sensor_angles]
        speed_knots_averaged, angle_degrees_averaged = \
            vectorAverage(speed_knots_sensor_data, angle_degrees_sensor_data, self.active_wind_sensor_weights)
        ewma_weight = self.get_ewma_weight(speed_knots_averaged)
        rospy.loginfo('EWMA weight: {}'.format(ewma_weight))
        self.wind_sensor.measuredSpeedKnots = ewma(current_val=speed_knots_averaged,
                                                   past_ewma=self.wind_sensor.measuredSpeedKnots,
                                                   weight=ewma_weight, is_angle=False)
        self.wind_sensor.measuredBearingDegrees = ewma(current_val=angle_degrees_averaged,
                                                       past_ewma=self.wind_sensor.measuredBearingDegrees,
                                                       weight=ewma_weight, is_angle=True)

    def convert(self):
        '''Convert to conventions used by the pathfinding and controller. Conversion notes:
            - Sensors data types are int32 or float32, GPS and windSensor are float64, so no loss of precision
            - gps speed_kmph and wind_sensor measured_speed_kmph: knots -> km/h
            - gps heading_degrees and wind_sensor measured_heading_degrees: bearing to heading degrees
              (0 is north/forward, 90 is east/right - cw) -> (0 is east/right, 90 is north/forward - ccw)
        Note: the order of fields must match the msg file (fields in convert() are after filter().
        '''
        self.gps.speedKmph = self.gps.speedKnots * KNOTS_TO_KMPH
        self.gps.headingDegrees = bearingToHeadingDegrees(self.gps.bearingDegrees)
        self.wind_sensor.measuredSpeedKmph = self.wind_sensor.measuredSpeedKnots * KNOTS_TO_KMPH
        self.wind_sensor.measuredDirectionDegrees = \
            bearingToHeadingDegrees(self.wind_sensor.measuredBearingDegrees)

    def get_global_wind(self):
        speed, direction = measuredWindToGlobalWind(
            measuredWindSpeed=self.wind_sensor.measuredSpeedKmph,
            measuredWindDirectionDegrees=self.wind_sensor.measuredDirectionDegrees,
            boatSpeed=self.gps.speedKmph,
            headingDegrees=self.gps.headingDegrees)

        self.global_wind.directionDegrees = direction
        self.global_wind.speedKmph = speed

    def get_ewma_weight(self, measured_speed_knots):
        '''Simple linear function to get EWMA weight.'''
        for i in range(len(WIND_EWMA_UPPER_BOUNDS)):
            if measured_speed_knots < WIND_EWMA_UPPER_BOUNDS[i]:
                return WIND_EWMA_WEIGHTS[i]
        return WIND_EWMA_WEIGHTS[-1]


if __name__ == "__main__":
    rosInterface = RosInterface()
    rate = rospy.Rate(1 / CHECK_PERIOD_SECONDS)
    while not rospy.is_shutdown():
        rosInterface.pub()
        rate.sleep()
