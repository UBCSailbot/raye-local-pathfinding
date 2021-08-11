#!/usr/bin/env python
import rospy
from sailbot_msg.msg import Sensors, windSensor, GPS, globalWind
from utilities import bearingToHeadingDegrees, measuredWindToGlobalWind


# Constants
CHECK_PERIOD_SECONDS = 0.1  # How often fields are updated
KNOTS_TO_KMPH = 1.852
MAX_ALLOWABLE_PERCENT_ERROR = 0.2  # If signals need different margin of errors; modify as required

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
        self.initialized = False
        self.data = None

        # Current state
        self.gps_lat_decimalDegrees = None
        self.gps_lon_decimalDegrees = None
        self.gps_headingDegrees = None
        self.gps_speedKmph = None
        self.measured_wind_speedKmph = None
        self.measured_wind_direction = None

        # Past state
        self.past_gps_lat_decimalDegrees = None
        self.past_gps_lon_decimalDegrees = None
        self.past_gps_headingDegrees = None
        self.past_gps_speedKmph = None
        self.past_measured_wind_speedKmph = None
        self.past_measured_wind_direction = None

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

            rospy.loginfo("Publishing to GPS and windSensor with self.gps_lat_decimalDegrees = {}, "
                          "self.gps_lon_decimalDegrees = {}, self.gps_headingDegrees = {}, self.gps_speedKmph = {}, "
                          "self.measured_wind_direction = {}, self.measured_wind_speedKmph = {}"
                          .format(self.gps_lat_decimalDegrees, self.gps_lon_decimalDegrees, self.gps_headingDegrees,
                                  self.gps_speedKmph, self.measured_wind_direction, self.measured_wind_speedKmph))
        else:
            rospy.loginfo("Tried to publish sensor values, but not initialized yet. Waiting for first sensor message.")

    # Translate from many sensor outputs to one signal, checking error margin, averaging, and converting units
    def translate(self):
        data = self.data

        # Past values
        self.past_gps_lat_decimalDegrees = self.gps_lat_decimalDegrees
        self.past_gps_lon_decimalDegrees = self.gps_lon_decimalDegrees
        self.past_gps_headingDegrees = self.gps_headingDegrees
        self.past_gps_speedKmph = self.gps_speedKmph
        self.past_measured_wind_speedKmph = self.measured_wind_speedKmph
        self.past_measured_wind_direction = self.measured_wind_direction

        # GPS sensors - use can, ais, or both? which takes priority? true heading / track made good usage?
        self.gps_lat_decimalDegrees = self.getTrustedAvg(
                pastValue=self.past_gps_lat_decimalDegrees,
                vals=[data.gps_can_latitude_degrees, data.gps_ais_latitude_degrees])
        self.gps_lon_decimalDegrees = self.getTrustedAvg(
                pastValue=self.past_gps_lon_decimalDegrees,
                vals=[data.gps_can_longitude_degrees, data.gps_ais_longitude_degrees])
        self.gps_headingDegrees = self.getTrustedAvg(
                pastValue=self.past_gps_headingDegrees,
                vals=[bearingToHeadingDegrees(data.gps_can_true_heading_degrees),
                      bearingToHeadingDegrees(data.gps_ais_true_heading_degrees)])
        self.gps_speedKmph = self.getTrustedAvg(
                pastValue=self.past_gps_speedKmph,
                vals=[data.gps_can_groundspeed_knots * KNOTS_TO_KMPH,
                      data.gps_ais_groundspeed_knots * KNOTS_TO_KMPH])

        # Wind sensors
        self.measured_wind_speedKmph = self.getTrustedAvg(
                pastValue=self.past_measured_wind_speedKmph,
                vals=[data.wind_sensor_1_speed_knots * KNOTS_TO_KMPH,
                      data.wind_sensor_2_speed_knots * KNOTS_TO_KMPH,
                      data.wind_sensor_3_speed_knots * KNOTS_TO_KMPH])
        self.measured_wind_direction = self.getTrustedAvg(
                pastValue=self.past_measured_wind_direction,
                vals=[bearingToHeadingDegrees(data.wind_sensor_1_angle_degrees),
                      bearingToHeadingDegrees(data.wind_sensor_2_angle_degrees),
                      bearingToHeadingDegrees(data.wind_sensor_3_angle_degrees)])

    def getTrustedAvg(self, pastValue, vals):
        '''Averages the trust values, or defaults to the first term in vals if pastValue is 0 or not initialized'''
        trustedVals = self.getTrusted(pastValue, vals)
        if len(trustedVals) == 0:
            rospy.logwarn("Values don't match with pastValue, or if pastValue is None or 0, defaulting to first sensor")
            return vals[0]
        else:
            return float(sum(trustedVals)) / len(trustedVals)

    def getTrusted(self, pastValue, vals):
        '''Returns vals without outlier terms like None
            - Assumes sensor readings are consistent and first sensor initial reading is accurate
            - Could modify past value to be the average of the past few outputs (store in list)
        '''
        if pastValue is None or pastValue == 0:
            return vals

        trustedVals = []
        for value in vals:
            if value is not None and abs(float(pastValue - value) / pastValue) < MAX_ALLOWABLE_PERCENT_ERROR:
                trustedVals.append(value)
        return trustedVals

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
