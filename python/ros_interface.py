#!/usr/bin/env python
import rospy
from sailbot_msg.msg import Sensors, windSensor, GPS  # does the sailbot_msg name need to be updated?

# Constants
CHECK_PERIOD_SECONDS = 0.1  # How often fields are updated
MIN_TO_DEGREES = 1.0 / 60
KNOTS_TO_KMPH = 1.852
ERROR = 0.2  # If signals need different margin of errors; modify as required

"""
Conversion Notes:
- Sensors data types are int32 or float32, GPS and windSensor are float64, so no loss of precision
- gps_lat_decimalDegrees, gps_lon_decimalDegrees: degrees and decimal minutes (DDMM.MMMMMMM) -> decimal degrees
- gps_heading_degrees: (0 is north, 90 is east, etc - cw - degrees) -> (0 is east, 90 is north, etc - ccw - degrees)
- wind_direction: (0 is forward, 90 is right, etc - cw - degrees) -> (0 is right, 90 is bow, etc - ccw - degrees)
- gps_speedKmph, wind_speedKmph: knots -> km/h
"""


class RosInterface:
    def __init__(self):
        rospy.init_node('ros_interface', anonymous=True)
        rospy.Subscriber("sensors", Sensors, self.sensorsCallback)
        self.pubWind = rospy.Publisher('windSensor', windSensor, queue_size=4)
        self.pubGPS = rospy.Publisher('GPS', GPS, queue_size=4)
        self.initialized = False
        self.data = None

        self.gps_lat_decimalDegrees = None
        self.gps_lon_decimalDegrees = None
        self.gps_headingDegrees = None
        self.gps_speedKmph = None
        self.wind_speedKmph = None
        self.wind_direction = None

        self.past_gps_lat_decimalDegrees = None
        self.past_gps_lon_decimalDegrees = None
        self.past_gps_headingDegrees = None
        self.past_gps_speedKmph = None
        self.past_wind_speedKmph = None
        self.past_wind_direction = None

    def sensorsCallback(self, data):
        if not self.initialized:
            self.initialized = True
        self.data = data

    def pub(self):
        if self.initialized:
            self.translate()
            self.pubGPS.publish(self.gps_lat, self.gps_lon, self.gps_headingDegrees, self.gps_speedKmph)
            self.pubWind.publish(self.wind_direction, self.wind_speedKmph)
        else:
            rospy.logwarn("Tried to publish sensor values, but not initialized yet. Waiting for first sensor message.")

    # Translate from many sensor outputs to one signal, checking error margin, averaging, and converting units
    def translate(self):
        data = self.data

        # Past values
        self.past_gps_lat_decimalDegrees = self.gps_lat_decimalDegrees
        self.past_gps_lon_decimalDegrees = self.gps_lon_decimalDegrees
        self.past_gps_headingDegrees = self.gps_headingDegrees
        self.past_gps_speedKmph = self.gps_speedKmph
        self.past_wind_speedKmph = self.wind_speedKmph
        self.past_wind_direction = self.wind_direction

        # GPS sensors - use can, ais, or both? which takes priority? true heading / track made good usage?
        self.gps_lat_decimalDegrees = self.getTrustedAvg(self.past_gps_lat_decimalDegrees,
                                                         [self.convertCoordinates(data.gps_can_latitude_degreeMinutes),
                                                          self.convertCoordinates(data.gps_ais_latitude_degreeMinutes)
                                                          ])
        self.gps_lon_decimalDegrees = self.getTrustedAvg(self.past_gps_lon_decimalDegrees,
                                                         [self.convertCoordinates(data.gps_can_longitude_degreeMinutes),
                                                          self.convertCoordinates(data.gps_ais_longitude_degreeMinutes)
                                                          ])
        self.gps_headingDegrees = self.getTrustedAvg(self.past_gps_headingDegrees,
                                                     [self.convertDegrees(data.gps_can_true_heading_degrees),
                                                      self.convertDegrees(data.gps_ais_true_heading_degrees)])
        self.gps_speedKmph = self.getTrustedAvg(self.past_gps_speedKmph,
                                                [data.gps_can_groundspeed_knots * KNOTS_TO_KMPH,
                                                 data.gps_ais_groundspeed_knots * KNOTS_TO_KMPH])
        # Wind sensors
        self.wind_speedKmph = self.getTrustedAvg(self.past_wind_speedKmph,
                                                 [data.wind_sensor_1_speed_knots * KNOTS_TO_KMPH,
                                                  data.wind_sensor_2_speed_knots * KNOTS_TO_KMPH,
                                                  data.wind_sensor_3_speed_knots * KNOTS_TO_KMPH])
        self.wind_direction = self.getTrustedAvg(self.past_wind_direction,
                                                 [self.convertDegrees(data.wind_sensor_1_angle_degrees),
                                                  self.convertDegrees(data.wind_sensor_2_angle_degrees),
                                                  self.convertDegrees(data.wind_sensor_3_angle_degrees)])

    def convertCoordinates(self, coord):
        '''Convert coordinate units from degrees and decimal minutes (DDMM.MMMMMMM) to decimal degrees'''
        coordStr = str(coord)
        return int(coordStr[0:2]) + float(coordStr[2:]) * MIN_TO_DEGREES

    def convertDegrees(self, degree):
        '''Convert degree convention from (0 is up, 90 is right, etc - cw - degrees)
        to (0 is right, 90 is up, etc - ccw - degrees)
        '''
        converted = -1 * degree + 90
        return converted if converted >= 0 else converted + 360

    def getTrustedAvg(self, pastValue, vals):
        '''Averages the trust values, or defaults to the first term in vals if pastValue is 0 or not initialized'''
        err_free_vals = self.getTrusted(pastValue, vals)
        if not err_free_vals:
            rospy.logwarn("Values don't match with pastValue, or if pastValue is None or 0, defaulting to first sensor")
            return vals[0]
        else:
            return float(sum(err_free_vals)) / len(err_free_vals)

    def getTrusted(self, pastValue, vals):
        '''Returns vals without None, 0, and outlier terms
            - Assumes sensor readings are consistent and first sensor initial reading is accurate
            - Could modify past value to be the average of the past few outputs (store in list)
        '''
        errorFreeVals = []
        for value in vals:
            if pastValue is not None and pastValue != 0 and abs(float((pastValue - value)) / pastValue) < ERROR:
                errorFreeVals.append(value)
        return errorFreeVals


if __name__ == "__main__":
    rosInterface = RosInterface()
    rate = rospy.Rate(1 / CHECK_PERIOD_SECONDS)
    while not rospy.is_shutdown():
        rosInterface.pub()
        rate.sleep()
