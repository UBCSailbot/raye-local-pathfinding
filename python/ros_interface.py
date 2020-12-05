import rospy
from sailbot_msg.msg import Sensors, windSensor, GPS

# Constants
CHECK_PERIOD_SECONDS = 0.1  # How often fields are updated
KNOTS_TO_KMPH = 1.852
ERROR = 0.2  # If signals need different margin of errors, modify as required

"""
Conversion Notes:
- wind_speedKmph, wind_direction, wind_headingDegrees, gps_speedKmph: int32 -> float64 (no loss of precision)
- wind_speedKmph, gps_speedKmph: knots -> km/h
- wind_direction: (0 is forward, 90 is right, etc - cw - degrees) -> (0 is right, 90 is bow, etc - ccw - degrees)
- heading_degrees: (0 is north, 90 is east, etc - cw - degrees) -> (0 is east, 90 is north, etc - ccw - degrees)
"""


class RosInterface:
    def __init__(self):
        rospy.init_node('ros_interface', anonymous=True)
        rospy.Subscriber("sensors", Sensors, self.sensorsCallback)
        self.pubWind = rospy.Publisher('windSensor', windSensor, queue_size=4)
        self.pubGPS = rospy.Publisher('GPS', GPS, queue_size=4)
        self.initialized = False
        self.data = None

        self.wind_speedKmph = None
        self.wind_direction = None
        self.gps_lat = None
        self.gps_lon = None
        self.gps_headingDegrees = None
        self.gps_speedKmph = None

        self.past_wind_speedKmph = None
        self.past_wind_direction = None
        self.past_gps_lat = None
        self.past_gps_lon = None
        self.past_gps_headingDegrees = None
        self.past_gps_speedKmph = None

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
        self.past_gps_lat = self.gps_lat
        self.past_gps_lon = self.gps_lon
        self.past_gps_headingDegrees = self.gps_headingDegrees
        self.past_gps_speedKmph = self.gps_speedKmph
        self.past_wind_speedKmph = self.wind_speedKmph
        self.past_wind_direction = self.wind_direction

        # GPS sensors
        self.gps_lat = self.getTrustedAndAvg(self.past_gps_lat,
                                             [data.gps_0_latitude, data.gps_1_latitude])
        self.gps_lon = self.getTrustedAndAvg(self.past_gps_lon,
                                             [data.gps_0_longitude, data.gps_1_longitude])
        self.gps_headingDegrees = self.getTrustedAndAvg(self.past_gps_headingDegrees,
                                                        [self.convertDegrees(data.gps_0_true_heading),
                                                         self.convertDegrees(data.gps_1_true_heading)])
        self.gps_speedKmph = self.getTrustedAndAvg(self.past_gps_speedKmph,
                                                   [data.gps_0_groundspeed * KNOTS_TO_KMPH,
                                                    data.gps_1_groundspeed * KNOTS_TO_KMPH])
        # Wind sensors
        self.wind_speedKmph = self.getTrustedAndAvg(self.past_wind_speedKmph,
                                                    [data.wind_sensor_0_speed * KNOTS_TO_KMPH,
                                                     data.wind_sensor_1_speed * KNOTS_TO_KMPH,
                                                     data.wind_sensor_2_speed * KNOTS_TO_KMPH])
        self.wind_direction = self.getTrustedAndAvg(self.past_wind_direction,
                                                    [self.convertDegrees(data.wind_sensor_0_direction),
                                                     self.convertDegrees(data.wind_sensor_1_direction),
                                                     self.convertDegrees(data.wind_sensor_2_direction)])

    # (0 is up, 90 is right, etc - cw - degrees) -> (0 is right, 90 is up, etc - ccw - degrees)
    def convertDegrees(self, degree):
        converted = -1 * degree + 90
        return converted if converted >= 0 else converted + 360

    def getTrustedAndAvg(self, pastValue, vals):
        err_free_vals = self.getTrusted(pastValue, vals)
        if not err_free_vals:
            rospy.logwarn("Values don't match with pastValue, or if pastValue is none or 0, defaulting to first sensor")
            return vals[0]
        else:
            return float(sum(err_free_vals)) / len(err_free_vals)

    # assumes sensor readings are consistent, first sensor initial reading is accurate
    # could modify past value to be the average of the past few outputs (store in list)
    def getTrusted(self, pastValue, vals):
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
