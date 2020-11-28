import rospy
from sailbot_msg.msg import Sensors, windSensor, GPS

# Constants
CHECK_PERIOD_SECONDS = 0.1  # How often fields are updated
KNOTS_TO_KMPH = 1.852
ERROR = 0.2  # If signals need different margin of errors, modify as required

# Conversion Notes:
# - wind_speedKmph, wind_direction, wind_headingDegrees, gps_speedKmph: int32 -> float64 (no loss of precision)
# - wind_speedKmph, gps_speedKmph: knots -> km/h
# - wind_direction: no conversion required (0 is bow, 90 is starboard/right, etc - cw - degrees)
# - heading_degrees: (0 is north, 90 is east, etc - cw - degrees) -> (0 is east, 90 is north, etc - ccw - degrees)


class RosInterface:
    def __init__(self):
        rospy.init_node('ros_interface', anonymous=True)
        rospy.Subscriber("sensors", Sensors, self.sensorsCallback)
        self.pubWind = rospy.Publisher('windSensor', windSensor, queue_size=4)
        self.pubGPS = rospy.Publisher('GPS', GPS, queue_size=4)

        # sensorMsg = rospy.wait_for_message('sensors', Sensors)
        self.wind_speedKmph = 0
        self.wind_direction = 0
        self.gps_lat = 0
        self.gps_lon = 0
        self.gps_headingDegrees = None
        self.gps_speedKmph = 0

    def sensorsCallback(self, data):
        self.data = data

    def pub(self):
        self.translate()
        self.pubGPS.publish(self.gps_lat, self.gps_lon, self.gps_headingDegrees, self.gps_speedKmph)
        self.pubWind.publish(self.wind_direction, self.wind_speed)

    # Translate from many sensor outputs to one signal, checking error margin, averaging, and converting units
    def translate(self):
        # GPS sensors
        self.gps_lat = self.checkErrAndAvg([self.data.gps_0_latitude, self.data.gps_1_latitude])
        self.gps_lon = self.checkErrAndAvg([self.data.gps_0_longitude, self.data.gps_1_longitude])
        self.gps_headingDegrees = self.checkErrAndAvg([self.convertDegrees(self.data.gps_0_true_heading),
                                                       self.convertDegrees(self.data.gps_1_true_heading)])
        self.gps_speedKmph = self.checkErrAndAvg([KNOTS_TO_KMPH * self.data.gps_0_groundspeed,
                                                  KNOTS_TO_KMPH * self.data.gps_1_groundspeed])

        # Wind sensors
        self.wind_speed = self.checkErrAndAvg([KNOTS_TO_KMPH * self.data.wind_sensor_0_speed,
                                               KNOTS_TO_KMPH * self.data.wind_sensor_1_speed,
                                               KNOTS_TO_KMPH * self.data.wind_sensor_2_speed])
        self.wind_direction = self.checkErrAndAvg([self.data.wind_sensor_0_direction,
                                                   self.data.wind_sensor_1_direction,
                                                   self.data.wind_sensor_2_direction])

    # (0 is north, 90 is east, etc - cw - degrees) -> (0 is east, 90 is north, etc - ccw - degrees)
    def convertDegrees(self, degree):
        converted = -1 * degree + 90
        return converted if converted >= 0 else converted + 360

    def checkErrAndAvg(self, arr):
        # err_free_arr = checkError(arr)
        err_free_arr = arr
        return float(sum(err_free_arr)) / len(err_free_arr)

    # def checkError(self, arr):
    #     # input is an array of values
    #     # returns array with values outside error of margin removed
    #     # possible to modify a boolean if error is found?


if __name__ == "__main__":
    rosInterface = RosInterface()
    rate = rospy.Rate(1 / CHECK_PERIOD_SECONDS)
    while not rospy.is_shutdown():
        rosInterface.pub()
        rate.sleep()
