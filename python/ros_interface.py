import rospy
import numpy
from sailbot_msg.msg import Sensors, windSensor, GPS

# Constants
CHECK_PERIOD_SECONDS = 0.1  # How often fields are updated
KNOTS_TO_KMPH = 1.852
ERROR = 0.2  # If signals need different margin of errors, modify as required

"""
Conversion Notes:
- wind_speedKmph, wind_direction, wind_headingDegrees, gps_speedKmph: int32 -> float64 (no loss of precision)
- wind_speedKmph, gps_speedKmph: knots -> km/h
- wind_direction: no conversion required (0 is bow, 90 is starboard/right, etc - cw - degrees)
- heading_degrees: (0 is north, 90 is east, etc - cw - degrees) -> (0 is east, 90 is north, etc - ccw - degrees)
"""


class RosInterface:
    def __init__(self):
        rospy.init_node('ros_interface', anonymous=True)
        rospy.Subscriber("sensors", Sensors, self.sensorsCallback)
        self.pubWind = rospy.Publisher('windSensor', windSensor, queue_size=4)
        self.pubGPS = rospy.Publisher('GPS', GPS, queue_size=4)
        self.initialized = False

        self.wind_speedKmph = 0
        self.wind_direction = 0
        self.gps_lat = 0
        self.gps_lon = 0
        self.gps_headingDegrees = 0
        self.gps_speedKmph = 0

        self.past_wind_speedKmph = 0
        self.past_wind_direction = 0
        self.past_gps_lat = 0
        self.past_gps_lon = 0
        self.past_gps_headingDegrees = 0
        self.past_gps_speedKmph = 0

    def sensorsCallback(self, data):
        if not self.initialized:
            self.initialized = True
        self.data = data

    def pub(self):
        if self.initialized:
            self.translate()
            self.pubGPS.publish(self.gps_lat, self.gps_lon, self.gps_headingDegrees, self.gps_speedKmph)
            self.pubWind.publish(self.wind_direction, self.wind_speedKmph)

    # Translate from many sensor outputs to one signal, checking error margin, averaging, and converting units
    def translate(self):
        # Past values
        self.past_gps_lat = self.gps_lat
        self.past_gps_lon = self.gps_lon
        self.past_gps_headingDegrees = self.gps_headingDegrees
        self.past_gps_speedKmph = self.gps_speedKmph
        self.past_wind_speedKmph = self.wind_speedKmph
        self.past_wind_direction = self.wind_direction

        # GPS sensors
        self.gps_lat = self.checkErrAndAvg([self.past_gps_lat,
                                            self.data.gps_0_latitude, self.data.gps_1_latitude])
        self.gps_lon = self.checkErrAndAvg([self.past_gps_lon,
                                            self.data.gps_0_longitude, self.data.gps_1_longitude])
        self.gps_headingDegrees = self.checkErrAndAvg([self.past_gps_headingDegrees,
                                                       self.convertDegrees(self.data.gps_0_true_heading),
                                                       self.convertDegrees(self.data.gps_1_true_heading)])
        self.gps_speedKmph = self.checkErrAndAvg([self.past_gps_speedKmph,
                                                  self.data.gps_0_groundspeed * KNOTS_TO_KMPH,
                                                  self.data.gps_1_groundspeed * KNOTS_TO_KMPH])
        # Wind sensors
        self.wind_speedKmph = self.checkErrAndAvg([self.past_wind_speedKmph,
                                                   self.data.wind_sensor_0_speed * KNOTS_TO_KMPH,
                                                   self.data.wind_sensor_1_speed * KNOTS_TO_KMPH,
                                                   self.data.wind_sensor_2_speed * KNOTS_TO_KMPH])
        self.wind_direction = self.checkErrAndAvg([self.past_wind_direction,
                                                   self.data.wind_sensor_0_direction,
                                                   self.data.wind_sensor_1_direction,
                                                   self.data.wind_sensor_2_direction])

    # (0 is north, 90 is east, etc - cw - degrees) -> (0 is east, 90 is north, etc - ccw - degrees)
    def convertDegrees(self, degree):
        converted = -1 * degree + 90
        return converted if converted >= 0 else converted + 360

    def checkErrAndAvg(self, arr):
        err_free_arr = self.checkError(arr)
        return float(sum(err_free_arr)) / len(err_free_arr)

    # past value is first element of arr
    # might causes errors: returns empty array, what if sensor readings aren't consistent
    # could modify past value to be the average of the past few outputs (store in array)
    def checkError(self, arr):
        pastValue = arr[0]
        presentValues = arr - arr[0]

        errorFreeList = list()
        for value in presentValues:
            if abs(float((pastValue - value)) / pastValue) < pastValue * ERROR:
                errorFreeList.append(value)

        return numpy.array(errorFreeList)


if __name__ == "__main__":
    rosInterface = RosInterface()
    rate = rospy.Rate(1 / CHECK_PERIOD_SECONDS)
    while not rospy.is_shutdown():
        rosInterface.pub()
        rate.sleep()
