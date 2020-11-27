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
# - heading_degrees: ??? -> (0 is east, 90 is north, etc - ccw - degrees)
#     - magnetic variation: angle between magnetic and true north, where east difference is positive - cw - degrees


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
        translate()
        self.pubGPS.publish(self.gps_lat, self.gps_lon, self.gps_headingDegrees, self.gps_speedKmph)
        self.pubWind.publish(self.wind_direction, self.wind_speed)

    # Translate from many sensor outputs to one signal, checking error margin, averaging, and converting units
    def translate(self):
        # GPS sensors (missing headingDegrees)
        self.gps_lat = checkErrAndAvg([self.data.gps_0_latitude, self.data.gps_1_latitude])
        self.gps_lon = checkErrAndAvg([self.data.gps_0_longitude, self.data.gps_1_longitude])
        self.gps_speedKmph = checkErrAndAvg([KNOTS_TO_KMPH * self.data.gps_0_groundspeed,
                                             KNOTS_TO_KMPH * self.data.gps_1_groundspeed])

        # Wind sensors
        self.wind_speed = checkErrAndAvg([KNOTS_TO_KMPH * self.data.wind_sensor_0_speed,
                                          KNOTS_TO_KMPH * self.data.wind_sensor_1_speed,
                                          KNOTS_TO_KMPH * self.data.wind_sensor_2_speed])
        self.wind_direction = checkErrAndAvg([self.data.wind_sensor_0_direction,
                                              self.data.wind_sensor_1_direction,
                                              self.data.wind_sensor_2_direction])

    def checkErrAndAvg(self, arr):
        err_free_arr = checkError(arr)
        return float(sum(err_free_arr)) / len(err_free_arr)

    def checkError(self, arr):
        # define constants for each signal's error of margin (compare sensor data)
        # if within error of margin, average values
        # returns array
        # modifies a boolean if error is found?


if __name__ == "__main__":
    rosInterface = RosInterface()
    rate = rospy.Rate(1 / CHECK_PERIOD_SECONDS)
    while not rospy.is_shutdown():
        rosInterface.pub()
        rate.sleep()
