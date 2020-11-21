#! /usr/bin/env python
import rospy
from sailbot_msg.msg import Sensors, windSensor, GPS

CHECK_PERIOD_SECONDS = 0.1


class RosInterface:
    def __init__(self):
        rospy.init_node('ros_interface', anonymous=True)
        rospy.Subscriber("sensors", Sensors, self.sensorsCallback)
        self.pubWind = rospy.Publisher('windSensor', windSensor, queue_size=4)
        self.pubGPS = rospy.Publisher('GPS', GPS, queue_size=4)

        sensorMsg = rospy.wait_for_message('sensors', Sensors)
        self.wind_speed = sensorMsg.wind_sensor_0_speed
        self.wind_direction = sensorMsg.wind_sensor_0_direction
        self.gps_lat = sensorMsg.gps_0_latitude
        self.gps_lon = sensorMsg.gps_0_longitude
        self.gps_headingDegrees = None
        self.gps_speedKmph = sensorMsg.gps_0_groundspeed

    def sensorsCallback(self, data):
        # int32 -> float64 precision matters?
        self.wind_speed = data.wind_sensor_0_speed  # wind sensor
        self.wind_direction = data.wind_sensor_0_direction
        # self.wind_sensor_reference = wind_sensor_0_reference -- necessary?
        # how about other wind sensors?

        self.gps_lat = data.gps_0_latitude
        self.gps_lon = data.gps_0_longitude
        self.gps_headingDegrees = None  # maps to what?
        self.gps_speedKmph = data.gps_0_groundspeed

        # how about other gps fields?
        # how about other gps sensors?

        # need to incorporate accelerometer?

    def pub(self):
        self.pubGPS.publish(self.gps_lat, self.gps_lon, self.gps_headingDegrees, self.gps_speedKmph)
        self.pubWind.publish(self.wind_direction, self.wind_speed)


if __name__ == "__main__":
    rosInterface = RosInterface()
    rate = rospy.Rate(1 / CHECK_PERIOD_SECONDS)
    while not rospy.is_shutdown():
        rosInterface.pub()
        rate.sleep()
