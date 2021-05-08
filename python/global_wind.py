#! /usr/bin/env python
import rospy
import math
from sailbot_msg.msg import GPS, windSensor, globalWind
from MOCK_AIS import AIS_PUBLISH_PERIOD_SECONDS

CHECK_PERIOD = AIS_PUBLISH_PERIOD_SECONDS


class GlobalWind:
    def __init__(self):
        '''
        measuredWindSpeed (float): speed of the wind measured from the boat. All speed values must be in the same units.
        measuredWindDirectionDegrees (float): angle of the measured with wrt the boat.
                                             0 degrees is wind blowing to the right. 90 degrees is wind blowing forward.
        boatSpeed (float): speed of the boat
        headingDegrees (float): angle of the boat in global frame. 0 degrees is East. 90 degrees is North.
       '''
        self.measuredWindSpeed = None
        self.measuredWindDirection = None
        self.boatSpeed = None
        self.headingDegrees = None

        rospy.Subscriber("GPS", GPS, self.GPSCallback)
        rospy.Subscriber("windSensor", windSensor, self.windSensorCallback)

    def GPSCallback(self, data):
        self.headingDegrees = data.headingDegrees
        self.boatSpeed = data.speedKmph

    def windSensorCallback(self, data):
        self.measuredWindDirection = data.measuredDirectionDegrees
        self.measuredWindSpeed = data.measuredSpeedKmph

    def measuredWindToGlobalWind(self):
        '''Calculate the global wind based on the measured wind and the boat velocity

        Returns:
        float, float pair representing the globalWindSpeed (same units as input speed), globalWindDirectionDegrees
        '''
        measuredWindRadians = math.radians(self.measuredWindDirectionDegrees)
        headingRadians = math.radians(self.headingDegrees)

        # GF = global frame. BF = boat frame
        # Calculate wind speed in boat frame. X is right. Y is forward.
        measuredWindSpeedXBF = self.measuredWindSpeed * math.cos(measuredWindRadians)
        measuredWindSpeedYBF = self.measuredWindSpeed * math.sin(measuredWindRadians)

        # Assume boat is moving entirely in heading direction, so all boat speed is in boat frame Y direction
        trueWindSpeedXBF = measuredWindSpeedXBF
        trueWindSpeedYBF = measuredWindSpeedYBF + self.boatSpeed

        # Calculate wind speed in global frame. X is EAST. Y is NORTH.
        trueWindSpeedXGF = trueWindSpeedXBF * math.sin(headingRadians) + trueWindSpeedYBF * math.cos(headingRadians)
        trueWindSpeedYGF = trueWindSpeedYBF * math.sin(headingRadians) - trueWindSpeedXBF * math.cos(headingRadians)

        # Calculate global wind speed and direction
        globalWindSpeed = (trueWindSpeedXGF**2 + trueWindSpeedYGF**2)**0.5
        globalWindDirectionDegrees = math.degrees(math.atan2(trueWindSpeedYGF, trueWindSpeedXGF))

        return globalWindSpeed, globalWindDirectionDegrees


if __name__ == '__main__':
    global_wind_conversion = GlobalWind()
    global_wind_pub = rospy.Publisher('global_wind', globalWind, queue_size=4)
    rate = rospy.Rate(1 / CHECK_PERIOD)

    while not rospy.is_shutdown():
        speed, direction = global_wind_conversion.measuredWindToGlobalWind()
        global_wind_pub.publish(globalWind(directionDegrees=direction, speedKmph=speed))
        rate.sleep()
