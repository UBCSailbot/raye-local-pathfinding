#!/usr/bin/env python
import numpy as np
import rospy
from sailbot_msg.msg import Sensors, windSensor, GPS, globalWind
from utilities import bearingToHeadingDegrees, measuredWindToGlobalWind, boundTo180, absDiffBetweenAngles, averageAngles


# Constants
CHECK_PERIOD_SECONDS = 0.1  # How often fields are updated
KNOTS_TO_KMPH = 1.852
OLDEST_PAST_VAL_AGE_SECONDS = 10
GPS_SENSOR_FREQ_HZ = 1
WIND_SENSOR_FREQ_HZ = 2

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

        # Current value
        self.gps_lat_decimalDegrees = None
        self.gps_lon_decimalDegrees = None
        self.gps_headingDegrees = None
        self.gps_speedKmph = None
        self.measured_wind_speedKmph = None
        self.measured_wind_direction = None

        # Past state
        self.pastState_gps_lat_decimalDegrees = self.initPastState()
        self.pastState_gps_lon_decimalDegrees = self.initPastState()
        self.pastState_gps_headingDegrees = self.initPastState()
        self.pastState_gps_speedKmph = self.initPastState()
        self.pastState_measured_wind_speedKmph = self.initPastState()
        self.pastState_measured_wind_direction = self.initPastState()

        # Maximum length of past values for each sensor field such that the oldest value < OLDEST_PAST_VAL_AGE_SECONDS
        self.gps_pastVal_max_len = OLDEST_PAST_VAL_AGE_SECONDS * GPS_SENSOR_FREQ_HZ
        self.wind_pastVal_max_len = OLDEST_PAST_VAL_AGE_SECONDS * WIND_SENSOR_FREQ_HZ

    def initPastState(self):
        '''Initializes the self.pastState_* instance variables, one for each current value instance variable
            - vals: list of the previous few current values, limited by self.*_pastVal_max_len
            - brokenSensors: list of sensor indices in currVals of filterSensors() that were excluded from the
                             calculation of the latest current value
        '''
        return {'vals': [], 'brokenSensors': []}

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

    def translate(self):
        '''Translate from many sensor outputs to one signal, converting units, checking for errors, and averaging.'''
        # GPS sensors - only use CAN for now
        rospy.loginfo('Filtering GPS latitude')
        self.gps_lat_decimalDegrees = self.filterSensors(
                pastState=self.pastState_gps_lat_decimalDegrees,
                currVals=[self.data.gps_can_latitude_degrees],
                maxAbsErr=0.5, isGPS=True, isHeading=False)
        rospy.loginfo('Filtering GPS longitude')
        self.gps_lon_decimalDegrees = self.filterSensors(
                pastState=self.pastState_gps_lon_decimalDegrees,
                currVals=[self.data.gps_can_longitude_degrees],
                maxAbsErr=0.5, isGPS=True, isHeading=False)
        rospy.loginfo('Filtering GPS heading')
        self.gps_headingDegrees = self.filterSensors(
                pastState=self.pastState_gps_headingDegrees,
                currVals=[bearingToHeadingDegrees(self.data.gps_can_true_heading_degrees)],
                maxAbsErr=5.0, isGPS=True, isHeading=True)
        rospy.loginfo('Filtering GPS speed')
        self.gps_speedKmph = self.filterSensors(
                pastState=self.pastState_gps_speedKmph,
                currVals=[self.data.gps_can_groundspeed_knots * KNOTS_TO_KMPH],
                maxAbsErr=2.0, isGPS=True, isHeading=False)

        # Wind sensors
        rospy.loginfo('Filtering wind speed')
        self.measured_wind_speedKmph = self.filterSensors(
                pastState=self.pastState_measured_wind_speedKmph,
                currVals=[self.data.wind_sensor_1_speed_knots * KNOTS_TO_KMPH,
                          self.data.wind_sensor_2_speed_knots * KNOTS_TO_KMPH,
                          self.data.wind_sensor_3_speed_knots * KNOTS_TO_KMPH],
                maxAbsErr=5.0, isGPS=False, isHeading=False)
        rospy.loginfo('Filtering wind direction')
        self.measured_wind_direction = self.filterSensors(
                pastState=self.pastState_measured_wind_direction,
                currVals=[bearingToHeadingDegrees(self.data.wind_sensor_1_angle_degrees),
                          bearingToHeadingDegrees(self.data.wind_sensor_2_angle_degrees),
                          bearingToHeadingDegrees(self.data.wind_sensor_3_angle_degrees)],
                maxAbsErr=10.0, isGPS=False, isHeading=True)

    def filterSensors(self, pastState, currVals, maxAbsErr, isGPS, isHeading):
        '''Compares all currVals that are not from broken sensors to its median.
            - If currVal is broken, do not include in the calculation of nextCurrVal
            - If abs(currVal - median) > maxAbsErr, the sensor of currVal is added to broken sensors and
              currVal is not included in the calculation of nextCurrVal
            - nextCurrVal is the average of the remaining currVals, including the median
            - If isHeading is True, the field being filtered is a heading
        '''
        if isHeading:
            rospy.loginfo('Raw heading sensor data: {}'.format(currVals))
            currVals = [boundTo180(a) for a in currVals]
            rospy.loginfo('Heading sensor data bounded to 180: {}'.format(currVals))

        # Compute nextBrokenSensors and nextCurrVals by comparing currVals to its median
        median = np.median(currVals)
        nextBrokenSensors = pastState['brokenSensors']
        rospy.loginfo('Initial broken sensors: {}'.format(nextBrokenSensors))
        checkedCurrVals = []
        rospy.loginfo('Initial current values = {}'.format(currVals))

        for i in range(len(currVals)):
            if i in nextBrokenSensors:
                continue  # do not add to workingCurrVals
            else:
                absErr = absDiffBetweenAngles(currVals[i], median) if isHeading else abs(currVals[i] - median)
                if absErr > maxAbsErr:
                    rospy.logwarn('Erratic sensor data: Sensor {} ({}) varies greatly from the median ({})'
                                  .format(i, currVals[i], median))
                    nextBrokenSensors.append(i)
                    rospy.logwarn('Marking Sensor {} as broken; will not use for this field moving forwards'.format(i))
                else:
                    checkedCurrVals.append(currVals[i])

        rospy.loginfo('nextBrokenSensors = {}'.format(nextBrokenSensors))
        rospy.loginfo('checkedCurrVals = {}'.format(checkedCurrVals))

        # Compute nextCurrVals by averaging past values with the average of checkedCurrVals if it exists
        if len(checkedCurrVals) == 0:
            rospy.logfatal('There are no working sensors; nextCurrVal will be an average of the past current values')
            nextCurrVals = pastState['vals']
        else:
            checkedCurrValsAvg = averageAngles(checkedCurrVals) if isHeading else np.average(checkedCurrVals)
            nextCurrVals = pastState['vals'] + [checkedCurrValsAvg]
        nextCurrVal = averageAngles(nextCurrVals) if isHeading else np.average(nextCurrVals)
        rospy.loginfo('nextCurrVal = average{} = {}' .format(nextCurrVals, nextCurrVal))

        # Update the past values list, factoring in pastValMaxLen
        rospy.loginfo('Initial pastState["vals"]: {}'.format(pastState['vals']))
        pastValMaxLen = self.gps_pastVal_max_len if isGPS else self.wind_pastVal_max_len
        if len(pastState['vals']) == pastValMaxLen:
            pastState['vals'] = pastState['vals'][1:] + [nextCurrVal]
        else:
            pastState['vals'] = pastState['vals'] + [nextCurrVal]
        rospy.loginfo('Updated pastState["vals"] to {}'.format(pastState['vals']))

        return nextCurrVal

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
