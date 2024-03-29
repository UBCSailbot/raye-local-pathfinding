#!/usr/bin/env python
import numpy as np
import rospy

import sailbot_msg.msg as msg
import geopy.distance
from utilities import (
    flipDirection,
    headingToBearingDegrees,
    bearingToHeadingDegrees,
    PORT_RENFREW_LATLON,
    globalWindToMeasuredWind,
    get_rosparam_or_default_if_invalid,
)


# Constants
KNOTS_TO_KMPH = 1.852
SENSORS_PUBLISH_PERIOD_SECONDS = 0.1  # Keep below 1.0 for smoother boat motion
START_BOAT_SPEED_KMPH = 14.4  # Boat should move at about 4m/s = 14.4 km/h
START_BOAT_HEADING_DEGREES = 180
START_GLOBAL_WIND_DIRECTION_DEGREES = 0
START_GLOBAL_WIND_SPEED_KMPH = 10
STDEV_GPS = 0.00001
STDEV_WIND = 0.1


class MOCK_SensorManager:
    def __init__(self, startLat, startLon, startHeadingDegrees, startSpeedKmph, startGlobalWindSpeedKmph,
                 startGlobalWindDirectionDegrees):
        # Initialize starting values
        self.lat = startLat
        self.lon = startLon
        self.headingDegrees = startHeadingDegrees
        self.speedKmph = startSpeedKmph
        self.globalWindSpeedKmph = startGlobalWindSpeedKmph
        self.globalWindDirectionDegrees = startGlobalWindDirectionDegrees
        self.measuredWindSpeedKmph, self.measuredWindDirectionDegrees = globalWindToMeasuredWind(
            self.globalWindSpeedKmph, self.globalWindDirectionDegrees, self.speedKmph, self.headingDegrees)
        self.publishPeriodSeconds = SENSORS_PUBLISH_PERIOD_SECONDS

        # Setup ROS node inputs and outputs
        self.publisher = rospy.Publisher("sensors", msg.Sensors, queue_size=4)
        rospy.Subscriber("desired_heading_degrees", msg.heading, self.desiredHeadingCallback)

        # Inputs for testing
        rospy.Subscriber("changeGPS", msg.GPS, self.changeGPSCallback)

    def update(self):
        # TODO: Use rospy.get_param('smooth_changes', default=True) to use PID controller on heading and speed
        speedup = rospy.get_param('speedup', default=1.0)

        # Travel based on boat speed
        kmTraveledPerPeriod = self.speedKmph * self.publishPeriodSeconds / 3600.0
        kmTraveledPerPeriod *= speedup  # Move greater distances with speedup
        distanceTraveled = geopy.distance.distance(kilometers=kmTraveledPerPeriod)
        destination = distanceTraveled.destination(point=(self.lat, self.lon),
                                                   bearing=headingToBearingDegrees(self.headingDegrees))
        self.lon = destination.longitude
        self.lat = destination.latitude

        # Travel based on ocean current
        oceanCurrentSpeedKmph = rospy.get_param('ocean_current_speed', default=0.0)
        oceanCurrentDirectionDegress = rospy.get_param('ocean_current_direction', default=0.0)
        oceanCurrentKmTraveledPerPeriod = oceanCurrentSpeedKmph * self.publishPeriodSeconds / 3600.0
        oceanCurrentKmTraveledPerPeriod *= speedup  # Move greater distances with speedup
        distanceTraveled = geopy.distance.distance(kilometers=oceanCurrentKmTraveledPerPeriod)
        destination = distanceTraveled.destination(point=(self.lat, self.lon),
                                                   bearing=headingToBearingDegrees(oceanCurrentDirectionDegress))
        self.lon = destination.longitude
        self.lat = destination.latitude

        # Update wind
        self.globalWindSpeedKmph = rospy.get_param('global_wind_speed_kmph', default=self.globalWindSpeedKmph)
        self.globalWindDirectionDegrees = rospy.get_param('global_wind_direction_degrees',
                                                          default=self.globalWindDirectionDegrees)
        self.measuredWindSpeedKmph, self.measuredWindDirectionDegrees = globalWindToMeasuredWind(
            self.globalWindSpeedKmph, self.globalWindDirectionDegrees, self.speedKmph, self.headingDegrees)

    def publish(self):
        # Populate sensors, leaving commented out the ones we don't know for now
        data = msg.Sensors()

        # data.sailencoder_degrees

        data.wind_sensor_1_speed_knots = self.measuredWindSpeedKmph / KNOTS_TO_KMPH
        data.wind_sensor_1_angle_degrees = headingToBearingDegrees(self.measuredWindDirectionDegrees)
        data.wind_sensor_1_angle_degrees = flipDirection(data.wind_sensor_1_angle_degrees)
        data.wind_sensor_2_speed_knots = self.measuredWindSpeedKmph / KNOTS_TO_KMPH
        data.wind_sensor_2_angle_degrees = headingToBearingDegrees(self.measuredWindDirectionDegrees)
        data.wind_sensor_2_angle_degrees = flipDirection(data.wind_sensor_2_angle_degrees)
        data.wind_sensor_3_speed_knots = self.measuredWindSpeedKmph / KNOTS_TO_KMPH
        data.wind_sensor_3_angle_degrees = headingToBearingDegrees(self.measuredWindDirectionDegrees)
        data.wind_sensor_3_angle_degrees = flipDirection(data.wind_sensor_3_angle_degrees)

        # data.gps_can_timestamp_utc
        data.gps_can_latitude_degrees = self.lat
        data.gps_can_longitude_degrees = self.lon
        data.gps_can_groundspeed_knots = self.speedKmph / KNOTS_TO_KMPH
        data.gps_can_track_made_good_degrees = headingToBearingDegrees(self.headingDegrees)
        data.gps_can_true_heading_degrees = headingToBearingDegrees(self.headingDegrees)
        # data.gps_can_magnetic_variation_degrees
        # data.gps_can_state

        # data.gps_ais_timestamp_utc
        data.gps_ais_latitude_degrees = self.lat
        data.gps_ais_longitude_degrees = self.lon
        data.gps_ais_groundspeed_knots = self.speedKmph / KNOTS_TO_KMPH
        data.gps_ais_track_made_good_degrees = headingToBearingDegrees(self.headingDegrees)
        data.gps_ais_true_heading_degrees = headingToBearingDegrees(self.headingDegrees)
        # data.gps_ais_magnetic_variation_degrees
        # data.gps_ais_state

        # data.winch_main_angle_degrees
        # data.winch_jib_angle_degrees
        # data.rudder_port_angle_degrees
        # data.rudder_stbd_angle_degrees

        # data.accelerometer_x_force_millig
        # data.accelerometer_y_force_millig
        # data.accelerometer_z_force_millig

        # data.gyroscope_x_velocity_millidegreesps
        # data.gyroscope_y_velocity_millidegreesps
        # data.gyroscope_z_velocity_millidegreesps

        if rospy.get_param('sensor_noise', default=False):
            data = self.add_noise(data)
        self.publisher.publish(data)

    def add_noise(self, data):
        # Add noise to sensor data using numpy.random.normal. GPS and wind sensors have different standard deviations,
        # with GPS readings being much more accurate and consistent. Unused sensors are commented out.
        noisy_data = msg.Sensors()

        # data.sailencoder_degrees

        noisy_data.wind_sensor_1_speed_knots = np.random.normal(data.wind_sensor_1_speed_knots, STDEV_WIND)
        noisy_data.wind_sensor_1_angle_degrees = np.random.normal(data.wind_sensor_1_angle_degrees, STDEV_WIND)
        noisy_data.wind_sensor_2_speed_knots = np.random.normal(data.wind_sensor_2_speed_knots, STDEV_WIND)
        noisy_data.wind_sensor_2_angle_degrees = np.random.normal(data.wind_sensor_2_angle_degrees, STDEV_WIND)
        noisy_data.wind_sensor_3_speed_knots = np.random.normal(data.wind_sensor_3_speed_knots, STDEV_WIND)
        noisy_data.wind_sensor_3_angle_degrees = np.random.normal(data.wind_sensor_3_angle_degrees, STDEV_WIND)

        # data.gps_can_timestamp_utc
        noisy_data.gps_can_latitude_degrees = np.random.normal(data.gps_can_latitude_degrees, STDEV_GPS)
        noisy_data.gps_can_longitude_degrees = np.random.normal(data.gps_can_longitude_degrees, STDEV_GPS)
        noisy_data.gps_can_groundspeed_knots = np.random.normal(data.gps_can_groundspeed_knots, STDEV_GPS)
        noisy_data.gps_can_track_made_good_degrees = np.random.normal(data.gps_can_track_made_good_degrees, STDEV_GPS)
        noisy_data.gps_can_true_heading_degrees = np.random.normal(data.gps_can_true_heading_degrees, STDEV_GPS)
        # data.gps_can_magnetic_variation_degrees
        # data.gps_can_state

        noisy_data.gps_ais_latitude_degrees = np.random.normal(data.gps_ais_latitude_degrees, STDEV_GPS)
        noisy_data.gps_ais_longitude_degrees = np.random.normal(data.gps_ais_longitude_degrees, STDEV_GPS)
        noisy_data.gps_ais_groundspeed_knots = np.random.normal(data.gps_ais_groundspeed_knots, STDEV_GPS)
        noisy_data.gps_ais_track_made_good_degrees = np.random.normal(data.gps_ais_track_made_good_degrees, STDEV_GPS)
        noisy_data.gps_ais_true_heading_degrees = np.random.normal(data.gps_ais_true_heading_degrees, STDEV_GPS)
        # data.gps_ais_magnetic_variation_degrees
        # data.gps_ais_state

        # data.winch_main_angle_degrees
        # data.winch_jib_angle_degrees
        # data.rudder_port_angle_degrees
        # data.rudder_stbd_angle_degrees

        # data.accelerometer_x_force_millig
        # data.accelerometer_y_force_millig
        # data.accelerometer_z_force_millig

        # data.gyroscope_x_velocity_millidegreesps
        # data.gyroscope_y_velocity_millidegreesps
        # data.gyroscope_z_velocity_millidegreesps

        return noisy_data

    def desiredHeadingCallback(self, data):
        # rostopic uses North=0, East=90
        # Internal local_pathfinding uses East=0, North=90
        self.headingDegrees = bearingToHeadingDegrees(data.headingDegrees)

    def changeGPSCallback(self, data):
        rospy.loginfo("Received change GPS message = {}".format(data))
        self.lat = data.lat
        self.lon = data.lon
        self.headingDegrees = data.headingDegrees
        self.speedKmph = data.speedKmph


if __name__ == '__main__':
    rospy.init_node('MOCK_SensorManager', anonymous=True)
    startLat = get_rosparam_or_default_if_invalid('start_lat', default=PORT_RENFREW_LATLON.lat)
    startLon = get_rosparam_or_default_if_invalid('start_lon', default=PORT_RENFREW_LATLON.lon)
    sensorManager = MOCK_SensorManager(startLat=startLat, startLon=startLon,
                                       startHeadingDegrees=START_BOAT_HEADING_DEGREES,
                                       startSpeedKmph=START_BOAT_SPEED_KMPH,
                                       startGlobalWindSpeedKmph=START_GLOBAL_WIND_SPEED_KMPH,
                                       startGlobalWindDirectionDegrees=START_GLOBAL_WIND_DIRECTION_DEGREES)
    r = rospy.Rate(float(1) / SENSORS_PUBLISH_PERIOD_SECONDS)

    while not rospy.is_shutdown():
        sensorManager.update()
        sensorManager.publish()
        r.sleep()
