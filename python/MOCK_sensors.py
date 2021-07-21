#!/usr/bin/env python
import rospy

import sailbot_msg.msg as msg
import geopy.distance
from utilities import headingToBearingDegrees, bearingToHeadingDegrees, PORT_RENFREW_LATLON, globalWindToMeasuredWind


# Constants
KNOTS_TO_KMPH = 1.852
SENSORS_PUBLISH_PERIOD_SECONDS = 0.1  # Keep below 1.0 for smoother boat motion
START_BOAT_SPEED_KMPH = 14.4  # Boat should move at about 4m/s = 14.4 km/h
START_BOAT_HEADING_DEGREES = 180
START_GLOBAL_WIND_DIRECTION_DEGREES = 0
START_GLOBAL_WIND_SPEED_KMPH = 10


def getStartLatLon():
    # Read in startLat and startLon
    try:
        startLat = rospy.get_param('start_lat', default=PORT_RENFREW_LATLON.lat)
        startLat = float(startLat)
    except ValueError:
        rospy.logwarn("Invalid startLat must be a float, but received startLat = {}".format(startLat))
        startLat = PORT_RENFREW_LATLON.lat
        rospy.logwarn("Defaulting to startLat = {}".format(startLat))
    try:
        startLon = rospy.get_param('start_lon', default=PORT_RENFREW_LATLON.lon)
        startLon = float(startLon)
    except ValueError:
        rospy.logwarn("Invalid startLon must be a float, but received startLon = {}".format(startLon))
        startLon = PORT_RENFREW_LATLON.lon
        rospy.logwarn("Defaulting to startLon = {}".format(startLon))
    return (startLat, startLon)


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
        rospy.init_node('MOCK_SensorManager', anonymous=True)
        self.publisher = rospy.Publisher("sensors", msg.Sensors, queue_size=4)
        rospy.Subscriber("heading_degrees", msg.heading, self.desiredHeadingCallback)

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
        data.wind_sensor_2_speed_knots = self.measuredWindSpeedKmph / KNOTS_TO_KMPH
        data.wind_sensor_2_angle_degrees = headingToBearingDegrees(self.measuredWindDirectionDegrees)
        data.wind_sensor_3_speed_knots = self.measuredWindSpeedKmph / KNOTS_TO_KMPH
        data.wind_sensor_3_angle_degrees = headingToBearingDegrees(self.measuredWindDirectionDegrees)

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

        self.publisher.publish(data)

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
    startLat, startLon = getStartLatLon()
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
