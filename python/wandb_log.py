#!/usr/bin/env python
import wandb
import rospy
import os
import numpy as np
import time
from log_closest_obstacle import LogClosestObstacle
from path_storer import PathStorer
from collision_checker import CollisionChecker
from store_sailbot_gps import SailbotGPSData
from utilities import takeScreenshot, getShipsSortedByDistance
from PIL import Image
import Sailbot as sbot
import sailbot_msg.msg as msg

# Parameters for what and how to log
UPDATE_TIME_SECONDS = 1.0
SCATTER_PLOTS_PERIOD_SECONDS = 60
SCREENSHOT_PERIOD_SECONDS = 10
LATLON_TABLE = False
SCREENSHOT = False
LOG_SENSORS = True


# Globals for subscribing
sensor_data = None
targetGlobalLatLon = None


# Globals for wind sensor filtering
WIND_SPEED_FIELDS = ['wind_sensor_{}_speed_knots'.format(i) for i in range(1, 4)]
WIND_DIRECTION_FIELDS = ['wind_sensor_{}_angle_degrees'.format(i) for i in range(1, 4)]
WIND_SPEED_THRESHOLD = 2.0  # wind direction is set to 0 if its speed is below the threshold


def sensorsCallback(data):
    global sensor_data
    sensor_data = data


def nextGlobalWaypointCallback(newTargetGlobalLatLon):
    global targetGlobalLatLon
    targetGlobalLatLon = newTargetGlobalLatLon


def sensorFilters(sensor_data_dict):
    """Returns a dictionary with sensor data filtered in various ways.

    Note:
        Handles angles wrapping around; for example, average(359, 1) is ~0.0 (float equality)

    Statistics:
        Standard deviation

    Filters:
        General: '-' at beginning of the line means not implemented
            Mean (for angle, use vectorAverage() with magnitudes being all 1's)
            Vector average
            -Time average
            -Extrapolation (linear, polynomial, cubic spline)
            -Convolution filter?
        Wind angle-specific:
            Threshold
        Wind speed-specific:
            Median

    Args:
        sensor_data_dict (dict): { <sensor field name>: <sensor field value> }
    """
    filtered_values_dict = {}

    # wind speed filters/stats
    wind_speeds = [sensor_data_dict[field] for field in WIND_SPEED_FIELDS]
    filtered_values_dict['wind_sensor_median_speed_knots'] = np.median(wind_speeds)
    filtered_values_dict['wind_sensor_mean_speed_knots'] = np.mean(wind_speeds)
    filtered_values_dict['wind_sensor_std_speed_knots'] = np.std(wind_speeds)

    # wind direction filters/stats
    wind_directions = [sensor_data_dict[field] for field in WIND_DIRECTION_FIELDS]
    average_angle = vectorAverage(np.ones(len(wind_directions)), wind_directions)[1]
    filtered_values_dict['wind_sensor_mean_threshold_angle_degrees'] = average_angle \
        if filtered_values_dict['wind_sensor_mean_speed_knots'] >= WIND_SPEED_THRESHOLD else 0.0
    filtered_values_dict['wind_sensor_std_angle_degrees'] = np.std(wind_directions)

    # wind vector average filters
    vector_speed, vector_angle = vectorAverage(wind_speeds, wind_directions)
    filtered_values_dict['wind_sensor_vector_speed_knots'] = vector_speed
    filtered_values_dict['wind_sensor_vector_threshold_angle_degrees'] = vector_angle \
        if filtered_values_dict['wind_sensor_vector_speed_knots'] >= WIND_SPEED_THRESHOLD else 0.0

    return filtered_values_dict


def vectorAverage(magnitudes, angles):
    """Finds the average vector.

    Args:
        magnitudes (list): The list of vector magnitude floats.
        angles (list): The list of vector angle floats corresponding to the magnitudes.

    Returns:
        tuple: (magnitude, angle), where magnitude and angles are floats and angle is in [0, 360).
    """
    # Convert from magnitude and direction to components representation of the vectors
    y_components = []
    x_components = []
    for i in range(len(magnitudes)):
        magnitude = magnitudes[i]
        angle = np.deg2rad(angles[i])
        y_components.append(magnitude * np.sin(angle))
        x_components.append(magnitude * np.cos(angle))

    # Compute magnitude and direction of average vector
    y_component = np.average(y_components)
    x_component = np.average(x_components)
    angle_180 = np.rad2deg(np.arctan2(y_component, x_component))  # angle_180 in [-180, 180]
    angle_360 = angle_180 if angle_180 >= 0 or np.isclose(angle_180, 0) else 360 + angle_180  # angle_360 in [0, 360)
    return np.linalg.norm([y_component, x_component]), angle_360


if __name__ == '__main__':
    os.environ["WANDB_SILENT"] = "true"  # Avoid small wandb bug
    wandb.init(entity='ubcsailbot', project='sailbot-default-project')

    # Subscribe to essential pathfinding info
    sailbot = sbot.Sailbot(nodeName='wandb_log')
    rospy.Subscriber("sensors", msg.Sensors, sensorsCallback)
    sailbot.waitForFirstSensorDataAndGlobalPath()

    # Log parameters
    config = wandb.config
    params = {name: rospy.get_param(name) for name in rospy.get_param_names()}
    config.update(params)

    # Setup subscribers
    last_scatter_plots_time = None
    last_screenshot_time = None
    screenshot_num = 1
    log_closest_obstacle = LogClosestObstacle(create_ros_node=False)
    path_storer = PathStorer(create_ros_node=False)
    collision_checker = CollisionChecker(create_ros_node=False)
    sailbot_gps_data = SailbotGPSData(create_ros_node=False)
    rate = rospy.Rate(UPDATE_TIME_SECONDS)
    rospy.Subscriber("nextGlobalWaypoint", msg.latlon, nextGlobalWaypointCallback)

    while not rospy.is_shutdown():
        # Update states
        log_closest_obstacle.find_closest_ship()
        path_storer.store_path()
        collision_checker.check_for_collisions()
        collision_checker.check_for_warnings()
        sailbot_gps_data.StoreSailbotGPS()
        boatState = sailbot.getCurrentState()

        # Store logs
        validDataReady = (len(log_closest_obstacle.closestDistances) > 0 and len(path_storer.pathCosts) > 0 and
                          len(path_storer.pathCostBreakdowns) > 0 and targetGlobalLatLon is not None)
        if validDataReady:
            # Most logs
            new_log = {'Collisions': collision_checker.get_times_collided(),
                       'NearCollisions': collision_checker.get_times_warned(),
                       'Lat': sailbot_gps_data.lat,
                       'Lon': sailbot_gps_data.lon,
                       'DisplacementKm': sailbot_gps_data.Find_Distance(),
                       'ClosestObstacleDistanceKm': log_closest_obstacle.closestDistances[-1],
                       'PathCost': path_storer.pathCosts[-1],
                       'Heading': boatState.headingDegrees,
                       'Speed': boatState.speedKmph,
                       'GlobalWindDirection': boatState.globalWindDirectionDegrees,
                       'GlobalWindSpeedKmph': boatState.globalWindSpeedKmph,
                       'GlobalWaypoint lat': targetGlobalLatLon.lat,
                       'GlabalWaypoint lon': targetGlobalLatLon.lon,
                       'NumAISShips': len(boatState.AISData.ships),
                       }

            # Log sensor data
            if LOG_SENSORS and sensor_data is not None:
                sensor_data_fields = [f for f in dir(sensor_data)
                                      if not f.startswith('_') and not callable(getattr(sensor_data, f))]
                sensor_data_dict = {f: getattr(sensor_data, f) for f in sensor_data_fields}
                new_log.update(sensor_data_dict)
                new_log.update(sensorFilters(sensor_data_dict))

            # Next, previous, and last local waypoint
            currentPath = path_storer.paths[-1]
            if len(currentPath) >= 2:
                nextWaypoint, prevWaypoint, lastWaypoint = currentPath[1], currentPath[0], currentPath[-1],
                new_log.update({"NextWaypointLat": nextWaypoint.lat,
                                "NextWaypointLon": nextWaypoint.lon,
                                "PrevWaypointLat": prevWaypoint.lat,
                                "PrevWaypointLon": prevWaypoint.lon,
                                "LastWaypointLat": lastWaypoint.lat,
                                "LastWaypointLon": lastWaypoint.lon,
                                })
            # Closest AIS ships
            NUM_CLOSEST_AIS_LOG = 5
            shipsSortedByDist = getShipsSortedByDistance(boatState.AISData.ships, boatState.position)
            if len(shipsSortedByDist) > NUM_CLOSEST_AIS_LOG:
                shipsSortedByDist = shipsSortedByDist[:NUM_CLOSEST_AIS_LOG]
            new_log.update({"Ship{}Lat".format(i): ship.lat for i, ship in enumerate(shipsSortedByDist)})
            new_log.update({"Ship{}Lon".format(i): ship.lon for i, ship in enumerate(shipsSortedByDist)})
            new_log.update({"Ship{}Speed".format(i): ship.speedKmph for i, ship in enumerate(shipsSortedByDist)})
            new_log.update({"Ship{}Heading".format(i): ship.headingDegrees for i, ship in enumerate(shipsSortedByDist)})

            # Get objective and their weighted cost.
            # Will be in form: [..., 'Weighted', 'cost', '=', '79343.0', ...]
            objectives = [x for x in path_storer.pathCostBreakdowns[0].split(" ")
                          if "Objective" in x and "Multi" not in x]
            costBreakdownStringSplit = path_storer.pathCostBreakdowns[-1].split(" ")
            costBreakdown = [float(costBreakdownStringSplit[i+3]) for i in range(len(costBreakdownStringSplit))
                             if costBreakdownStringSplit[i] == "Weighted"]
            for i, objective in enumerate(objectives):
                new_log['{}WeightedCost'.format(objective)] = costBreakdown[i]

            wandb.log(new_log)

            # Latlon scatter plot
            if LATLON_TABLE and (last_scatter_plots_time is None
                                 or time.time() - last_scatter_plots_time >= SCATTER_PLOTS_PERIOD_SECONDS):
                # Sailbot latlon
                data = [[lon, lat] for (lat, lon) in zip(sailbot_gps_data.latArray, sailbot_gps_data.lonArray)]
                table = wandb.Table(data=data, columns=['Lon', 'Lat'])
                wandb.log({'LatlonPlot': wandb.plot.scatter(table, 'Lon', 'Lat', title='Sailbot Latlon Plot')})

                last_scatter_plots_time = time.time()

            # Screenshot
            if SCREENSHOT and (last_screenshot_time is None
                               or time.time() - last_screenshot_time >= SCREENSHOT_PERIOD_SECONDS):
                screenshot_path = takeScreenshot(return_path_to_file=True)
                screenshot = Image.open(screenshot_path)

                # Change to jpg file extension for jpg compression
                compressed_screenshot_path = os.path.splitext(screenshot_path)[0] + '.jpg'
                screenshot.save(compressed_screenshot_path, "JPEG", optimize=True, quality=5)
                screenshot = Image.open(compressed_screenshot_path)

                wandb.log({"Screenshot {}".format(screenshot_num):
                           [wandb.Image(screenshot, caption="My Screenshot {}".format(screenshot_num))]})

                last_screenshot_time = time.time()
                screenshot_num += 1

        rate.sleep()
