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


# Constants for wind sensor filtering
WIND_ACTIVE_SENSORS = range(1, 4)
WIND_SPEED_FIELDS = ['wind_sensor_{}_speed_knots'.format(i) for i in WIND_ACTIVE_SENSORS]
WIND_DIRECTION_FIELDS = ['wind_sensor_{}_angle_degrees'.format(i) for i in WIND_ACTIVE_SENSORS]
WIND_SPEED_THRESHOLD = 0.5  # wind direction is set to 0 if its speed is below the threshold
PI_DEG = 180.0

# Globals for wind EWMA filtering
WIND_EWMA_WEIGHTS = [w for w in [0.01, 0.02, 0.05, 0.1, 0.2, 0.35, 0.5]]  # floats in [0, 1]
WIND_SPEED_PAST_EWMAS = [None for _ in range(len(WIND_EWMA_WEIGHTS))]
WIND_DIR_PAST_EWMAS = [None for _ in range(len(WIND_EWMA_WEIGHTS))]


def sensorsCallback(data):
    global sensor_data
    sensor_data = data


def nextGlobalWaypointCallback(newTargetGlobalLatLon):
    global targetGlobalLatLon
    targetGlobalLatLon = newTargetGlobalLatLon


def sensorFilters(sensor_data_dict):
    """Returns a dictionary with sensor data filtered in various ways.

    Statistics:
        Wind speed:
            Standard deviation
        Wind direction
            Standard deviation of circular data: see angleStats()

    Filters:
        Wind speed:
            Real time: median
            Moving average: EWMA of different weights
        Wind direction:
            Real time: median threshold: see angleStats()
            Moving average: EWMA (convert to continuous values using mitsutaVals()) of different weights threshold

    Args:
        sensor_data_dict (dict): { <sensor field name>: <sensor field value> }
    """
    filtered_values_dict = {}

    # wind speed stats and real time filters
    wind_speeds = [sensor_data_dict[field] for field in WIND_SPEED_FIELDS]
    filtered_values_dict['wind_sensor_std_speed'] = np.std(wind_speeds)
    wind_speed_median = np.median(wind_speeds)
    filtered_values_dict['wind_sensor_median_speed_knots'] = wind_speed_median

    # wind direction stats and real time filters
    wind_directions = [sensor_data_dict[field] for field in WIND_DIRECTION_FIELDS]
    wind_angle_mean, wind_angle_std = angleStats(wind_directions)
    filtered_values_dict['wind_sensor_std_angle'] = wind_angle_std
    filtered_values_dict['wind_sensor_mean_threshold_angle_degrees'] = wind_angle_mean \
        if wind_speed_median >= WIND_SPEED_THRESHOLD else 0.0

    # wind moving average filters
    for i in range(len(WIND_EWMA_WEIGHTS)):
        WIND_SPEED_PAST_EWMAS[i] = exponentiallyWeightedMovingAverage(wind_speed_median, i, is_angle=False)
        filtered_values_dict['wind_sensor_ewma{}_speed_knots'.format(i)] = WIND_SPEED_PAST_EWMAS[i]

        WIND_DIR_PAST_EWMAS[i] = exponentiallyWeightedMovingAverage(wind_angle_mean, i, is_angle=True)
        filtered_values_dict['wind_sensor_ewma{}_threshold_angle_degrees'.format(i)] = WIND_DIR_PAST_EWMAS[i] \
            if WIND_SPEED_PAST_EWMAS[i] >= WIND_SPEED_THRESHOLD else 0.0

    return filtered_values_dict


def angleStats(angles):
    """Computes the mean and standard deviation of angles using vector addition.
    Standard deviation of circular data: https://stackoverflow.com/a/13936342

    Args:
        angles (list): The list of floats in [0, 360].
    Returns:
        tuple: (mean, std), where mean and std are floats.
    """
    # Compute components of the average vector
    y_components = []
    x_components = []
    for i in range(len(angles)):
        angle = np.deg2rad(angles[i])
        y_components.append(np.sin(angle))
        x_components.append(np.cos(angle))
    y_component = np.average(y_components)
    x_component = np.average(x_components)

    mean_180 = np.rad2deg(np.arctan2(y_component, x_component))  # mean_180 in [-180, 180]
    mean_360 = mean_180 if mean_180 >= 0 or np.isclose(mean_180, 0) else 360 + mean_180  # mean_360 in [0, 360)
    std = np.sqrt(-np.log(y_component**2 + x_component**2))
    return mean_360, std


def exponentiallyWeightedMovingAverage(current_val, ewma_ind, is_angle):
    """Calculate the EWMA from the current value and past EWMA:
    https://hackaday.com/2019/09/06/sensor-filters-for-coders/.

    Args:
        current_val (float): Current value.
        ewma_ind (int): Index of the past EWMA and weight of current_val in
            WIND_SPEED_PAST_EWMAS and WIND_EWMA_WEIGHTS, respectively.
        is_angle (bool): True when getting the field is an angle, false otherwise.

    Returns:
        float: The exponentially weighted moving average.
    """
    past_ewma = WIND_DIR_PAST_EWMAS[ewma_ind] if is_angle else WIND_SPEED_PAST_EWMAS[ewma_ind]
    if past_ewma is None:
        return current_val

    weight = WIND_EWMA_WEIGHTS[ewma_ind]
    if is_angle:
        _, current_val = mitsutaVals([past_ewma, current_val])
    return weight * current_val + (1 - weight) * past_ewma


def mitsutaVals(angles):
    """Make angle readings continuous following
    https://journals.ametsoc.org/view/journals/apme/25/10/1520-0450_1986_025_1387_eospeo_2_0_co_2.xml
    following section 2.d. Assumes the time scale is small enough so that the direction to the next reading is < 180.
    Once it is continuous, the mean and standard deviation can be computed using conventional methods.

    Args:
        angles (list): List of floats in [0, 360].

    Returns:
        list: angles such that its values are sequentially continuous.
            Example of readings crossing the discontinuity point:
                mitsutaVals([359.0, 1.0]) = [359, 361.0]
                mitsutaVals([1.0, 359.0]) = [1.0, -1.0]
    """
    vals = [angles[0]]
    for i in range(1, len(angles)):
        delta = angles[i] - angles[i-1]
        if delta < -PI_DEG:
            delta += 2 * PI_DEG
        elif delta > PI_DEG:
            delta -= 2 * PI_DEG
        vals.append(vals[i-1] + delta)
    return vals


if __name__ == '__main__':
    os.environ["WANDB_SILENT"] = "true"  # Avoid small wandb bug
    wandb.init(entity='sailbot', project='dry-land-testing')

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
