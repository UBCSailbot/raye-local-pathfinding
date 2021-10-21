#!/usr/bin/env python
import wandb
import rospy
import os
from log_closest_obstacle import LogClosestObstacle
from path_storer import PathStorer
from collision_checker import CollisionChecker
from store_sailbot_gps import SailbotGPSData
from utilities import takeScreenshot, getShipsSortedByDistance
from PIL import Image
import Sailbot as sbot
from sailbot_msg.msg import Sensors

# Parameters for what and how to log
UPDATE_TIME_SECONDS = 1.0
SCREENSHOT_PERIOD_SECONDS = 10.0
LATLON_TABLE = False
SCREENSHOT = False
LOG_SENSORS = True


# Globals for subscribring
sensor_data = None


def sensorsCallback(data):
    global sensor_data
    sensor_data = data


if __name__ == '__main__':
    os.environ["WANDB_SILENT"] = "true"  # Avoid small wandb bug
    wandb.init(entity='ubcsailbot', project='sailbot-default-project')

    # Subscribe to essential pathfinding info
    sailbot = sbot.Sailbot(nodeName='wandb_log')
    rospy.Subscriber("sensors", Sensors, sensorsCallback)
    sailbot.waitForFirstSensorDataAndGlobalPath()

    # Log parameters
    config = wandb.config
    params = {name: rospy.get_param(name) for name in rospy.get_param_names()}
    config.update(params)

    # Setup subscribers
    counter = 0
    log_closest_obstacle = LogClosestObstacle(create_ros_node=False)
    path_storer = PathStorer(create_ros_node=False)
    collision_checker = CollisionChecker(create_ros_node=False)
    sailbot_gps_data = SailbotGPSData(create_ros_node=False)
    rate = rospy.Rate(UPDATE_TIME_SECONDS)

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
                          len(path_storer.pathCostBreakdowns) > 0)
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
                       'NumAISShips': len(boatState.AISData.ships),
                       }

            # Log sensor data
            if LOG_SENSORS and sensor_data is not None:
                sensor_data_fields = [f for f in dir(sensor_data)
                                      if not f.startswith('_') and not callable(getattr(sensor_data, f))]
                sensor_data_dict = {f: getattr(sensor_data, f) for f in sensor_data_fields}
                new_log.update(sensor_data_dict)

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
            if LATLON_TABLE:
                data = [[lon, lat] for (lat, lon) in zip(sailbot_gps_data.latArray, sailbot_gps_data.lonArray)]
                table = wandb.Table(data=data, columns=['Lon', 'Lat'])
                wandb.log({'LatlonPlot': wandb.plot.scatter(table, 'Lon', 'Lat', title='Sailbot Latlon Plot')})

            # Screenshot
            if SCREENSHOT and counter % (SCREENSHOT_PERIOD_SECONDS // UPDATE_TIME_SECONDS) == 0:
                screenshot_path = takeScreenshot(return_path_to_file=True)
                screenshot = Image.open(screenshot_path)

                # Change to jpg file extension for jpg compression
                compressed_screenshot_path = os.path.splitext(screenshot_path)[0] + '.jpg'
                screenshot.save(compressed_screenshot_path, "JPEG", optimize=True, quality=5)
                screenshot = Image.open(compressed_screenshot_path)

                wandb.log({"Screenshot {}".format(counter):
                           [wandb.Image(screenshot, caption="My Screenshot {}".format(counter))]})
            counter += 1

        rate.sleep()
