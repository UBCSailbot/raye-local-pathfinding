#!/usr/bin/env python
import wandb
import rospy
import os
from datetime import datetime
from path_storer import PathStorer
from store_sailbot_gps import SailbotGPSData
from utilities import getShipsSortedByDistance
import Sailbot as sbot
import sailbot_msg.msg as msg

# Parameters for what and how to log
UPDATE_TIME_SECONDS = 30.0
NUM_CLOSEST_AIS_LOG = 10

# Globals for subscribing
targetGlobalLatLon = None


def nextGlobalWaypointCallback(newTargetGlobalLatLon):
    global targetGlobalLatLon
    targetGlobalLatLon = newTargetGlobalLatLon


if __name__ == '__main__':
    os.environ["WANDB_SILENT"] = "true"  # Avoid small wandb bug

    # Subscribe to essential pathfinding info
    sailbot = sbot.Sailbot(nodeName='log_latlon_plots')
    sailbot.waitForFirstSensorDataAndGlobalPath()

    # Setup subscribers
    path_storer = PathStorer(create_ros_node=False)
    sailbot_gps_data = SailbotGPSData(create_ros_node=False)
    rate = rospy.Rate(UPDATE_TIME_SECONDS)
    rospy.Subscriber("nextGlobalWaypoint", msg.latlon, nextGlobalWaypointCallback)

    # Set wandb project name and run directory path
    start_datetime = datetime.now().strftime('%b_%d-%H_%M_%S')
    project_name = 'latlon_plots-' + start_datetime
    dir_path = os.path.expanduser('~/.ros/latlon_plots/' + start_datetime)
    os.makedirs(dir_path)

    while not rospy.is_shutdown():

        # Update states
        path_storer.store_path()
        sailbot_gps_data.StoreSailbotGPS()
        boatState = sailbot.getCurrentState()

        # Store logs
        validDataReady = (len(path_storer.pathCosts) > 0 and
                          len(path_storer.pathCostBreakdowns) > 0 and targetGlobalLatLon is not None)
        if validDataReady:
            run = wandb.init(entity='sailbot', dir=dir_path, project=project_name)

            # Log parameters
            params = {
                'updatePeriodSeconds': UPDATE_TIME_SECONDS,
                'maxShipsPlotted': NUM_CLOSEST_AIS_LOG
            }
            wandb.config.update(params)

            current_latlon = [sailbot_gps_data.lon, sailbot_gps_data.lat]
            log = {}

            # GPS coordinates scatter plot
            table = wandb.Table(data=[current_latlon], columns=['Lon', 'Lat'])
            log.update({'gps_plot': wandb.plot.scatter(table, 'Lon', 'Lat', title='Raye GPS Coordinates')})

            # Local waypoints scatter plot
            currentPath = path_storer.paths[-1]
            data = [[waypoint.lon, waypoint.lat] for waypoint in currentPath]
            data.append(current_latlon)  # add Raye position
            table = wandb.Table(data=data, columns=['Lon', 'Lat'])
            log.update({'waypoint_plot': wandb.plot.scatter(table, 'Lon', 'Lat', title='Local Waypoints')})

            # Closest AIS ships
            shipsSortedByDist = getShipsSortedByDistance(boatState.AISData.ships,
                                                         boatState.position)[:NUM_CLOSEST_AIS_LOG]
            data = [[ship.lon, ship.lat] for ship in shipsSortedByDist]
            data.append(current_latlon)  # add Raye position
            table = wandb.Table(data=data, columns=['Lon', 'Lat'])
            log.update({'ship_plot': wandb.plot.scatter(table, 'Lon', 'Lat', title='Closet Ships')})
            # would it be beneficial to log speed (ship.speedKmph) and heading (ship.headingDegrees)?

            wandb.log(log)
            run.finish()

        rate.sleep()
