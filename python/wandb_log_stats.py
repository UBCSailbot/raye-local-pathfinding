#!/usr/bin/env python
import wandb
import rospy
import os
from datetime import datetime
from log_closest_obstacle import LogClosestObstacle
from path_storer import PathStorer
from collision_checker import CollisionChecker
from store_sailbot_gps import SailbotGPSData
import Sailbot as sbot
import sailbot_msg.msg as msg

# Parameters for what and how to log
UPDATE_TIME_SECONDS = 1.0

# Globals for subscribing
sensor_data = None
actuation_angle_data = None
min_voltage_data = None


def sensorsCallback(data):
    global sensor_data
    sensor_data = data


def actuationAngleCallback(data):
    global actuation_angle_data
    actuation_angle_data = data


def minVoltageCallback(data):
    global min_voltage_data
    min_voltage_data = data


def getDictFromMsg(section_name, data):
    data_fields = [f for f in dir(data) if not f.startswith('_') and not callable(getattr(data, f))]
    data_dict = {section_name + '/' + f: getattr(data, f) for f in data_fields}
    return data_dict


if __name__ == '__main__':
    os.environ["WANDB_SILENT"] = "true"  # Avoid small wandb bug

    # Subscribe to essential pathfinding info
    sailbot = sbot.Sailbot(nodeName='log_stats')
    rospy.Subscriber("sensors", msg.Sensors, sensorsCallback)
    rospy.Subscriber("actuation_angle", msg.actuation_angle, actuationAngleCallback)
    rospy.Subscriber("min_voltage", msg.min_voltage, minVoltageCallback)
    sailbot.waitForFirstSensorDataAndGlobalPath()

    # Set up subscribers
    log_closest_obstacle = LogClosestObstacle(create_ros_node=False)
    path_storer = PathStorer(create_ros_node=False)
    collision_checker = CollisionChecker(create_ros_node=False)
    sailbot_gps_data = SailbotGPSData(create_ros_node=False)
    rate = rospy.Rate(UPDATE_TIME_SECONDS)

    # Set wandb run name and directory path
    start_datetime = datetime.now().strftime('%b_%d-%H_%M_%S')
    run_name = 'stats-' + start_datetime
    dir_dir = os.path.expanduser('~/.ros/stats/')
    dir_path = dir_dir + start_datetime
    os.makedirs(dir_dir + start_datetime)
    os.symlink(dir_path, dir_dir + 'tmp')
    os.rename(dir_dir + 'tmp', dir_dir + 'latest-dir')  # use replace() in Python 3

    wandb.init(entity='sailbot', project='stats', dir=dir_path, name=run_name)

    # Log parameters
    params = {name: rospy.get_param(name) for name in rospy.get_param_names()}
    params['updatePeriodSeconds'] = UPDATE_TIME_SECONDS
    wandb.config.update(params)

    while not rospy.is_shutdown():
        # Update states
        log_closest_obstacle.find_closest_ship()
        path_storer.store_path()
        collision_checker.check_for_collisions()
        collision_checker.check_for_warnings()
        sailbot_gps_data.StoreSailbotGPS()
        boatState = sailbot.getCurrentState()

        # Store logs
        validDataReady = len(path_storer.pathCosts) > 0 and len(path_storer.pathCostBreakdowns) > 0
        if validDataReady:
            log = {}

            # Most logs
            general = {
                'Number of Collisions': collision_checker.get_times_collided(),
                'Number of Near Collisions': collision_checker.get_times_warned(),
                'Displacement (km)': sailbot_gps_data.Find_Distance(),
                'Closest Obstacle Distance (km)': log_closest_obstacle.closestDistances[-1],
                'Path Cost': path_storer.pathCosts[-1],
                'Heading': boatState.headingDegrees,
                'Speed': boatState.speedKmph,
                'Global Wind Direction': boatState.globalWindDirectionDegrees,
                'Global Wind Speed Kmph': boatState.globalWindSpeedKmph,
                'Number of AIS Ships': len(boatState.AISData.ships),
            }
            general = {'General Statistics/' + k: v for k, v in general.items()}
            log.update(general)

            # Log sensor data
            if sensor_data is not None:
                log.update(getDictFromMsg(section_name='Sensor Data', data=sensor_data))

            # Log actuation angle
            if actuation_angle_data is not None:
                log.update(getDictFromMsg(section_name='Actuation Angle', data=actuation_angle_data))

            # Log minimum voltage
            if min_voltage_data is not None:
                log.update(getDictFromMsg(section_name='Minimum Voltage', data=min_voltage_data))

            # Get objective and their weighted cost.
            # Will be in form: [..., 'Weighted', 'cost', '=', '79343.0', ...]
            objectives = [x for x in path_storer.pathCostBreakdowns[0].split(" ")
                          if "Objective" in x and "Multi" not in x]
            costBreakdownStringSplit = path_storer.pathCostBreakdowns[-1].split(" ")
            costBreakdown = [float(costBreakdownStringSplit[i + 3]) for i in range(len(costBreakdownStringSplit))
                             if costBreakdownStringSplit[i] == "Weighted"]
            for i, objective in enumerate(objectives):
                log['Cost Breakdown/{} Weighted Cost'.format(objective)] = costBreakdown[i]

            wandb.log(log)

        rate.sleep()
