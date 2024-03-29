#!/usr/bin/env python
import wandb
import rospy
import os
import re
from datetime import datetime
from log_closest_obstacle import LogClosestObstacle
from path_storer import PathStorer
from collision_checker import CollisionChecker
from store_sailbot_gps import SailbotGPSData
import Sailbot as sbot
import sailbot_msg.msg as sailbot_msg
import std_msgs.msg as std_msgs

# Parameters for what and how to log
UPDATE_TIME_SECONDS = 1

# Globals for subscribing
sensor_data = None
actuation_angle_data = None
gps_data = None
windSensor_data = None
lowWindConditions_data = None


def sensorsCallback(data):
    global sensor_data
    sensor_data = data


def actuationAngleCallback(data):
    global actuation_angle_data
    actuation_angle_data = data


def gpsCallback(data):
    global gps_data
    gps_data = data


def windSensorCallback(data):
    global windSensor_data
    windSensor_data = data


def lowWindConditionsCallback(data):
    global lowWindConditions_data
    lowWindConditions_data = data


def getDictFromMsg(section_name, data, regex='.'):
    data_fields = [f for f in dir(data)
                   if not f.startswith('_') and not callable(getattr(data, f)) and re.match(regex, f)]
    data_dict = {section_name + '/' + f: getattr(data, f) for f in data_fields}
    return data_dict


if __name__ == '__main__':
    os.environ["WANDB_SILENT"] = "true"  # Avoid small wandb bug

    # Subscribe to essential pathfinding info
    sailbot = sbot.Sailbot(nodeName='log_stats')
    rospy.Subscriber("sensors", sailbot_msg.Sensors, sensorsCallback)
    rospy.Subscriber("actuation_angle", sailbot_msg.actuation_angle, actuationAngleCallback)
    rospy.Subscriber("GPS", sailbot_msg.GPS, gpsCallback)
    rospy.Subscriber("windSensor", sailbot_msg.windSensor, windSensorCallback)
    rospy.Subscriber('lowWindConditions', std_msgs.Bool, lowWindConditionsCallback)
    sailbot.waitForFirstSensorDataAndGlobalPath()

    # Set up subscribers
    log_closest_obstacle = LogClosestObstacle(create_ros_node=False)
    path_storer = PathStorer(create_ros_node=False)
    collision_checker = CollisionChecker(create_ros_node=False)
    sailbot_gps_data = SailbotGPSData(create_ros_node=False)
    rate = rospy.Rate(1.0 / UPDATE_TIME_SECONDS)

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
                log.update(getDictFromMsg(section_name='GPS Sensors', data=sensor_data, regex='gps'))
                log.update(getDictFromMsg(section_name='Wind Sensors', data=sensor_data, regex='wind_sensor'))
                log.update(getDictFromMsg(section_name='Other Sensors', data=sensor_data, regex='(?!gps|wind_sensor)'))

            # Log GPS
            if gps_data is not None:
                log.update(getDictFromMsg(section_name='GPS Sensors', data=gps_data))

            # Log windSensor
            if windSensor_data is not None:
                log.update(getDictFromMsg(section_name='Wind Sensors', data=windSensor_data))

            # Log actuation angle
            if actuation_angle_data is not None:
                log.update(getDictFromMsg(section_name='Actuation Angle', data=actuation_angle_data))

            # Log low wind conditions
            if lowWindConditions_data is not None:
                log['Low Power Mode/lowWindConditions'] = lowWindConditions_data.data

            # Get objective and their weighted cost.
            # Will be in form: [..., 'Weighted', 'cost', '=', '79343.0', ...]
            objectives = [x for x in path_storer.pathCostBreakdowns[0].split(" ")
                          if "Objective" in x and "Multi" not in x]
            costBreakdownStringSplit = path_storer.pathCostBreakdowns[-1].split(" ")
            costBreakdown = [float(costBreakdownStringSplit[i + 3]) for i in range(len(costBreakdownStringSplit))
                             if costBreakdownStringSplit[i] == "Weighted"]
            for i, objective in enumerate(objectives):
                log['Cost Breakdown/{} Weighted Cost'.format(objective)] = costBreakdown[i]

            rospy.loginfo('Logging stats to Wandb')
            wandb.log(log)

        rate.sleep()
