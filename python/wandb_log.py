#!/usr/bin/env python
import wandb
import rospy
from log_closest_obstacle import LogClosestObstacle
from path_storer import PathStorer
from collision_checker import CollisionChecker
from store_sailbot_gps import SailbotGPSData

UPDATE_TIME_SECONDS = 1.0


if __name__ == '__main__':
    rospy.init_node('wandb_log')
    wandb.init(entity='tylerlum', project='sailbot-test-1')

    # Log parameters
    config = wandb.config
    params = {name: rospy.get_param(name) for name in rospy.get_param_names()}
    config.update(params)

    # Setup subscribers
    log_closest_obstacle = LogClosestObstacle()
    path_storer = PathStorer()
    collision_checker = CollisionChecker()
    sailbot_gps_data = SailbotGPSData()
    rate = rospy.Rate(UPDATE_TIME_SECONDS)

    while not rospy.is_shutdown():
        # Update states
        log_closest_obstacle.find_closest_ship()
        path_storer.store_path()
        collision_checker.check_for_collisions()
        collision_checker.check_for_warnings()
        sailbot_gps_data.StoreSailbotGPS()

        # Store logs
        if len(log_closest_obstacle.closestDistances) > 0 and len(path_storer.pathCosts) > 0 and len(path_storer.pathCostBreakdowns) > 0:
            new_log = {'Collisions': collision_checker.get_times_collided(),
                       'NearCollisions': collision_checker.get_times_warned(),
                       'Lat': sailbot_gps_data.lat,
                       'Lon': sailbot_gps_data.lon,
                       'DisplacementKm': sailbot_gps_data.Find_Distance(),
                       'closestObstacleDistanceKm': log_closest_obstacle.closestDistances[-1],
                       'pathCost': path_storer.pathCosts[-1],
                      }

            objectives = [x for x in path_storer.pathCostBreakdowns[0].split(" ") if "Objective" in x and "Multi" not in x]

            # Get weighted cost. Will be in form: [..., 'Weighted', 'cost', '=', '79343.0', ...]
            costBreakdownStringSplit = path_storer.pathCostBreakdowns[-1].split(" ")
            costBreakdown = [float(costBreakdownStringSplit[i+3]) for i in range(len(costBreakdownStringSplit))
                             if costBreakdownStringSplit[i] == "Weighted"]

            for i, objective in enumerate(objectives):
                new_log['{}WeightedCost'.format(objective)] = costBreakdown[i]
            wandb.log(new_log)

            # Latlon scatter plot
            data = [[lon, lat] for (lat, lon) in zip(sailbot_gps_data.latArray, sailbot_gps_data.lonArray)]
            table = wandb.Table(data=data, columns=['lon', 'lat'])
            wandb.log({'latlonPlot': wandb.plot.scatter(table, 'lon', 'lat', title='Sailbot Latlon Plot')})

        rate.sleep()
