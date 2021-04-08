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
        if len(log_closest_obstacle.closestDistances) > 0:
            wandb.log({'closestObstacleDistanceKm': log_closest_obstacle.closestDistances[-1]})
        if len(path_storer.pathCosts) > 0:
            wandb.log({'pathCost': path_storer.pathCosts[-1]})
        if len(path_storer.pathCostBreakdowns) > 0:
            objectives = [x for x in path_storer.pathCostBreakdowns[0].split(" ") if "Objective" in x and "Multi" not in x]

            # Get weighted cost. Will be in form: [..., 'Weighted', 'cost', '=', '79343.0', ...]
            costBreakdownStringSplit = path_storer.pathCostBreakdowns[-1].split(" ")
            costBreakdown = [float(costBreakdownStringSplit[i+3]) for i in range(len(costBreakdownStringSplit))
                             if costBreakdownStringSplit[i] == "Weighted"]

            for i, objective in enumerate(objectives):
                wandb.log({'{}WeightedCost'.format(objective): costBreakdown[i]})
        wandb.log({'Collisions': collision_checker.get_times_collided()})
        wandb.log({'NearCollisions': collision_checker.get_times_warned()})
        wandb.log({'Lat': sailbot_gps_data.lat})
        wandb.log({'Lon': sailbot_gps_data.lon})
        wandb.log({'DisplacementKm': sailbot_gps_data.Find_Distance()})

        rate.sleep()
