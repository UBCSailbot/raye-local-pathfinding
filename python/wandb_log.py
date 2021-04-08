#!/usr/bin/env python
import wandb
import cv2
import rospy
from log_closest_obstacle import LogClosestObstacle
from path_storer import PathStorer
from collision_checker import CollisionChecker
from store_sailbot_gps import SailbotGPSData
from utilities import takeScreenshot
# import matplotlib.pyplot as plt
from PIL import Image

UPDATE_TIME_SECONDS = 1.0
SCREENSHOT_PERIOD_SECONDS = 2.0


if __name__ == '__main__':
    import os
    os.environ["WANDB_SILENT"] = "true"
    rospy.init_node('wandb_log')
    wandb.init(entity='ubcsailbot', project='sailbot-test-2')

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
                       'PathCost': path_storer.pathCosts[-1]}

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
            data = [[lon, lat] for (lat, lon) in zip(sailbot_gps_data.latArray, sailbot_gps_data.lonArray)]
            table = wandb.Table(data=data, columns=['Lon', 'Lat'])
            wandb.log({'LatlonPlot': wandb.plot.scatter(table, 'Lon', 'Lat', title='Sailbot Latlon Plot')})

            # Screenshot
            if counter % (SCREENSHOT_PERIOD_SECONDS // UPDATE_TIME_SECONDS) == 0:
                screenshot_path = takeScreenshot(return_path_to_file=True)
                # screenshot = plt.imread(screenshot_path)
                # screenshot = cv2.imread(screenshot_path)
                # print(screenshot.shape)
                # screenshot = cv2.resize(screenshot, dsize=(screenshot.shape[1]//4, screenshot.shape[0]//4))
                # print(screenshot.shape)
                screenshot = Image.open(screenshot_path)
                print(screenshot_path)
                print(screenshot.size)
                print("WORKING11111111111111111111111")
                output_path
                screenshot.save(screenshot_path, "JPEG", optimize=True, quality=50)
                screenshot = Image.open(screenshot_path)
                print(screenshot.size)
                print("2WORKING11111111111111111111111")

                wandb.log({"Screenshot {}".format(counter):
                           [wandb.Image(screenshot, caption="My Screenshot {}".format(counter))]})
            counter += 1

        rate.sleep()
