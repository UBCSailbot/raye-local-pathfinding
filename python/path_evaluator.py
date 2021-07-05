#!/usr/bin/env python
import wandb
import os
import numpy as np
import Path
import Sailbot as sbot
import rospy

NUMBER_TESTS = 5
WANDB = True

if __name__ == '__main__':
    # Setup wandb
    if WANDB:
        os.environ["WANDB_SILENT"] = "true"  # Avoid small wandb bug
        wandb.init(entity='ubcsailbot', project='path-evaluation-1')
        config = wandb.config
        params = {name: rospy.get_param(name) for name in rospy.get_param_names()}
        config.update(params)

    # Setup subscribers
    sailbot = sbot.Sailbot(nodeName='path_evaluator')
    sailbot.waitForFirstSensorDataAndGlobalPath()
    state = sailbot.getCurrentState()

    # Run pathfinding multiple times
    paths = [Path.createPath(state) for _ in range(NUMBER_TESTS)]

    # Extract key information
    logs = []
    for path in paths:
        # Create log
        log = {'PathCost': path.getCost(),
               'PathLength': path.getLength()}

        objectives = [x for x in path.getPathCostBreakdownString().split(" ")
                      if "Objective" in x and "Multi" not in x]
        costBreakdownStringSplit = path.getPathCostBreakdownString().split(" ")
        costBreakdown = [float(costBreakdownStringSplit[i+3]) for i in range(len(costBreakdownStringSplit))
                         if costBreakdownStringSplit[i] == "Weighted"]
        for i, objective in enumerate(objectives):
            log['{}WeightedCost'.format(objective)] = costBreakdown[i]

        # Output logs
        logs.append(log)

    # Log key information
    for test_num, log in enumerate(logs):
        rospy.loginfo("Path evaluator run {}: {}".format(test_num, log))
        if WANDB:
            wandb.log(log)

    # Log aggregated metrics
    averagePathCost = np.mean([log['PathCost'] for log in logs])
    stdPathCost = np.std([log['PathCost'] for log in logs])
    rospy.loginfo("AveragePathCost = {}".format(averagePathCost))
    rospy.loginfo("StdPathCost = {}".format(stdPathCost))
    if WANDB:
        wandb.log({'AveragePathCost': averagePathCost})
        wandb.log({'StdPathCost': stdPathCost})

    rospy.signal_shutdown("Path evaluator complete")
