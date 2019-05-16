// Copyright 2019 UBC Sailbot

#ifndef PLANNING_PLANNER_H_
#define PLANNING_PLANNER_H_

#include <ompl/base/State.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/control/SimpleSetup.h>
#include "SailboatStatePropagator.h"
#include "Obstacle.h"

class Planner {
 public:
  Planner(float windDirection, std::vector<Obstacle> obstacles);
  ompl::base::PlannerStatus Solve();
  void printSetup();
  ompl::control::PathControl & getPath();

 private:
  ompl::control::SimpleSetupPtr ss_;
  float wind_direction_;
  std::vector<Obstacle> obstacles_;
};

#endif  // PLANNING_PLANNER_H_
