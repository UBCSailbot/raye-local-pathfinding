// Copyright 2019 UBC Sailbot

#ifndef PLANNING_PLANNER_H_
#define PLANNING_PLANNER_H_

#include <ompl/base/State.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/control/SimpleSetup.h>
#include "SailboatStatePropagator.h"

class Planner {
 public:
  Planner(float windDirection);
  ompl::base::PlannerStatus Solve();
  void printSetup();
  ompl::control::PathControl & getPath();

 private:
  bool isStateValid(const ompl::base::State *state);

  ompl::control::SimpleSetupPtr ss_;
  float wind_direction_;
};

#endif  // PLANNING_PLANNER_H_
