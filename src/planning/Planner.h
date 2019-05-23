// Copyright 2019 UBC Sailbot

#ifndef PLANNING_PLANNER_H_
#define PLANNING_PLANNER_H_

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/ScopedState.h>
#include "SailboatStatePropagator.h"
#include "Obstacle.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;


class Planner {
 public:
  Planner(double windDirection,
          const std::vector<Obstacle> &obstacles,
          double lowerBound,
          double upperBound,
          const ompl::base::ScopedState<ompl::base::SE2StateSpace> &start,
          const ompl::base::ScopedState<ompl::base::SE2StateSpace> &goal);
  ob::PlannerStatus Solve(double time);
  void printSetup();
  oc::PathControl & getPath();

 private:
  oc::SimpleSetupPtr ss_;
  float wind_direction_;
  std::vector<Obstacle> obstacles_;
};

#endif  // PLANNING_PLANNER_H_
