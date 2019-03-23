// Copyright 2019 UBC Sailbot

#ifndef EXAMPLE_PLANNER_H_
#define EXAMPLE_PLANNER_H_

#include <ompl/base/State.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/PlannerStatus.h>

class Planner {
 public:
  Planner();
  ompl::base::PlannerStatus Solve();
  void printSetup();
  ompl::geometric::PathGeometric & getPath();

 private:
  bool isStateValid(const ompl::base::State *state);

  ompl::geometric::SimpleSetupPtr ss;

};

#endif  // EXAMPLE_PLANNER_H_
