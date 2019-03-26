// Copyright 2019 UBC Sailbot

#ifndef PLANNING_PLANNER_H_
#define PLANNING_PLANNER_H_

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

  std::shared_ptr<ompl::geometric::SimpleSetup> ss;
};

#endif  // PLANNING_PLANNER_H_
