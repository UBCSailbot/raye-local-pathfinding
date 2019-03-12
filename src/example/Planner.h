// Copyright 2019 UBC Sailbot

#ifndef EXAMPLE_PLANNER_H_
#define EXAMPLE_PLANNER_H_

#include <ompl/base/State.h>

class Planner {
 public:
  void planWithSimpleSetup();
  bool isStateValid(const ompl::base::State *state);
};

#endif  // EXAMPLE_PLANNER_H_
