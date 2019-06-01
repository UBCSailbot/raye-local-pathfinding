// Copyright 2019 UBC Sailbot

#include "SailboatGoalRegion.h"
#include <ompl/base/spaces/SE2StateSpace.h>

SailboatGoalRegion::SailboatGoalRegion(const ompl::base::SpaceInformationPtr &si,
                                       double threshold,
                                       const Coordinate &goal)
    : ompl::base::GoalRegion(si), goal_(goal) {
  setThreshold(threshold);
}

double SailboatGoalRegion::distanceGoal(const ompl::base::State *state) const {
  const auto compound_state = state->as<ompl::base::CompoundStateSpace::StateType>();
  const auto se2_state = compound_state->as<ompl::base::SE2StateSpace::StateType>(0);
  double x = se2_state->getX();
  double y = se2_state->getY();

  return sqrt(pow(goal_.getX() - x, 2) +
      pow(goal_.getY() - y, 2));
}
