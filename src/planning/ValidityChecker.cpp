// Copyright 2019 UBC Sailbot

#include "ValidityChecker.h"
#include <ompl/base/spaces/SE2StateSpace.h>

ValidityChecker::ValidityChecker(const ompl::base::SpaceInformationPtr &si,
                                 const std::vector<Obstacle> &obstacles)
    : StateValidityChecker(si), obstacles_(obstacles) {}

bool ValidityChecker::isValid(const ompl::base::State *state) const {
  const auto compound_state = state->as<ompl::base::CompoundStateSpace::StateType>();
  const auto se2_state = compound_state->as<ompl::base::SE2StateSpace::StateType>(0);
  double x = se2_state->getX();
  double y = se2_state->getY();

  for (auto obstacle : this->obstacles_) {
    if (std::sqrt(pow(x - obstacle.getX(), 2) + pow(y - obstacle.getY(), 2)) - obstacle.getRadius() <= 0) {
      return false;
    }
  }
  return true;
}
