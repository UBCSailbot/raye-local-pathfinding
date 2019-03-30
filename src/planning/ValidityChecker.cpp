// Copyright 2019 UBC Sailbot

#include <ompl/base/spaces/SE2StateSpace.h>
#include "ValidityChecker.h"

ValidityChecker::ValidityChecker(const ompl::base::SpaceInformationPtr &si,
                                 std::vector<Obstacle> obstacles) : StateValidityChecker(si), obstacles_(obstacles) {}

bool ValidityChecker::isValid(const ompl::base::State *state) const {
  const auto *SE2State = state->as<ompl::base::SE2StateSpace::StateType>();
  auto x = SE2State->getX();
  auto y = SE2State->getY();
  for (auto obstacle : this->obstacles_) {
    if (std::sqrt(pow(x - obstacle.getX(), 2) + pow(y - obstacle.getY(), 2)) - obstacle.getRadius() <= 0) {
      return false;
    }
  }
  return true;
}