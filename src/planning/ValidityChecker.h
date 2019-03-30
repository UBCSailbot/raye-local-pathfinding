// Copyright 2019 UBC Sailbot

#ifndef PLANNING_VALIDITYCHECKER_H_
#define PLANNING_VALIDITYCHECKER_H_

#include <ompl/base/StateValidityChecker.h>
#include <vector>
#include "Obstacle.h"

class ValidityChecker : ompl::base::StateValidityChecker {
 public:
  ValidityChecker(const ompl::base::SpaceInformationPtr &si,
                    std::vector<Obstacle> obstacles);
 private:
  bool isValid(const ompl::base::State *state) const override;
  std::vector<Obstacle> obstacles_;
};

#endif  // PLANNING_VALIDITYCHECKER_H_
