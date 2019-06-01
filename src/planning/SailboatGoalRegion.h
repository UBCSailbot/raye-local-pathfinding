// Copyright 2019 UBC Sailbot

#ifndef PLANNING_SAILBOATGOALREGION_H_
#define PLANNING_SAILBOATGOALREGION_H_

#include <ompl/base/goals/GoalRegion.h>
#include "Coordinate.h"

class SailboatGoalRegion : public ompl::base::GoalRegion {
 public:
  SailboatGoalRegion(const ompl::base::SpaceInformationPtr &si,
                     double threshold,
                     const Coordinate &goal);
  virtual double distanceGoal(const ompl::base::State *state) const;
 private:
  Coordinate goal_;
};

#endif  // PLANNING_SAILBOATGOALREGION_H_
