// Copyright 2019 UBC Sailbot

#ifndef LOCAL_PATHFINDING_SAILBOATGOALREGION_H
#define LOCAL_PATHFINDING_SAILBOATGOALREGION_H

#include <ompl/base/goals/GoalRegion.h>


class SailboatGoalRegion : public ompl::base::GoalRegion {
 public:
  SailboatGoalRegion(const ompl::base::SpaceInformationPtr &si) : ompl::base::GoalRegion(si)
  {
    setThreshold(0.1);
  }
  virtual double distanceGoal(const ompl::base:State *st) const;
};

#endif  // LOCAL_PATHFINDING_SAILBOATGOALREGION_H
