// Copyright 2019 UBC Sailbot

#ifndef LOCAL_PATHFINDING_TIMEOBJECTIVE_H
#define LOCAL_PATHFINDING_TIMEOBJECTIVE_H

#include <ompl/base/OptimizationObjective.h>

class TimeObjective : public ompl::base::OptimizationObjective {
 public:
  TimeObjective(const ompl::base::SpaceInformationPtr &si);
  ompl::base::Cost stateCost(const ompl::base::State *s) const override;
  ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const override;
};

#endif  // LOCAL_PATHFINDING_TIMEOBJECTIVE_H
