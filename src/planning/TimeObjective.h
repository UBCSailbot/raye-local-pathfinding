// Copyright 2019 UBC Sailbot

#ifndef PLANNING_TIMEOBJECTIVE_H_
#define PLANNING_TIMEOBJECTIVE_H_

#include <ompl/base/OptimizationObjective.h>

class TimeObjective : public ompl::base::OptimizationObjective {
 public:
  explicit TimeObjective(const ompl::base::SpaceInformationPtr &si);
  ompl::base::Cost stateCost(const ompl::base::State *s) const override;
  ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const override;
};

#endif  // PLANNING_TIMEOBJECTIVE_H_
