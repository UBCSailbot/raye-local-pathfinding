// Copyright 2019 UBC Sailbot

#include "TimeObjective.h"
#include <ompl/base/spaces/TimeStateSpace.h>

TimeObjective::TimeObjective(const ompl::base::SpaceInformationPtr &si) : OptimizationObjective(si) {}


ompl::base::Cost TimeObjective::motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const {
  const auto start_compound_state = s1->as<ompl::base::CompoundStateSpace::StateType>();

  auto start_time = start_compound_state->as<ompl::base::TimeStateSpace::StateType>(2)->position;

  const auto end_compound_state = s2->as<ompl::base::CompoundStateSpace::StateType>();
  auto end_time = end_compound_state->as<ompl::base::TimeStateSpace::StateType>(2)->position;

  return ompl::base::Cost(end_time-start_time);
}
ompl::base::Cost TimeObjective::stateCost(const ompl::base::State *s) const {
  const auto start_compound_state = s->as<ompl::base::CompoundStateSpace::StateType>();
  auto time = start_compound_state->as<ompl::base::TimeStateSpace::StateType>(2)->position;
  return ompl::base::Cost(time);
}
