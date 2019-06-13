// Copyright 2019 UBC Sailbot

#include "Planner.h"
#include "SailboatStatePropagator.h"
#include "ValidityChecker.h"
#include "SailboatGoalRegion.h"

#include <utility>

#include <ompl/control/StatePropagator.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/spaces/TimeStateSpace.h>

Planner::Planner(double windDirection,
                 const std::vector<Obstacle> &obstacles,
                 double lowerBound,
                 double upperBound,
                 const ompl::base::ScopedState<ompl::base::SE2StateSpace> &start,
                 const BoatPosition &goal)
    : wind_direction_(windDirection), obstacles_(obstacles) {
  // construct the state space we are planning in
  ompl::base::StateSpacePtr space = getStateSpace(lowerBound, upperBound);

  oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 1));
  cspace->setName("Rudder");

  ob::RealVectorBounds cbounds(1);
  cbounds.setLow(-M_PI / 3);
  cbounds.setHigh(M_PI / 3);
  cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

  // define a simple setup class
  ss_ = std::make_shared<oc::SimpleSetup>(cspace);

  oc::SpaceInformationPtr si(ss_->getSpaceInformation());

  ob::GoalPtr goal_ptr(new SailboatGoalRegion(si, 0.1, goal));

  ss_->setGoal(goal_ptr);
  ss_->addStartState(start);

  ob::StateValidityCheckerPtr vc(new ValidityChecker(si, obstacles_));

  // set state validity checking for this space
  ss_->setStateValidityChecker(vc);

  oc::StatePropagatorPtr statePropagator(new SailboatStatePropagator(si, wind_direction_));

  ss_->setStatePropagator(statePropagator);

  // this call is optional, but we put it in to get more output information
  ss_->setup();
}

ompl::base::StateSpacePtr Planner::getStateSpace(double lowerBound, double upperBound) {
  ob::StateSpacePtr se2_space(new ob::SE2StateSpace());
  se2_space->setName("Position");
  ob::RealVectorBounds se2_bounds(2);
  se2_bounds.setHigh(lowerBound);
  se2_bounds.setHigh(upperBound);
  se2_space->as<ob::SE2StateSpace>()->setBounds(se2_bounds);

  ob::StateSpacePtr velocity(new ob::RealVectorStateSpace(1));

  velocity->setName("Velocity");
  ob::RealVectorBounds velocity_bounds(1);
  velocity_bounds.setLow(0);
  velocity_bounds.setHigh(50);
  velocity->as<ob::RealVectorStateSpace>()->setBounds(velocity_bounds);


  ob::StateSpacePtr time(new ob::TimeStateSpace());

  ob::StateSpacePtr space = se2_space + velocity + time;
  ob::ProjectionEvaluatorPtr proj(new ob::SubspaceProjectionEvaluator(space.get(), 0));
  space->registerDefaultProjection(proj);

  return space;
}

ob::PlannerStatus Planner::Solve(double time) {
  ob::PlannerStatus solved = ss_->solve(time);
  return solved;
}

void Planner::printSetup() {
  ss_->print();
}

ompl::control::PathControl &Planner::getPath() {
  return ss_->getSolutionPath();
}
