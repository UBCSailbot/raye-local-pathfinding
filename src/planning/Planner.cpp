#include <utility>

// Copyright 2019 UBC Sailbot

#include "Planner.h"
#include "SailboatStatePropagator.h"
#include "ValidityChecker.h"

#include <ompl/control/StatePropagator.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>

Planner::Planner(double windDirection,
                 const std::vector<Obstacle> &obstacles,
                 double lowerBound,
                 double upperBound,
                 const ompl::base::ScopedState<ompl::base::SE2StateSpace> &start,
                 const ompl::base::ScopedState<ompl::base::SE2StateSpace> &goal)
    : wind_direction_(windDirection), obstacles_(obstacles){
  // construct the state space we are planning in
  ob::StateSpacePtr se2_space(new ob::SE2StateSpace());

  ob::RealVectorBounds se2_bounds(2);
  se2_bounds.setHigh(lowerBound);
  se2_bounds.setHigh(upperBound);
  se2_space->as<ob::SE2StateSpace>()->setBounds(se2_bounds);

  ob::StateSpacePtr velocity(new ob::RealVectorStateSpace(1));

  ob::RealVectorBounds velocity_bounds(1);
  velocity_bounds.setLow(0);
  velocity->as<ob::RealVectorStateSpace>()->setBounds(velocity_bounds);

  ob::StateSpacePtr space = se2_space + velocity;


  oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 1));

  ob::RealVectorBounds cbounds(1);
  cbounds.setLow(-M_PI / 3);
  cbounds.setHigh(M_PI / 3);
  cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);
  
  // define a simple setup class
  ss_ = std::make_shared<oc::SimpleSetup>(cspace);
  oc::SpaceInformationPtr si(ss_->getSpaceInformation());

  ob::StateValidityCheckerPtr vc(new ValidityChecker(si, obstacles_));

  // set state validity checking for this space
  ss_->setStateValidityChecker(vc);

  oc::StatePropagatorPtr statePropagator(new SailboatStatePropagator(si, wind_direction_));

  ss_->setStatePropagator(statePropagator);

  // set the start and goal states
  ss_->getProblemDefinition()->setStartAndGoalStates(start, goal);

  // this call is optional, but we put it in to get more output information
  ss_->setup();
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
