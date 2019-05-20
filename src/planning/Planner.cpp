#include <utility>

// Copyright 2019 UBC Sailbot

#include "Planner.h"
#include "SailboatStatePropagator.h"
#include "ValidityChecker.h"

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

Planner::Planner(float windDirection, std::vector<Obstacle> obstacles)
    : wind_direction_(windDirection), obstacles_(std::move(obstacles)) {
  // construct the state space we are planning in
  ob::StateSpacePtr space(new ob::SE2StateSpace());

  ob::RealVectorBounds bounds(2);
  bounds.setLow(0);
  bounds.setHigh(5);

  space->as<ob::SE2StateSpace>()->setBounds(bounds);

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

  // create a random start state
  ob::ScopedState<> start(space);
  start[0] = 0;
  start[1] = 0;
  start[2] = M_PI/ 4;

  // create a random goal state
  ob::ScopedState<> goal(space);
  goal[0] = 5;
  goal[1] = 5;
  goal[2] = M_PI/ 4;
  
  // set the start and goal states
  ss_->setStartAndGoalStates(start, goal);

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
