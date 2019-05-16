// Copyright 2019 UBC Sailbot

#include "Planner.h"
#include "SailboatStatePropagator.h"

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

Planner::Planner(float windDirection) : wind_direction_(windDirection) {
  // construct the state space we are planning in
  ob::StateSpacePtr space(new ob::SE2StateSpace());

  ob::RealVectorBounds bounds(2);
  bounds.setLow(-1);
  bounds.setHigh(1);

  space->as<ob::SE2StateSpace>()->setBounds(bounds);

  oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 1));

  ob::RealVectorBounds cbounds(1);
  cbounds.setLow(-M_PI / 3);
  cbounds.setHigh(M_PI / 3);
  cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

  // define a simple setup class
  ss_ = std::make_shared<oc::SimpleSetup>(cspace);

  // set state validity checking for this space
  ss_->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });

  ompl::control::SpaceInformationPtr si = ss_->getSpaceInformation();
  oc::StatePropagatorPtr statePropagator(new SailboatStatePropagator(si.get(), wind_direction_));

  ss_->setStatePropagator(statePropagator);

  // create a random start state
  ob::ScopedState<> start(space);
  start.random();

  // create a random goal state
  ob::ScopedState<> goal(space);
  goal.random();

  // set the start and goal states
  ss_->setStartAndGoalStates(start, goal);

  // this call is optional, but we put it in to get more output information
  ss_->setup();
}

bool Planner::isStateValid(const ob::State *state) {
  return true;
}

ob::PlannerStatus Planner::Solve() {
  // attempt to solve the problem within one second of planning time
  ob::PlannerStatus solved = ss_->solve(1.0);
  return solved;
}

void Planner::printSetup() {
  ss_->print();
}

ompl::control::PathControl &Planner::getPath() {
  return ss_->getSolutionPath();
}
