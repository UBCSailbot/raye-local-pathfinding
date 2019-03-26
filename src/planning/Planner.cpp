#include <memory>

// Copyright 2019 UBC Sailbot

#include "Planner.h"

#include <ompl/base/spaces/SE2StateSpace.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

Planner::Planner() {
  // construct the state space we are planning in
  ob::StateSpacePtr space(new ob::SE2StateSpace());

  ob::RealVectorBounds bounds(2);
  bounds.setLow(-1);
  bounds.setHigh(1);

  space->as<ob::SE2StateSpace>()->setBounds(bounds);
  // define a simple setup class
  ss = std::make_shared<og::SimpleSetup>(space);

  // set state validity checking for this space
  ss->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });

  // create a random start state
  ob::ScopedState<> start(space);
  start.random();

  // create a random goal state
  ob::ScopedState<> goal(space);
  goal.random();

  // set the start and goal states
  ss->setStartAndGoalStates(start, goal);

  // this call is optional, but we put it in to get more output information
  ss->setup();
}

bool Planner::isStateValid(const ob::State *state) {
  return true;
}

ob::PlannerStatus Planner::Solve() {
  // attempt to solve the problem within one second of planning time
  ob::PlannerStatus solved = ss->solve(1.0);
  return solved;
}

void Planner::printSetup() {
  ss->print();
}

ompl::geometric::PathGeometric &Planner::getPath() {
  ss->simplifySolution();
  return ss->getSolutionPath();
}
