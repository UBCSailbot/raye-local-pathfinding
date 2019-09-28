// Copyright 2017 UBC Sailbot

#include <iostream>
#include <ompl/config.h>
#include <planning/Planner.h>

int main(int, char *[]) {
  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

  ompl::base::StateSpacePtr space(Planner::getStateSpace(0, 5));
  ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(space);
  start->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(0, 1);
  start->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(M_PI / 4);
  start->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0] = 0;

  BoatPosition goal(Coordinate(4, 4), M_PI / 4);

  Planner p(0, std::vector<Obstacle>(), -2, 7, start, goal);

  p.printSetup();
  auto solved = p.Solve(1.0);

  if (solved) {
    std::cout << "Found solution:" << std::endl;
    // print the path to screen
    p.getPath().print(std::cout);
  } else {
    std::cout << "No solution found" << std::endl;
  }

  std::cout << std::endl << std::endl;
}
