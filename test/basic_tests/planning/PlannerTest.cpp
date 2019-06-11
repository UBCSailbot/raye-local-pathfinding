// Copyright 2019 UBC Sailbot

#include "PlannerTest.h"
#include <planning/Planner.h>
#include <planning/SailboatGoalRegion.h>

PlannerTest::PlannerTest() {}

TEST_F(PlannerTest, PlannerTestNoObstacles) {
  ompl::base::StateSpacePtr space(Planner::getStateSpace(0, 5));
  ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(space);
  start->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(0, 0);
  start->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(M_PI / 4);
  start->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0] = 0;

  BoatPosition goal(Coordinate(4, 4), M_PI / 4);

  Planner p(0, std::vector<Obstacle>(), 0, 5, start, goal);

  p.printSetup();
  auto solved = p.Solve(1.0);

  if (solved) {
    std::cout << "Found solution:" << std::endl;
    // print the path to screen
    p.getPath().print(std::cout);
  } else {
    std::cout << "No solution found" << std::endl;
  }

  EXPECT_TRUE(solved);
}

TEST_F(PlannerTest, PlannerTestObstacle) {
  ompl::base::StateSpacePtr space(Planner::getStateSpace(0, 5));
  ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(space);
  start->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(0, 0);
  start->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(M_PI / 4);
  start->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0] = 3;

  BoatPosition goal(Coordinate(0, 0), M_PI / 4);

  Planner p(0, std::vector<Obstacle>({Obstacle(Coordinate(2.5, 2.5), 0.4)}), -2, 7, start, goal);

  p.printSetup();
  auto solved = p.Solve(5.0);

  if (solved) {
    std::cout << "Found solution:" << std::endl;
    // print the path to screen
    p.getPath().print(std::cout);
  } else {
    std::cout << "No solution found" << std::endl;
  }

  EXPECT_TRUE(solved);
}
