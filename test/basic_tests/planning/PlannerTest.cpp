// Copyright 2019 UBC Sailbot

#include "PlannerTest.h"
#include <planning/Planner.h>

PlannerTest::PlannerTest() {}

TEST_F(PlannerTest, PlannerTestNoObstacles) {
  ompl::base::StateSpacePtr space(new ompl::base::SE2StateSpace());
  ompl::base::ScopedState<ompl::base::SE2StateSpace> start(space);
  start->setX(0);
  start->setY(0);
  start->setYaw(M_PI/ 4);

  ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(space);
  goal->setX(5);
  goal->setY(5);
  goal->setYaw(M_PI/ 4);

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
  ompl::base::StateSpacePtr space(new ompl::base::SE2StateSpace());
  ompl::base::ScopedState<ompl::base::SE2StateSpace> start(space);
  start->setX(0);
  start->setY(0);
  start->setYaw(M_PI/ 4);

  ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(space);
  goal->setX(5);
  goal->setY(5);
  goal->setYaw(M_PI/ 4);


  Planner p(0, std::vector<Obstacle>({Obstacle(2.5, 2.5, 0.4)}), -2, 7, start, goal);

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
