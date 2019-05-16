// Copyright 2019 UBC Sailbot

#include "PlannerTest.h"
#include <planning/Planner.h>

PlannerTest::PlannerTest() {}

TEST_F(PlannerTest, PlannerTestNoObstacles) {
  Planner p(0, std::vector<Obstacle>());

  p.printSetup();
  auto solved = p.Solve();

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
  Planner p(0, std::vector<Obstacle>({Obstacle(0.5,0.5,0.4)}));

  p.printSetup();
  auto solved = p.Solve();

  if (solved) {
    std::cout << "Found solution:" << std::endl;
    // print the path to screen
    p.getPath().print(std::cout);
  } else {
    std::cout << "No solution found" << std::endl;
  }

  EXPECT_TRUE(solved);
}
