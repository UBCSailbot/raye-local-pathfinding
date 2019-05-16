// Copyright 2017 UBC Sailbot

#include "PlannerTest.h"
#include <planning/Planner.h>

PlannerTest::PlannerTest() {}

TEST_F(PlannerTest, PlannerTest) {
  Planner p(0);

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
