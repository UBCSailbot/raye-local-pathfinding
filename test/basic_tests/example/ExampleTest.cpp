// Copyright 2017 UBC Sailbot

#include "ExampleTest.h"
#include <example/Planner.h>

ExampleTest::ExampleTest() {}

TEST_F(ExampleTest, AddTest) {
  example.add(2, 2);
  EXPECT_EQ(4, example.add(2, 2));
}

TEST_F(ExampleTest, PlannerTest) {
  Planner p;

  p.planWithSimpleSetup();

  EXPECT_EQ(4, example.add(2, 2));
}
