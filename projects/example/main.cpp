// Copyright 2017 UBC Sailbot

#include <iostream>
#include <ompl/config.h>
#include <planning/Planner.h>

int main(int, char *[]) {
  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
  Planner p(0, std::vector<Obstacle>(), 0, 0, nullptr, nullptr);

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
