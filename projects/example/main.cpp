// Copyright 2017 UBC Sailbot

#include <iostream>
#include <example/Example.h>
#include <ompl/config.h>
#include <example/Planner.h>

int main(int, char *[]) {
  Planner p;
  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

  p.planWithSimpleSetup();

  std::cout << std::endl << std::endl;
}
