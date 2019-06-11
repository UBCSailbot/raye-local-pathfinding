// Copyright 2019 UBC Sailbot

#ifndef DATATYPES_OBSTACLE_H_
#define DATATYPES_OBSTACLE_H_

#include "Coordinate.h"

class Obstacle {
 public:
  Obstacle(Coordinate coordinate, double radius) :coordinate_(coordinate),  radius_(radius) {
  }
  double getX() const {
    return coordinate_.getX();
  }
  double getY() const {
    return coordinate_.getY();
  }
  double getRadius() const {
    return radius_;
  }

 private:
  Coordinate coordinate_;
  double radius_;
};

#endif   // DATATYPES_OBSTACLE_H_
