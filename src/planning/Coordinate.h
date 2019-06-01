// Copyright 2019 UBC Sailbot


#ifndef PLANNING_COORDINATE_H_
#define PLANNING_COORDINATE_H_

class Coordinate {
 public:
  Coordinate(double x, double y, double yaw) : x_(x), y_(y), yaw_(yaw) {}
 public:
  double getX() const {
    return x_;
  }

  double getY() const {
    return y_;
  }
  double getYaw() const {
    return yaw_;
  }

 private:
  const double x_;
  const double y_;
  const double yaw_;
};

#endif  // PLANNING_COORDINATE_H_
