// Copyright 2019 UBC Sailbot


#ifndef DATATYPES_COORDINATE_H_
#define DATATYPES_COORDINATE_H_

class Coordinate {
 public:
  Coordinate(double x, double y) : x_(x), y_(y) {}

  double getX() const {
    return x_;
  }
  double getY() const {
    return y_;
  }
 private:
  double x_;
  double y_;
};

class BoatPosition {
 public:
  BoatPosition(const Coordinate &coordinate, double yaw) : yaw_(yaw), coordinate_(coordinate) {}
 public:
  double getX() const {
    return coordinate_.getX();
  }

  double getY() const {
    return coordinate_.getY();
  }
  double getYaw() const {
    return yaw_;
  }

 private:
  const double yaw_;
  const Coordinate coordinate_;
};

#endif  // DATATYPES_COORDINATE_H_
