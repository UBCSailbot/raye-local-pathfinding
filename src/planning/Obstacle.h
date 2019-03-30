// Copyright 2019 UBC Sailbot

#ifndef PLANNING_OBSTACLE_H_
#define PLANNING_OBSTACLE_H_

class Obstacle {
 public:
  Obstacle(float x, float y, float radius) : x_(x), y_(y), radius_(radius) {
  }
  float getX() const {
    return x_;
  }
  float getY() const {
    return y_;
  }
  float getRadius() const {
    return radius_;
  }

 private:
  float x_;
  float y_;
  float radius_;
};

#endif   // PLANNING_OBSTACLE_H_
