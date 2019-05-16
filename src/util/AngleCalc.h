// Copyright 2019 UBC Sailbot

#ifndef LOCAL_PATHFINDING_ANGLECALC_H
#define LOCAL_PATHFINDING_ANGLECALC_H

class AngleCalc {
 public:
  static double absoluteDistanceBetweenAngles(float angle1, float angle2);
  static double reverseAngle(float angle);
};

#endif //LOCAL_PATHFINDING_ANGLECALC_H
