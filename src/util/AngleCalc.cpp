// Copyright 2019 UBC Sailbot

#include "AngleCalc.h"
#include <cmath>

namespace AngleCalc {

double absoluteDistanceBetweenAngles(float angle1, float angle2) {
  return std::abs(std::atan2(std::sin(angle1 - angle2), std::cos(angle1 - angle2)));
}

double reverseAngle(float angle) {
  if (angle <= 0) {
    return angle + M_PI;
  } else {
    return angle - M_PI;
  }
}

}  // namespace AngleCalc
