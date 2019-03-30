//
// Created by denis on 30/03/19.
//

#include <cmath>
#include "AngleCalc.h"
double AngleCalc::absoltuteDistanceBetweenAngles(double angle1, double angle2) {
  return std::abs(std::atan2(std::sin(angle1 - angle2), std::cos(angle1 - angle2)));

}
double AngleCalc::reverseAngle(double angle) {
  if (angle <= 0) {
    return angle + M_PI;
  }
  else{
    return angle - M_PI;

  }
}
