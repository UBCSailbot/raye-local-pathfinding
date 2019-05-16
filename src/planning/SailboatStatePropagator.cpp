// Copyright 2019 UBC Sailbot

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <util/AngleCalc.h>
#include <ompl/control/SpaceInformation.h>
#include "SailboatStatePropagator.h"

SailboatStatePropagator::SailboatStatePropagator(const ompl::control::SpaceInformationPtr &si,
                                                 float windDirection)
    : StatePropagator(si), wind_angle_(windDirection) {}

void SailboatStatePropagator::propagate(const ompl::base::State *state,
                                        const ompl::control::Control *controlState,
                                        double duration,
                                        ompl::base::State *result) const {

  const auto *start = state->as<ompl::base::SE2StateSpace::StateType>();
  auto *end = result->as<ompl::base::SE2StateSpace::StateType>();
  const auto *control = controlState->as<ompl::control::RealVectorControlSpace::ControlType>();

  // Hacky formulas to allow wind direction to influence boat speed. Should be replace with real ones later
  const auto new_angle = start->getYaw() + control->values[0] * duration * 0.6;
  const auto boat_wind_angle = AngleCalc::absoluteDistanceBetweenAngles(AngleCalc::reverseAngle(wind_angle_), new_angle);
  auto boat_speed = pow((boat_wind_angle * 10) / (M_PI / 2), 1.1) / 3;
  const auto rotation_cost = AngleCalc::absoluteDistanceBetweenAngles(new_angle, start->getYaw()) / M_PI;
  boat_speed -= rotation_cost * 0.1;

  end->setX(start->getX() + boat_speed * duration * cos(start->getYaw()));
  end->setY(start->getY() + boat_speed * duration * sin(start->getYaw()));
  end->setYaw(new_angle);
}
bool SailboatStatePropagator::canPropagateBackward() const {
  return false;
}
bool SailboatStatePropagator::canSteer() const {
  return true;
}
