// Copyright 2019 UBC Sailbot

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <util/AngleCalc.h>
#include <cmath>
#include <ompl/geometric/SimpleSetup.h>

#include "SailboatStatePropagator.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

SailboatStatePropagator::SailboatStatePropagator(ompl::control::SpaceInformation *si, float wind_angle)
    : StatePropagator(si), wind_angle_(wind_angle) {}


void SailboatStatePropagator::propagate(const ob::State *state,
                                        const oc::Control *control,
                                        double duration,
                                        ob::State *result) const {

  const auto *start_state = state->as<ob::SE2StateSpace::StateType>();
  const auto *control_state = result->as<oc::RealVectorControlSpace::ControlType>();
  auto *result_state = control->as<ob::SE2StateSpace::StateType>();


  auto new_angle = start_state->getYaw() + (*control_state)[0] * duration * 0.6;

//Angle between the boat and the wind
  auto boat_wind_angle =  AngleCalc::absoltuteDistanceBetweenAngles(AngleCalc::reverseAngle(wind_angle_), new_angle);

// Hacky formulas to allow wind direction to influence boat speed. Should be replace with real ones later
  auto boat_speed = std::pow((boat_wind_angle * 10) / (M_PI / 2), 1.1) / 3;
  auto rotation_cost = AngleCalc::absoltuteDistanceBetweenAngles(new_angle, start_state->getYaw()) / M_PI;
  boat_speed -= rotation_cost * 0.1;

  result_state->setX(start_state->getX() + boat_speed * duration * std::cos(start_state->getYaw()));
  result_state->setY(start_state->getY() + boat_speed * duration * std::sin(start_state->getYaw()));
  result_state->setYaw(new_angle);
}
bool SailboatStatePropagator::canPropagateBackward() const {
  return StatePropagator::canPropagateBackward();
}

bool SailboatStatePropagator::canSteer() const {
  return false;
}
