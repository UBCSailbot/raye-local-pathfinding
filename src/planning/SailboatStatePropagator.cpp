// Copyright 2019 UBC Sailbot

#include "SailboatStatePropagator.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <util/AngleCalc.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/TimeStateSpace.h>

SailboatStatePropagator::SailboatStatePropagator(const ompl::control::SpaceInformationPtr &si,
                                                 float windDirection)
    : StatePropagator(si), wind_angle_(windDirection) {}

void SailboatStatePropagator::propagate(const ompl::base::State *state,
                                        const ompl::control::Control *controlState,
                                        double duration,
                                        ompl::base::State *result) const {
  const auto start_compound_state = state->as<ompl::base::CompoundStateSpace::StateType>();

  // Has the position of the boat in x, y, yaw
  const auto start_pos = start_compound_state->as<ompl::base::SE2StateSpace::StateType>(0);

  // Current boat velocity in m/s
  auto start_velocity = start_compound_state->as<ompl::base::RealVectorStateSpace::StateType>(1);
  // Current time in seconds
  auto start_time = start_compound_state->as<ompl::base::TimeStateSpace::StateType>(2);

  const auto end_compound_state = result->as<ompl::base::CompoundStateSpace::StateType>();
  // Output position
  auto end_pos = end_compound_state->as<ompl::base::SE2StateSpace::StateType>(0);
  // Output velocity in m/s
  auto end_velocity = end_compound_state->as<ompl::base::RealVectorStateSpace::StateType>(1);
  // Next time in seconds
  auto end_time = end_compound_state->as<ompl::base::TimeStateSpace::StateType>(2);

  const auto *control = controlState->as<ompl::control::RealVectorControlSpace::ControlType>();

  //  Hacky formulas to allow wind direction to influence boat speed. Should be replace with real ones later
  const auto new_angle = start_pos->getYaw() + control->values[0] * duration * 0.6;
  const auto
      boat_wind_angle = AngleCalc::absoluteDistanceBetweenAngles(AngleCalc::reverseAngle(wind_angle_), new_angle);
  auto boat_speed = pow((boat_wind_angle * 10) / (M_PI / 2), 1.1) / 3;
  const auto rotation_cost = AngleCalc::absoluteDistanceBetweenAngles(new_angle, start_pos->getYaw()) / M_PI;
  boat_speed -= rotation_cost * 0.1;

  end_pos->setX(start_pos->getX() + boat_speed * duration * cos(start_pos->getYaw()));
  end_pos->setY(start_pos->getY() + boat_speed * duration * sin(start_pos->getYaw()));
  end_pos->setYaw(new_angle);
  end_time->position = start_time->position + duration;
}

bool SailboatStatePropagator::canPropagateBackward() const {
  return false;
}
bool SailboatStatePropagator::canSteer() const {
  return true;
}
