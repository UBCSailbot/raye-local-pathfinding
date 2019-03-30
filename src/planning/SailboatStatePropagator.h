// Copyright 2019 UBC Sailbot

#ifndef LOCAL_PATHFINDING_SAILBOATSTATEPROPAGATOR_H
#define LOCAL_PATHFINDING_SAILBOATSTATEPROPAGATOR_H

#include <ompl/control/StatePropagator.h>

class SailboatStatePropagator : ompl::control::StatePropagator {

 public:
  SailboatStatePropagator(ompl::control::SpaceInformation *si, float wind_angle);
 private:
  void propagate(const ompl::base::State *state,
                 const ompl::control::Control *control,
                 double duration,
                 ompl::base::State *result) const override;
  bool canPropagateBackward() const override;

  bool canSteer() const override;
 private:
  float wind_angle_;

};

#endif  // LOCAL_PATHFINDING_SAILBOATSTATEPROPAGATOR_H
