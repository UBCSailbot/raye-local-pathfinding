// Copyright 2019 UBC Sailbot

#ifndef PLANNING_SAILBOATSTATEPROPAGATOR_H_
#define PLANNING_SAILBOATSTATEPROPAGATOR_H_

#include <ompl/control/StatePropagator.h>

class SailboatStatePropagator : public ompl::control::StatePropagator {
 public:
  SailboatStatePropagator(const ompl::control::SpaceInformationPtr &si,
                          float windDirection);

 private:
  void propagate(const ompl::base::State *state,
                 const ompl::control::Control *control,
                 double duration,
                 ompl::base::State *result) const override;

  bool canPropagateBackward() const override;

  bool canSteer() const override;

  float wind_angle_;
};

#endif  // PLANNING_SAILBOATSTATEPROPAGATOR_H_
