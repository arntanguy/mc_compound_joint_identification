#pragma once

#include <mc_control/fsm/State.h>

struct CompoundJointIdentification_Initial : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  void readJoints(mc_control::fsm::Controller & ctl);

private:
  std::pair<std::string, std::string> joints_;
  // Joint index in refJointOrder
  std::pair<size_t, size_t> jIdx_;
  std::vector<std::pair<double, double>> q_;
};
