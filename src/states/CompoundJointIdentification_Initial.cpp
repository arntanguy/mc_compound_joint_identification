#include "CompoundJointIdentification_Initial.h"

#include "../CompoundJointIdentification.h"

void CompoundJointIdentification_Initial::configure(const mc_rtc::Configuration & config)
{
}

void CompoundJointIdentification_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CompoundJointIdentification &>(ctl_);
}

bool CompoundJointIdentification_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CompoundJointIdentification &>(ctl_);
  output("OK");
  return true;
}

void CompoundJointIdentification_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CompoundJointIdentification &>(ctl_);
}

EXPORT_SINGLE_STATE("CompoundJointIdentification_Initial", CompoundJointIdentification_Initial)
