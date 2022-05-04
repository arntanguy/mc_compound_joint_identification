#include "CompoundJointIdentification_Initial.h"

#include "../CompoundJointIdentification.h"

void CompoundJointIdentification_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CompoundJointIdentification &>(ctl_);

  if(!config_.has("joints"))
  {
    mc_rtc::log::error_and_throw("[{}] Missing \"joints\" configuration", name());
  }

  joints_ = (config_("joints"));

  const auto & r = ctl.robot();
  auto check_joint = [&r, this](const std::string & j) {
    if(!r.hasJoint(joints_.first))
    {
      mc_rtc::log::error_and_throw("[{}] No joint named \"{}\" in robot \"{}\"", name(), j, r.name());
    }
  };

  auto encoderIndex = [&ctl](const std::string & joint) {
    const auto & ref_joint_order = ctl.robot().refJointOrder();
    auto it = std::find(ref_joint_order.begin(), ref_joint_order.end(), joint);
    return std::distance(ref_joint_order.begin(), it);
  };

  check_joint(joints_.first);
  check_joint(joints_.second);
  jIdx_ = {encoderIndex(joints_.first), encoderIndex(joints_.second)};

  run(ctl);

  using Color = mc_rtc::gui::Color;
  using PolygonDescription = mc_rtc::gui::plot::PolygonDescription;

  auto get_joint_limits = [this, &ctl](const size_t jidx) -> std::pair<double, double> {
    const auto & jointLimits = ctl.robot().module().bounds();
    const auto & ref_joint_order = ctl.robot().refJointOrder();
    auto lower = jointLimits[0].at(ref_joint_order[jidx])[0];
    auto upper = jointLimits[1].at(ref_joint_order[jidx])[0];
    return {lower, upper};
  };

  auto q1Limits = get_joint_limits(jIdx_.first);
  auto q1l = std::min(q1Limits.first, q1Limits.second);
  auto q1u = std::max(q1Limits.first, q1Limits.second);
  auto q2Limits = get_joint_limits(jIdx_.second);
  auto q2l = std::min(q2Limits.first, q2Limits.second);
  auto q2u = std::max(q2Limits.first, q2Limits.second);
  mc_rtc::log::info("[{}] Joint limits are:\n{}: [{}, {}]\n{}: [{}, {}]", name(), joints_.first, q1l, q1u,
                    joints_.second, q2l, q2u);

  auto redSquareBlueFill = PolygonDescription({{q1l, q2l}, {q1u, q2l}, {q1u, q2u}, {q1l, q2u}}, Color::Red);

  ctl.gui()->addXYPlot(
      fmt::format("Compound joint {}/{}", joints_.first, joints_.second),
      mc_rtc::gui::plot::XY(
          "Compound joint", [this]() { return q_.back().first; }, [this]() { return q_.back().second; }, Color::Blue),
      mc_rtc::gui::plot::Polygon("Joint limits", [redSquareBlueFill]() { return redSquareBlueFill; })

  );

  ctl.gui()->addElement({}, mc_rtc::gui::Button("Done", [this]() { output("OK"); }));
}

bool CompoundJointIdentification_Initial::run(mc_control::fsm::Controller & ctl)
{
  readJoints(ctl);

  return !output().empty();
}

void CompoundJointIdentification_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CompoundJointIdentification &>(ctl_);
}

void CompoundJointIdentification_Initial::readJoints(mc_control::fsm::Controller & ctl)
{
  const auto & r = ctl.robot();
  q_.emplace_back(r.encoderValues()[jIdx_.first], r.encoderValues()[jIdx_.second]);
  mc_rtc::log::info("Value: {} / {}", q_.back().first, q_.back().second);
}

EXPORT_SINGLE_STATE("CompoundJointIdentification_Initial", CompoundJointIdentification_Initial)
