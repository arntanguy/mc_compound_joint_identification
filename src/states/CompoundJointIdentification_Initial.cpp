#include "CompoundJointIdentification_Initial.h"

#include <geos/algorithm/ConvexHull.h>
#include <geos/geom/CoordinateSequenceFactory.h>
#include <geos/geom/Geometry.h>
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

  geom_factory = geos::geom::GeometryFactory::create();
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
          "Encoder measurements", [this]() { return q_.back()[0]; }, [this]() { return q_.back()[1]; }, Color::Blue),
      mc_rtc::gui::plot::Polygon("Joint limits", [redSquareBlueFill]() { return redSquareBlueFill; }),
      mc_rtc::gui::plot::Polygon("Convex hull", [this]() { return PolygonDescription(hull_, Color::Magenta); })

  );

  ctl.gui()->addElement({}, mc_rtc::gui::Button("Done", [this]() { output("OK"); }));
}

bool CompoundJointIdentification_Initial::run(mc_control::fsm::Controller & ctl)
{
  readJoints(ctl);
  computeConvexHull(q_);

  return !output().empty();
}

void CompoundJointIdentification_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CompoundJointIdentification &>(ctl_);
}

void CompoundJointIdentification_Initial::readJoints(mc_control::fsm::Controller & ctl)
{
  const auto & r = ctl.robot();
  q_.push_back({r.encoderValues()[jIdx_.first], r.encoderValues()[jIdx_.second]});
}

void CompoundJointIdentification_Initial::computeConvexHull(const std::vector<std::array<double, 2>> & points)
{
  auto seq = geom_factory->getCoordinateSequenceFactory()->create(static_cast<size_t>(0), 0);
  std::vector<geos::geom::Coordinate> seq_points;
  for(const auto & p : points)
  {
    seq_points.push_back(geos::geom::Coordinate(p[0], p[1]));
  }
  seq_points.push_back(seq_points[0]);
  seq->setPoints(seq_points);

  auto geom_points = geom_factory->createMultiPoint(*seq);
  auto geom_hull = geos::algorithm::ConvexHull(geom_points).getConvexHull();

  auto hullseq = geom_hull->getCoordinates();
  hull_.clear();
  for(size_t i = 0; i < hullseq->size(); ++i)
  {
    hull_.push_back({hullseq->getX(i), hullseq->getY(i)});
    mc_rtc::log::info("Hull {}: {}", i, hull_.back()[0], hull_.back()[1]);
  }
}

EXPORT_SINGLE_STATE("CompoundJointIdentification_Initial", CompoundJointIdentification_Initial)
