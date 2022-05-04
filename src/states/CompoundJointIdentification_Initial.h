#pragma once

#include <mc_control/fsm/State.h>
#include <geos/geom/GeometryFactory.h>

struct CompoundJointIdentification_Initial : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  void readJoints(mc_control::fsm::Controller & ctl);
  void computeConvexHull(const std::vector<std::array<double, 2>> & points);

private:
  std::pair<std::string, std::string> joints_;
  // Joint index in refJointOrder
  std::pair<size_t, size_t> jIdx_;
  // Encoder measurements
  std::vector<std::array<double, 2>> q_;
  // Convex hull points
  std::vector<std::array<double, 2>> hull_;

  geos::geom::GeometryFactory::Ptr geom_factory;
};
