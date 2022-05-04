#pragma once

#include <mc_control/fsm/State.h>
#include <geos/geom/GeometryFactory.h>

struct CompoundJointIdentification_Initial : mc_control::fsm::State
{
  using Point = std::array<double, 2>;
  using PointVector = std::vector<Point>;
  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  void readJoints(mc_control::fsm::Controller & ctl);
  void computeConvexHull(const PointVector & points);
  void linearRegression(const PointVector & hull);
  void printConstraint();
  void reset();

private:
  std::pair<std::string, std::string> joints_;
  // Joint index in refJointOrder
  std::pair<size_t, size_t> jIdx_;
  // Encoder measurements
  PointVector q_;
  // Convex hull points
  PointVector hull_;
  // Constraint segments (start, end point)
  std::vector<std::pair<Point, Point>> lines_;

  // Max fitting error before creating a new line
  double maxError_ = 0.0005;
  size_t iter_ = 0;
  size_t iterRate_ = 50;

  geos::geom::GeometryFactory::Ptr geom_factory;
};
