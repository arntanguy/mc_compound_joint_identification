#include "CompoundJointIdentification_Initial.h"

#include <mc_rtc/io_utils.h>
#include <geos/algorithm/ConvexHull.h>
#include <geos/geom/CoordinateSequenceFactory.h>
#include <geos/geom/Geometry.h>
#include "../CompoundJointIdentification.h"
#include <numeric>

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
  using PlotStyle = mc_rtc::gui::plot::Style;

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
      mc_rtc::gui::plot::Polygon(
          "Convex hull",
          [this]() {
            return PolygonDescription(hull_, Color::Magenta).style(PlotStyle::Dotted).fill(Color(1.0, 0.0, 0.0, 0.5));
          }),
      mc_rtc::gui::plot::Polygon("Compound Joint Polygon", [this]() -> PolygonDescription {
        auto vertices = PointVector{};
        if(!lines_.size()) return {};
        vertices.push_back(lines_.front().first);
        for(const auto & line : lines_)
        {
          vertices.push_back(line.second);
        }
        return PolygonDescription{vertices, Color::Green}.fill(Color(0.0, 1.0, 0.0, 0.5));
      }));

  ctl.gui()->addElement(
      {},
      mc_rtc::gui::NumberInput(
          "Max regression error", [this]() { return maxError_; }, [this](double error) { maxError_ = error; }),
      mc_rtc::gui::Label("Number of constraint lines", [this]() { return lines_.size(); }),
      mc_rtc::gui::Button("Reset",
                          [this]() {
                            reset();
                            output("");
                          }),
      mc_rtc::gui::Button("Done", [this]() {
        printConstraint();
        output("OK");
      }));
}

bool CompoundJointIdentification_Initial::run(mc_control::fsm::Controller & ctl)
{
  if(iter_ == 0 || iter_ % iterRate_ == 0)
  {
    readJoints(ctl);
    computeConvexHull(q_);
    linearRegression(hull_);
    printConstraint();
    iter_ = 0;
  }

  ++iter_;
  return !output().empty();
}

void CompoundJointIdentification_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CompoundJointIdentification &>(ctl_);
  printConstraint();
}

void CompoundJointIdentification_Initial::printConstraint()
{
  if(lines_.empty())
  {
    mc_rtc::log::error("No constraint lines found!");
    return;
  }

  std::vector<std::string> constraintLine;
  for(const auto & line : lines_)
  {
    constraintLine.push_back(fmt::format("{{ \"{}\", \"{}\", {{{}, {}}}, {{{}, {}}} }}", joints_.first, joints_.second,
                                         line.first[0], line.first[1], line.second[0], line.second[1]));
  }
  mc_rtc::log::info("Compound Joint Constraint for joints {} / {} is:\n{{\n{}\n}}", joints_.first, joints_.second,
                    mc_rtc::io::to_string(constraintLine, ",\n"));
}

void CompoundJointIdentification_Initial::readJoints(mc_control::fsm::Controller & ctl)
{
  const auto & r = ctl.robot();
  q_.push_back({r.encoderValues()[jIdx_.first], r.encoderValues()[jIdx_.second]});
}

void CompoundJointIdentification_Initial::reset()
{
  hull_.clear();
  q_.clear();
  lines_.clear();
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
    // mc_rtc::log::info("Hull {}: {}", i, hull_.back()[0], hull_.back()[1]);
  }
}

// y = a * x + b
// where a is the slope
// b is the intercept
template<typename T>
std::tuple<T, T, T> GetLinearFit(const std::vector<T> & xData, const std::vector<T> & yData)
{
  const auto & x = xData;
  const auto & y = yData;
  const auto n = x.size();
  const auto xSum = std::accumulate(x.begin(), x.end(), 0.0);
  const auto ySum = std::accumulate(y.begin(), y.end(), 0.0);
  const auto xxSum = std::inner_product(x.begin(), x.end(), x.begin(), 0.0);
  const auto xySum = std::inner_product(x.begin(), x.end(), y.begin(), 0.0);

  auto slope = (yData.size() * xySum - xSum * ySum) / (yData.size() * xxSum - xSum * xSum);
  auto intercept = (ySum - slope * xSum) / yData.size();

  // Compute error
  size_t i = 0;
  auto error = std::accumulate(xData.begin(), xData.end(), 0., [&slope, &yData, &intercept, &i](double acc, double x) {
    return std::fabs((slope * x + intercept) - yData[i]);
  });

  return {slope, intercept, error};
}

void CompoundJointIdentification_Initial::linearRegression(const std::vector<std::array<double, 2>> & hull)
{
  lines_.clear();
  // y = a * x + b

  mc_rtc::log::warning("Fitting lines");
  // Iteratively find lines fitting under a specified threshold
  auto xData = std::vector<double>{};
  auto yData = std::vector<double>{};
  for(const auto & hPoint : hull_)
  {
    xData.push_back(hPoint[0]);
    yData.push_back(hPoint[1]);
    auto [slope, intercept, error] = GetLinearFit(xData, yData);
    if(error >= maxError_)
    {
      lines_.push_back({// { xData.front(), slope * xData.front() + intercept},
                        // { xData.back(), slope * xData.back() + intercept}
                        {xData.front(), yData.front()},
                        {xData.back(), yData.back()}});
      mc_rtc::log::info("Linear fit: slope={:.2f} | intercept={:.2f} | error={:.2f}", slope, intercept, error);
      xData.clear();
      yData.clear();
      xData.push_back(hPoint[0]);
      yData.push_back(hPoint[1]);
    }
  }
  if(lines_.size())
  {
    // Close the polygon
    lines_.push_back({lines_.back().second, {hull.back()[0], hull.back()[1]}});
  }
}

EXPORT_SINGLE_STATE("CompoundJointIdentification_Initial", CompoundJointIdentification_Initial)
