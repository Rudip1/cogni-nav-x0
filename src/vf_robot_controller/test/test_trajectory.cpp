#include "vf_robot_controller/models/state.hpp"
#include <gtest/gtest.h>
#include "vf_robot_controller/models/trajectory.hpp"
#include <cmath>

using namespace vf_robot_controller::models;

TEST(BSplineTest, BuildKnotsLength)
{
  // For n control points and degree p: knot vector length = n + p + 1
  const int n = 8, p = 3;
  auto knots = BSplineTrajectory::buildKnots(n, p);
  EXPECT_EQ(static_cast<int>(knots.size()), n + p + 1);
}

TEST(BSplineTest, KnotVectorClamped)
{
  auto knots = BSplineTrajectory::buildKnots(6, 3);
  // First p+1 values must be 0, last p+1 values must be 1
  for (int i = 0; i <= 3; ++i) EXPECT_DOUBLE_EQ(knots[i], 0.0);
  for (int i = static_cast<int>(knots.size()) - 4;
       i < static_cast<int>(knots.size()); ++i) {
    EXPECT_DOUBLE_EQ(knots[i], 1.0);
  }
}

TEST(BSplineTest, EvaluateEndpoints)
{
  // A B-spline with all control points on x-axis should evaluate on x-axis
  BSplineTrajectory traj;
  traj.degree = 3;
  traj.control_pts = {
    {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0},
    {3.0, 0.0}, {4.0, 0.0}, {5.0, 0.0}
  };
  traj.knots = BSplineTrajectory::buildKnots(6, 3);

  const auto start = traj.evaluate(0.0);
  const auto end   = traj.evaluate(1.0 - 1e-9);

  EXPECT_NEAR(start.x(), 0.0, 1e-6);
  EXPECT_NEAR(start.y(), 0.0, 1e-6);
  EXPECT_NEAR(end.x(), 5.0, 0.1);  // clamped B-spline interpolates endpoints
  EXPECT_NEAR(end.y(), 0.0, 1e-6);
}

TEST(BSplineTest, SampleSize)
{
  BSplineTrajectory traj;
  traj.degree = 3;
  traj.control_pts = {
    {0,0},{1,0},{2,1},{3,0},{4,0},{5,0}
  };
  traj.knots = BSplineTrajectory::buildKnots(6, 3);

  const auto pts = traj.sample(20);
  EXPECT_EQ(static_cast<int>(pts.size()), 20);
}

TEST(BSplineTest, FromWaypoints_CorrectNumCtrl)
{
  std::vector<Eigen::Vector2d> wps = {
    {0,0},{1,0.5},{2,1},{3,0.5},{4,0}
  };
  auto traj = BSplineTrajectory::fromWaypoints(wps, 8, 3);
  EXPECT_EQ(traj.numControlPoints(), 8);
  EXPECT_EQ(static_cast<int>(traj.knots.size()), 8 + 3 + 1);
}

TEST(BSplineTest, PerturbedDiffersFromBase)
{
  BSplineTrajectory traj;
  traj.degree = 3;
  traj.control_pts = {{0,0},{1,0},{2,0},{3,0},{4,0},{5,0}};
  traj.knots = BSplineTrajectory::buildKnots(6, 3);

  std::vector<Eigen::Vector2d> noise(6, {0.0, 0.0});
  noise[2] = {0.0, 0.3};  // perturb only middle point

  auto perturbed = traj.perturbed(noise);
  // Middle control point should differ
  EXPECT_NEAR(perturbed.control_pts[2].y(), 0.3, 1e-9);
  // Other points unchanged
  EXPECT_NEAR(perturbed.control_pts[0].y(), 0.0, 1e-9);
}

TEST(SegmentMapTest, TotalKnotsRespectsBudget)
{
  const std::vector<double> complexity = {0.1, 0.8, 0.9, 0.2, 0.7};
  auto map = vf_robot_controller::models::SegmentMap::build(complexity, 2, 18, 0.6);
  EXPECT_LE(map.total_knots, 18);
  EXPECT_GE(map.total_knots, 2 * static_cast<int>(complexity.size()));
}

TEST(SegmentMapTest, TightSegmentsMarked)
{
  const std::vector<double> complexity = {0.2, 0.9, 0.3};
  auto map = vf_robot_controller::models::SegmentMap::build(complexity, 2, 12, 0.6);
  EXPECT_FALSE(map.segments[0].is_tight);
  EXPECT_TRUE(map.segments[1].is_tight);   // 0.9 > 0.6 threshold
  EXPECT_FALSE(map.segments[2].is_tight);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
