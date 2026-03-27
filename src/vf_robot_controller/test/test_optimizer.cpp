#include <gtest/gtest.h>
#include "vf_robot_controller/models/trajectory.hpp"
#include "vf_robot_controller/models/state.hpp"
#include "vf_robot_controller/tools/noise_generator.hpp"
#include "vf_robot_controller/parameter_handler.hpp"

static vf_robot_controller::Parameters makeParams()
{
  vf_robot_controller::Parameters p;
  p.num_samples        = 50;
  p.num_iterations     = 2;
  p.horizon_time       = 2.0;
  p.control_frequency  = 20.0;
  p.temperature        = 0.3;
  p.spline_degree      = 3;
  p.min_knots          = 4;
  p.max_knots          = 12;
  p.knot_density_scale = 1.0;
  p.noise_sigma_x      = 0.1;
  p.noise_sigma_theta  = 0.2;
  p.max_vel_x          = 0.5;
  p.min_vel_x          = 0.05;
  p.max_vel_theta      = 1.0;
  p.robot_width        = 0.4;
  p.robot_length       = 0.6;
  p.tight_gcf_threshold = 0.6;
  return p;
}

// ── Noise generator tests ─────────────────────────────────────────────────────
TEST(NoiseGeneratorTest, GeneratesCorrectNumberOfSamples)
{
  auto p = makeParams();
  vf_robot_controller::tools::NoiseGenerator gen(p, 42);

  vf_robot_controller::models::BSplineTrajectory base;
  base.degree = 3;
  base.control_pts = {{0,0},{1,0},{2,0.5},{3,0},{4,0},{5,0}};
  base.knots = vf_robot_controller::models::BSplineTrajectory::buildKnots(6, 3);

  const int n = 30;
  auto samples = gen.generateSamples(base, n, 0.0);
  EXPECT_EQ(static_cast<int>(samples.size()), n);
}

TEST(NoiseGeneratorTest, FirstSampleIsBase)
{
  auto p = makeParams();
  vf_robot_controller::tools::NoiseGenerator gen(p, 42);

  vf_robot_controller::models::BSplineTrajectory base;
  base.degree = 3;
  base.control_pts = {{0,0},{1,0},{2,0.5},{3,0},{4,0},{5,0}};
  base.knots = vf_robot_controller::models::BSplineTrajectory::buildKnots(6, 3);

  auto samples = gen.generateSamples(base, 10, 0.0);
  // First sample = zero noise = identical to base
  for (size_t i = 0; i < base.control_pts.size(); ++i) {
    EXPECT_NEAR(samples[0].control_pts[i].x(), base.control_pts[i].x(), 1e-9);
    EXPECT_NEAR(samples[0].control_pts[i].y(), base.control_pts[i].y(), 1e-9);
  }
}

TEST(NoiseGeneratorTest, FirstControlPointFixedAtOrigin)
{
  // First control point must never move — it's the robot's current position
  auto p = makeParams();
  vf_robot_controller::tools::NoiseGenerator gen(p, 99);

  vf_robot_controller::models::BSplineTrajectory base;
  base.degree = 3;
  base.control_pts = {{1.5, 2.3},{2,0},{3,0},{4,0},{5,0},{6,0}};
  base.knots = vf_robot_controller::models::BSplineTrajectory::buildKnots(6, 3);

  auto samples = gen.generateSamples(base, 20, 0.5);
  for (const auto & s : samples) {
    EXPECT_NEAR(s.control_pts[0].x(), base.control_pts[0].x(), 1e-9);
    EXPECT_NEAR(s.control_pts[0].y(), base.control_pts[0].y(), 1e-9);
  }
}

TEST(NoiseGeneratorTest, TightGCFReducesNoise)
{
  // With high GCF (tight), noise should be smaller than with low GCF (open)
  auto p = makeParams();
  p.noise_sigma_x = 0.5;  // large sigma to make effect visible
  vf_robot_controller::tools::NoiseGenerator gen(p, 7);

  vf_robot_controller::models::BSplineTrajectory base;
  base.degree = 3;
  base.control_pts = {{0,0},{1,0},{2,0},{3,0},{4,0},{5,0}};
  base.knots = vf_robot_controller::models::BSplineTrajectory::buildKnots(6, 3);

  const int N = 200;

  // Open space: gcf_mean = 0.0
  auto open_samples = gen.generateSamples(base, N, 0.0);
  double open_var = 0.0;
  for (const auto & s : open_samples) {
    const double dy = s.control_pts[2].y() - base.control_pts[2].y();
    open_var += dy * dy;
  }
  open_var /= N;

  // Tight space: gcf_mean = 1.0
  auto tight_samples = gen.generateSamples(base, N, 1.0);
  double tight_var = 0.0;
  for (const auto & s : tight_samples) {
    const double dy = s.control_pts[2].y() - base.control_pts[2].y();
    tight_var += dy * dy;
  }
  tight_var /= N;

  // Tight variance must be smaller (noise is scaled down by 0.3 factor)
  EXPECT_LT(tight_var, open_var);
}

// ── SegmentMap tests ──────────────────────────────────────────────────────────
TEST(SegmentMapTest, EmptyComplexity_ReturnsEmpty)
{
  const std::vector<double> c;
  auto map = vf_robot_controller::models::SegmentMap::build(c, 2, 12, 0.6);
  EXPECT_TRUE(map.segments.empty());
}

TEST(SegmentMapTest, UniformComplexity_EqualKnots)
{
  // All segments same complexity → approximately equal knot counts
  const std::vector<double> c(5, 0.5);
  auto map = vf_robot_controller::models::SegmentMap::build(c, 2, 20, 0.6);
  EXPECT_EQ(static_cast<int>(map.segments.size()), 5);
  for (const auto & seg : map.segments) {
    EXPECT_GE(seg.num_knots, 2);
  }
}

TEST(SegmentMapTest, HighComplexitySegmentGetsMoreKnots)
{
  // One very complex segment should get more knots than simple segments
  const std::vector<double> c = {0.1, 0.1, 0.95, 0.1, 0.1};
  auto map = vf_robot_controller::models::SegmentMap::build(c, 2, 20, 0.6);
  // Complex segment (index 2) should have most knots
  for (size_t i = 0; i < map.segments.size(); ++i) {
    if (i != 2) {
      EXPECT_LE(map.segments[i].num_knots, map.segments[2].num_knots);
    }
  }
}

TEST(SegmentMapTest, TotalNeverExceedsBudget)
{
  const std::vector<double> c = {0.9, 0.8, 0.7, 0.95, 0.85};
  // Even with high complexity everywhere, total must not exceed max_knots
  auto map = vf_robot_controller::models::SegmentMap::build(c, 2, 15, 0.6);
  EXPECT_LE(map.total_knots, 15);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
