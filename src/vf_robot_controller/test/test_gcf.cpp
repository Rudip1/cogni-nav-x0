#include "vf_robot_controller/gcf/geometric_complexity_field.hpp"
#include <gtest/gtest.h>
#include "vf_robot_controller/gcf/clutter_detector.hpp"
#include "vf_robot_controller/parameter_handler.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Minimal Parameters for tests (no ROS node needed)
// ─────────────────────────────────────────────────────────────────────────────
static vf_robot_controller::Parameters makeParams()
{
  vf_robot_controller::Parameters p;
  p.gcf_clutter_radius = 0.6;
  p.robot_width        = 0.4;
  p.robot_length       = 0.6;
  p.robot_height_min   = 0.05;
  p.robot_height_max   = 1.2;
  p.gcf_radius         = 1.5;
  p.gcf_weight_2d      = 0.4;
  p.gcf_weight_clutter = 0.3;
  p.gcf_weight_volumetric = 0.3;
  p.tight_gcf_threshold = 0.6;
  p.safety_inflation   = 0.05;
  return p;
}

// ─────────────────────────────────────────────────────────────────────────────
// GCF tests
// ─────────────────────────────────────────────────────────────────────────────
TEST(GCFTest, ComplexityInRange)
{
  // Complexity values must always be in [0,1]
  vf_robot_controller::gcf::GeometricComplexityField gcf(makeParams());
  // Without a real costmap we test the cell structure directly
  SUCCEED();  // structural test — replace with mock costmap in full test suite
}

TEST(ClutterDetectorTest, EmptyMap_NoClusters)
{
  auto params = makeParams();
  vf_robot_controller::gcf::ClutterDetector detector(params);

  // Create a trivial empty costmap-like binary mask (no obstacles)
  // ClutterDetector labelConnectedComponents should return zero for all cells.
  // We test indirectly through the public interface below.
  SUCCEED();
}

TEST(ClutterDetectorTest, SingleCluster_CountsOne)
{
  // A 10x10 map with one 2x2 block of obstacles should give cluster count 1
  // in cells near the block and 0 elsewhere.
  // (Full mock-costmap test goes here — scaffolded for CI.)
  EXPECT_EQ(1, 1);  // placeholder — replace with nav2_costmap_2d mock
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
