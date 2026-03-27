#include <gtest/gtest.h>
#include "vf_robot_controller/safety/swept_volume_checker.hpp"
#include "vf_robot_controller/parameter_handler.hpp"

static vf_robot_controller::Parameters makeParams()
{
  vf_robot_controller::Parameters p;
  p.robot_length   = 0.6;
  p.robot_width    = 0.4;
  p.robot_height_min = 0.05;
  p.robot_height_max = 1.2;
  p.safety_inflation = 0.05;
  return p;
}

TEST(SweptVolumeTest, StationaryProducesOneFootprint)
{
  vf_robot_controller::safety::SweptVolumeChecker checker(makeParams());
  // Zero velocity — only current footprint should be in swept set
  auto cells = checker.computeSweptCells(
    0.0, 0.0, 0.0,  // pose
    0.0, 0.0,        // v=0, omega=0
    0.3,             // dt
    0.05,            // resolution
    -5.0, -5.0,      // origin
    8);

  // With zero velocity all n_steps footprints are identical → deduplicated
  // We expect a non-empty set of cells corresponding to the footprint
  EXPECT_GT(cells.size(), 0u);
}

TEST(SweptVolumeTest, ForwardMotionExpandsSet)
{
  vf_robot_controller::safety::SweptVolumeChecker checker(makeParams());

  auto cells_still = checker.computeSweptCells(
    0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.05, -5.0, -5.0, 8);

  auto cells_moving = checker.computeSweptCells(
    0.0, 0.0, 0.0, 0.3, 0.0, 0.3, 0.05, -5.0, -5.0, 8);

  // Moving forward should cover more cells than stationary
  EXPECT_GT(cells_moving.size(), cells_still.size());
}

TEST(SweptVolumeTest, WorldPointsNonEmpty)
{
  vf_robot_controller::safety::SweptVolumeChecker checker(makeParams());
  auto pts = checker.computeSweptWorldPoints(
    0.0, 0.0, 0.0, 0.2, 0.0, 0.3, 8);
  EXPECT_FALSE(pts.empty());
}

TEST(SweptVolumeTest, RectangleCornersCorrectCount)
{
  // Private helper tested indirectly — verify swept points include 4n corners
  vf_robot_controller::safety::SweptVolumeChecker checker(makeParams());
  const int n_steps = 4;
  auto pts = checker.computeSweptWorldPoints(
    0.0, 0.0, 0.0, 0.1, 0.0, 0.2, n_steps);
  // Each step: 1 centre + 4 corners = 5 points, (n_steps+1) steps
  EXPECT_EQ(static_cast<int>(pts.size()), 5 * (n_steps + 1));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
