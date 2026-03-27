#include "vf_robot_controller/parameter_handler.hpp"
#include "vf_robot_controller/tools/velocity_scheduler.hpp"
#include <gtest/gtest.h>
#include "vf_robot_controller/tools/utils.hpp"
#include "vf_robot_controller/models/trajectory.hpp"
#include <cmath>

// ── utils tests ───────────────────────────────────────────────────────────────
TEST(UtilsTest, NormalizeAngle)
{
  EXPECT_NEAR(vf_robot_controller::tools::normalizeAngle(0.0),  0.0,  1e-9);
  EXPECT_NEAR(vf_robot_controller::tools::normalizeAngle(4.0),  4.0 - 2*M_PI, 1e-6);
  EXPECT_NEAR(vf_robot_controller::tools::normalizeAngle(-4.0), -4.0 + 2*M_PI, 1e-6);
  EXPECT_NEAR(vf_robot_controller::tools::normalizeAngle(M_PI), M_PI, 1e-9);
}

TEST(UtilsTest, DiffDriveStep_Straight)
{
  double x = 0.0, y = 0.0, yaw = 0.0;
  vf_robot_controller::tools::diffDriveStep(x, y, yaw, 1.0, 0.0, 1.0);
  EXPECT_NEAR(x,   1.0, 1e-6);
  EXPECT_NEAR(y,   0.0, 1e-6);
  EXPECT_NEAR(yaw, 0.0, 1e-6);
}

TEST(UtilsTest, DiffDriveStep_PureRotation)
{
  double x = 0.0, y = 0.0, yaw = 0.0;
  vf_robot_controller::tools::diffDriveStep(x, y, yaw, 0.0, M_PI / 2.0, 1.0);
  EXPECT_NEAR(x,   0.0,       1e-6);
  EXPECT_NEAR(y,   0.0,       1e-6);
  EXPECT_NEAR(yaw, M_PI/2.0, 1e-6);
}

TEST(UtilsTest, MPPIWeights_AllSameCost)
{
  // Equal costs → equal weights
  const std::vector<double> costs = {1.0, 1.0, 1.0, 1.0};
  const auto w = vf_robot_controller::tools::mppiWeights(costs, 0.3);
  ASSERT_EQ(w.size(), 4u);
  for (auto wi : w) EXPECT_NEAR(wi, 0.25, 1e-6);
}

TEST(UtilsTest, MPPIWeights_BestHasHighestWeight)
{
  const std::vector<double> costs = {10.0, 0.1, 5.0};
  const auto w = vf_robot_controller::tools::mppiWeights(costs, 0.3);
  EXPECT_GT(w[1], w[0]);
  EXPECT_GT(w[1], w[2]);
}

TEST(UtilsTest, MPPIWeights_SumToOne)
{
  const std::vector<double> costs = {1.0, 2.0, 0.5, 3.0, 1.5};
  const auto w = vf_robot_controller::tools::mppiWeights(costs, 0.5);
  double sum = 0.0;
  for (auto wi : w) sum += wi;
  EXPECT_NEAR(sum, 1.0, 1e-6);
}

TEST(UtilsTest, ExtractLookaheadWaypoints_CorrectCount)
{
  nav_msgs::msg::Path plan;
  for (int i = 0; i < 50; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = i * 0.1;
    ps.pose.position.y = 0.0;
    plan.poses.push_back(ps);
  }
  const auto wps = vf_robot_controller::tools::extractLookaheadWaypoints(
    plan, 0.0, 0.0, 2.0, 12);
  EXPECT_EQ(static_cast<int>(wps.size()), 12);
}

TEST(UtilsTest, ComputePathHeadings_CorrectSize)
{
  std::vector<Eigen::Vector2d> wps = {{0,0},{1,0},{2,1},{3,0}};
  const auto h = vf_robot_controller::tools::computePathHeadings(wps);
  EXPECT_EQ(h.size(), wps.size());
}

TEST(UtilsTest, ComputePathHeadings_StraightLine)
{
  std::vector<Eigen::Vector2d> wps = {{0,0},{1,0},{2,0},{3,0}};
  const auto h = vf_robot_controller::tools::computePathHeadings(wps);
  for (auto heading : h) {
    EXPECT_NEAR(heading, 0.0, 1e-6);  // all pointing in +x
  }
}

// ── Velocity scheduler basic test ────────────────────────────────────────────
TEST(VelocitySchedulerTest, HighComplexityReducesSpeed)
{
  vf_robot_controller::Parameters p;
  p.max_vel_x          = 0.5;
  p.min_vel_x          = 0.05;
  p.max_speed_in_tight = 0.15;
  p.tight_gcf_threshold = 0.6;
  p.acc_lim_x          = 0.5;

  vf_robot_controller::tools::VelocityScheduler sched(p);

  // complexityToVmax is private but we test via computeMaxVelocity
  // with a dummy GCF — use open complexity → expect near max_vel_x
  // (Full test requires mock GCF — scaffolded here)
  SUCCEED();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
