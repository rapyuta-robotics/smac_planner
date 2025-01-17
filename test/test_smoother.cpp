// Copyright (c) 2020, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <math.h>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "gtest/gtest.h"
#include <ros/ros.h>
#include "costmap_2d/costmap_2d.h"
#include "smac_planner/node_hybrid.hpp"
#include "smac_planner/a_star.hpp"
#include "smac_planner/collision_checker.hpp"
#include "smac_planner/smoother.hpp"

using namespace smac_planner;  // NOLINT

class RclCppFixture
{
public:
  RclCppFixture() {ros::init(0, nullptr);}
  ~RclCppFixture() {ros::shutdown();}
};
RclCppFixture g_rclcppfixture;

class SmootherWrapper : public smac_planner::Smoother
{
public:
  explicit SmootherWrapper(const SmootherParams & params)
  : smac_planner::Smoother(params)
  {}

  std::vector<PathSegment> findDirectionalPathSegmentsWrapper(nav_msgs::Path path)
  {
    return findDirectionalPathSegments(path);
  }
};

TEST(SmootherTest, test_full_smoother)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("SmacSmootherTest");
  smac_planner::SmootherParams params;
  params.get(node, "test");
  double maxtime = 1.0;

  // Make smoother and costmap to smooth in
  auto smoother = std::make_unique<SmootherWrapper>(params);
  smoother->initialize(0.4 /*turning radius*/);

  costmap_2d::Costmap2D * costmap =
    new costmap_2d::Costmap2D(100, 100, 0.05, 0.0, 0.0, 0);
  // island in the middle of lethal cost to cross
  for (unsigned int i = 20; i <= 30; ++i) {
    for (unsigned int j = 20; j <= 30; ++j) {
      costmap->setCost(i, j, 254);
    }
  }

  // Setup A* search to get path to smooth
  smac_planner::SearchInfo info;
  info.change_penalty = 0.05;
  info.non_straight_penalty = 1.05;
  info.reverse_penalty = 2.0;
  info.cost_penalty = 2.0;
  info.retrospective_penalty = 0.0;
  info.analytic_expansion_ratio = 3.5;
  info.minimum_turning_radius = 8;  // in grid coordinates 0.4/0.05
  info.analytic_expansion_max_length = 20.0;  // in grid coordinates
  unsigned int size_theta = 72;
  smac_planner::AStarAlgorithm<smac_planner::NodeHybrid> a_star(
    smac_planner::MotionModel::REEDS_SHEPP, info);
  int max_iterations = 10000;
  float tolerance = 10.0;
  int it_on_approach = 10;
  int terminal_checking_interval = 5000;
  double max_planning_time = 120.0;
  int num_it = 0;

  a_star.initialize(
    false, max_iterations,
    std::numeric_limits<int>::max(), terminal_checking_interval, max_planning_time, 401,
    size_theta);

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<costmap_2d::Costmap2DROS>("dummy_costmap", g_tf2_buffer);
  auto costmapi = costmap_ros->getCostmap();
  *costmapi = *costmap;

  std::unique_ptr<smac_planner::GridCollisionChecker> checker =
    std::make_unique<smac_planner::GridCollisionChecker>(costmap_ros, size_theta, node);

  auto dummy_cancel_checker = []() {
      return false;
    };

  // Create A* search to smooth
  a_star.setCollisionChecker(checker.get());
  a_star.setStart(5u, 5u, 0u);
  a_star.setGoal(45u, 45u, 36u);
  smac_planner::NodeHybrid::CoordinateVector path;
  EXPECT_TRUE(a_star.createPath(path, num_it, tolerance, dummy_cancel_checker));

  // Convert to world coordinates and get length to compare to smoothed length
  nav_msgs::Path plan;
  plan.header.stamp = node->now();
  plan.header.frame_id = "map";
  geometry_msgs::PoseStamped pose;
  pose.header = plan.header;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  double initial_length = 0.0;
  double x_m = path[path.size() - 1].x, y_m = path[path.size() - 1].y;
  plan.poses.reserve(path.size());
  for (int i = path.size() - 1; i >= 0; --i) {
    pose.pose = smac_planner::getWorldCoords(path[i].x, path[i].y, costmap);
    pose.pose.orientation = smac_planner::getWorldOrientation(path[i].theta);
    plan.poses.push_back(pose);
    initial_length += hypot(path[i].x - x_m, path[i].y - y_m);
    x_m = path[i].x;
    y_m = path[i].y;
  }

  // Check that we accurately detect that this path has a reversing segment
  EXPECT_EQ(smoother->findDirectionalPathSegmentsWrapper(plan).size(), 2u);

  // Test smoother, should succeed with same number of points
  // and shorter overall length, while still being collision free.
  auto path_size_in = plan.poses.size();
  EXPECT_TRUE(smoother->smooth(plan, costmap, maxtime));
  EXPECT_EQ(plan.poses.size(), path_size_in);  // Should have same number of poses
  double length = 0.0;
  x_m = plan.poses[0].pose.position.x;
  y_m = plan.poses[0].pose.position.y;
  for (unsigned int i = 0; i != plan.poses.size(); i++) {
    // Should be collision free
    EXPECT_EQ(costmap->getCost(plan.poses[i].pose.position.x, plan.poses[i].pose.position.y), 0);
    length += hypot(plan.poses[i].pose.position.x - x_m, plan.poses[i].pose.position.y - y_m);
    x_m = plan.poses[i].pose.position.x;
    y_m = plan.poses[i].pose.position.y;
  }
  EXPECT_LT(length, initial_length);  // Should be shorter

  // Try again but with failure modes

  // Failure mode: not enough iterations to complete
  params.max_its_ = 0;
  auto smoother_bypass = std::make_unique<SmootherWrapper>(params);
  EXPECT_FALSE(smoother_bypass->smooth(plan, costmap, maxtime));
  params.max_its_ = 1;
  auto smoother_failure = std::make_unique<SmootherWrapper>(params);
  EXPECT_FALSE(smoother_failure->smooth(plan, costmap, maxtime));

  // Failure mode: Not enough time
  double max_no_time = 0.0;
  EXPECT_FALSE(smoother->smooth(plan, costmap, max_no_time));

  // Failure mode: Path is in collision, do 2x to exercise overlapping point
  // attempts to update orientation should also fail
  pose.pose.position.x = 1.25;
  pose.pose.position.y = 1.25;
  plan.poses.push_back(pose);
  plan.poses.push_back(pose);
  EXPECT_FALSE(smoother->smooth(plan, costmap, maxtime));
  EXPECT_NEAR(plan.poses.end()[-2].pose.orientation.z, 1.0, 1e-3);
  EXPECT_NEAR(plan.poses.end()[-2].pose.orientation.x, 0.0, 1e-3);
  EXPECT_NEAR(plan.poses.end()[-2].pose.orientation.y, 0.0, 1e-3);
  EXPECT_NEAR(plan.poses.end()[-2].pose.orientation.w, 0.0, 1e-3);

  delete costmap;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
