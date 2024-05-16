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

#include "geometry_msgs/PoseStamped.h"
#include "gtest/gtest.h"
#include "costmap_2d/costmap_2d.h"
#include "smac_planner/a_star.hpp"
#include "smac_planner/collision_checker.hpp"
#include "smac_planner/node_hybrid.hpp"
#include "smac_planner/smac_planner_2d.hpp"
#include "smac_planner/smac_planner_hybrid.hpp"
#include <ros/ros.h>

class RclCppFixture
{
public:
  RclCppFixture() {ros::init(0, nullptr);}
  ~RclCppFixture() {ros::shutdown();}
};
RclCppFixture g_rclcppfixture;

// SMAC smoke tests for plugin-level issues rather than algorithms
// (covered by more extensively testing in other files)
// System tests in nav2_system_tests will actually plan with this work

TEST(SmacTest, test_smac_2d) {
  rclcpp_lifecycle::LifecycleNode::SharedPtr node2D =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("Smac2DTest");

  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  node2D->declare_parameter("test.smooth_path", true);
  node2D->set_parameter(ros::Parameter("test.smooth_path", true));
  node2D->declare_parameter("test.downsample_costmap", true);
  node2D->set_parameter(ros::Parameter("test.downsample_costmap", true));
  node2D->declare_parameter("test.downsampling_factor", 2);
  node2D->set_parameter(ros::Parameter("test.downsampling_factor", 2));

  auto dummy_cancel_checker = []() {
      return false;
    };

  geometry_msgs::PoseStamped start, goal;
  start.pose.position.x = 0.0;
  start.pose.position.y = 0.0;
  start.pose.orientation.w = 1.0;
  // goal = start;
  goal.pose.position.x = 7.0;
  goal.pose.position.y = 0.0;
  goal.pose.orientation.w = 1.0;
  auto planner_2d = std::make_unique<smac_planner::SmacPlanner2D>();
  planner_2d->configure(node2D, "test", nullptr, costmap_ros);
  planner_2d->activate();
  try {
    planner_2d->createPlan(start, goal, dummy_cancel_checker);
  } catch (...) {
  }

  planner_2d->deactivate();
  planner_2d->cleanup();

  planner_2d.reset();
  costmap_ros->on_cleanup(rclcpp_lifecycle::State());
  node2D.reset();
  costmap_ros.reset();
}

TEST(SmacTest, test_smac_2d_reconfigure) {
  rclcpp_lifecycle::LifecycleNode::SharedPtr node2D =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("Smac2DTest");

  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  auto planner_2d = std::make_unique<smac_planner::SmacPlanner2D>();
  planner_2d->configure(node2D, "test", nullptr, costmap_ros);
  planner_2d->activate();

  auto rec_param = std::make_shared<ros::AsyncParametersClient>(
    node2D->get_node_base_interface(), node2D->get_node_topics_interface(),
    node2D->get_node_graph_interface(),
    node2D->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {ros::Parameter("test.tolerance", 1.0),
      ros::Parameter("test.cost_travel_multiplier", 1.0),
      ros::Parameter("test.max_planning_time", 2.0),
      ros::Parameter("test.downsample_costmap", false),
      ros::Parameter("test.allow_unknown", false),
      ros::Parameter("test.downsampling_factor", 2),
      ros::Parameter("test.max_iterations", -1),
      ros::Parameter("test.max_on_approach_iterations", -1),
      ros::Parameter("test.terminal_checking_interval", 100),
      ros::Parameter("test.use_final_approach_orientation", false)});

  ros::spin_until_future_complete(
    node2D->get_node_base_interface(),
    results);

  EXPECT_EQ(node2D->get_parameter("test.tolerance").as_double(), 1.0);
  EXPECT_EQ(
    node2D->get_parameter("test.cost_travel_multiplier").as_double(),
    1.0);
  EXPECT_EQ(node2D->get_parameter("test.max_planning_time").as_double(), 2.0);
  EXPECT_EQ(node2D->get_parameter("test.downsample_costmap").as_bool(), false);
  EXPECT_EQ(node2D->get_parameter("test.allow_unknown").as_bool(), false);
  EXPECT_EQ(node2D->get_parameter("test.downsampling_factor").as_int(), 2);
  EXPECT_EQ(node2D->get_parameter("test.max_iterations").as_int(), -1);
  EXPECT_EQ(node2D->get_parameter("test.use_final_approach_orientation").as_bool(), false);
  EXPECT_EQ(
    node2D->get_parameter("test.max_on_approach_iterations").as_int(),
    -1);
  EXPECT_EQ(
    node2D->get_parameter("test.terminal_checking_interval").as_int(),
    100);

  results = rec_param->set_parameters_atomically(
    {ros::Parameter("test.downsample_costmap", true)});

  ros::spin_until_future_complete(
    node2D->get_node_base_interface(),
    results);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
