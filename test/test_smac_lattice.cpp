// Copyright (c) 2021 Samsung Research America
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

#include "gtest/gtest.h"
#include <ros/ros.h>
#include "costmap_2d/costmap_2d.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/smac_planner_lattice.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {ros::init(0, nullptr);}
  ~RclCppFixture() {ros::shutdown();}
};
RclCppFixture g_rclcppfixture;

// Simple wrapper to be able to call a private member
class LatticeWrap : public nav2_smac_planner::SmacPlannerLattice
{
public:
  void callDynamicParams(std::vector<ros::Parameter> parameters)
  {
    dynamicParametersCallback(parameters);
  }
};

// SMAC smoke tests for plugin-level issues rather than algorithms
// (covered by more extensively testing in other files)
// System tests in nav2_system_tests will actually plan with this work

TEST(SmacTest, test_smac_lattice)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr nodeLattice =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("SmacLatticeTest");
  nodeLattice->declare_parameter("test.debug_visualizations", ros::ParameterValue(true));

  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  auto dummy_cancel_checker = []() {
      return false;
    };

  geometry_msgs::PoseStamped start, goal;
  start.pose.position.x = 0.0;
  start.pose.position.y = 0.0;
  start.pose.orientation.w = 1.0;
  goal.pose.position.x = 1.0;
  goal.pose.position.y = 1.0;
  goal.pose.orientation.w = 1.0;
  auto planner = std::make_unique<nav2_smac_planner::SmacPlannerLattice>();
  try {
    // Expect to throw due to invalid prims file in param
    planner->configure(nodeLattice, "test", nullptr, costmap_ros);
  } catch (...) {
  }
  planner->activate();

  try {
    planner->createPlan(start, goal, dummy_cancel_checker);
  } catch (...) {
  }

  planner->deactivate();
  planner->cleanup();

  planner.reset();
  costmap_ros->on_cleanup(rclcpp_lifecycle::State());
  costmap_ros.reset();
  nodeLattice.reset();
}

TEST(SmacTest, test_smac_lattice_reconfigure)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr nodeLattice =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("SmacLatticeTest");

  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  auto planner = std::make_unique<LatticeWrap>();
  try {
    // Expect to throw due to invalid prims file in param
    planner->configure(nodeLattice, "test", nullptr, costmap_ros);
  } catch (...) {
  }
  planner->activate();

  auto rec_param = std::make_shared<ros::AsyncParametersClient>(
    nodeLattice->get_node_base_interface(), nodeLattice->get_node_topics_interface(),
    nodeLattice->get_node_graph_interface(),
    nodeLattice->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {ros::Parameter("test.allow_unknown", false),
      ros::Parameter("test.max_iterations", -1),
      ros::Parameter("test.cache_obstacle_heuristic", true),
      ros::Parameter("test.reverse_penalty", 5.0),
      ros::Parameter("test.change_penalty", 1.0),
      ros::Parameter("test.non_straight_penalty", 2.0),
      ros::Parameter("test.cost_penalty", 2.0),
      ros::Parameter("test.retrospective_penalty", 0.2),
      ros::Parameter("test.analytic_expansion_ratio", 4.0),
      ros::Parameter("test.max_planning_time", 10.0),
      ros::Parameter("test.lookup_table_size", 30.0),
      ros::Parameter("test.smooth_path", false),
      ros::Parameter("test.analytic_expansion_max_length", 42.0),
      ros::Parameter("test.tolerance", 42.0),
      ros::Parameter("test.rotation_penalty", 42.0),
      ros::Parameter("test.max_on_approach_iterations", 42),
      ros::Parameter("test.terminal_checking_interval", 42),
      ros::Parameter("test.allow_reverse_expansion", true)});

  try {
    // All of these params will re-init A* which will involve loading the control set file
    // which will cause an exception because the file does not exist. This will cause an
    // expected failure preventing parameter updates from being successfully processed
    ros::spin_until_future_complete(
      nodeLattice->get_node_base_interface(),
      results);
  } catch (...) {
  }

  // So instead, lets call manually on a change
  std::vector<ros::Parameter> parameters;
  parameters.push_back(ros::Parameter("test.lattice_filepath", std::string("HI")));
  EXPECT_THROW(planner->callDynamicParams(parameters), std::runtime_error);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
