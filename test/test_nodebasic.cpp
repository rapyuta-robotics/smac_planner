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

#include "gtest/gtest.h"
#include <ros/ros.h>
#include "costmap_2d/costmap_2d.h"
#include "smac_planner/node_basic.hpp"
#include "smac_planner/node_2d.hpp"
#include "smac_planner/node_hybrid.hpp"
#include "smac_planner/node_lattice.hpp"
#include "smac_planner/collision_checker.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {ros::init(0, nullptr);}
  ~RclCppFixture() {ros::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(NodeBasicTest, test_node_basic)
{
  smac_planner::NodeBasic<smac_planner::NodeHybrid> node(50);

  EXPECT_EQ(node.index, 50u);
  EXPECT_EQ(node.graph_node_ptr, nullptr);

  smac_planner::NodeBasic<smac_planner::Node2D> node2(100);

  EXPECT_EQ(node2.index, 100u);
  EXPECT_EQ(node2.graph_node_ptr, nullptr);

  smac_planner::NodeBasic<smac_planner::NodeLattice> node3(200);

  EXPECT_EQ(node3.index, 200u);
  EXPECT_EQ(node3.graph_node_ptr, nullptr);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
