// Copyright (c) 2023 Open Navigation LLC
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
// limitations under the License.

#include <math.h>
#include <memory>
#include <string>
#include <vector>

#include <tf/tf.h>

#include "gtest/gtest.h"
#include "smac_planner/utils.hpp"

using namespace smac_planner;  // NOLINT

TEST(transform_footprint_to_edges, test_basic)
{
  geometry_msgs::Point p1;
  p1.x = 1.0;
  p1.y = 1.0;

  geometry_msgs::Point p2;
  p2.x = 1.0;
  p2.y = -1.0;

  geometry_msgs::Point p3;
  p3.x = -1.0;
  p3.y = -1.0;

  geometry_msgs::Point p4;
  p4.x = -1.0;
  p4.y = 1.0;

  std::vector<geometry_msgs::Point> footprint{p1, p2, p3, p4};
  std::vector<geometry_msgs::Point> footprint_edges{p1, p2, p2, p3, p3, p4, p4, p1};

  // Identity pose
  geometry_msgs::Pose pose0;
  auto result = transformFootprintToEdges(pose0, footprint);
  EXPECT_EQ(result.size(), 8u);

  for (size_t i = 0; i < result.size(); i++) {
    auto & p = result[i];
    auto & q = footprint_edges[i];
    EXPECT_EQ(p.x, q.x);
    EXPECT_EQ(p.y, q.y);
  }
}

TEST(transform_footprint_to_edges, test_transition_rotation)
{
  geometry_msgs::Point p1;
  p1.x = 1.0;
  p1.y = 1.0;

  geometry_msgs::Point p2;
  p2.x = 1.0;
  p2.y = -1.0;

  geometry_msgs::Point p3;
  p3.x = -1.0;
  p3.y = -1.0;

  geometry_msgs::Point p4;
  p4.x = -1.0;
  p4.y = 1.0;

  geometry_msgs::Pose pose0;
  pose0.position.x = 1.0;
  pose0.position.y = 1.0;
  pose0.orientation = tf::createQuaternionMsgFromYaw(M_PI / 4.0);

  std::vector<geometry_msgs::Point> footprint{p1, p2, p3, p4};

  // q1
  geometry_msgs::Point q1;
  q1.x = 0.0 + pose0.position.x;
  q1.y = sqrt(2) + pose0.position.y;

  // q2
  geometry_msgs::Point q2;
  q2.x = sqrt(2.0) + pose0.position.x;
  q2.y = 0.0 + pose0.position.y;

  // q3
  geometry_msgs::Point q3;
  q3.x = 0.0 + pose0.position.x;
  q3.y = -sqrt(2) + pose0.position.y;

  // q4
  geometry_msgs::Point q4;
  q4.x = -sqrt(2.0) + pose0.position.x;
  q4.y = 0.0 + pose0.position.y;

  std::vector<geometry_msgs::Point> footprint_edges{q1, q2, q2, q3, q3, q4, q4, q1};
  auto result = transformFootprintToEdges(pose0, footprint);
  EXPECT_EQ(result.size(), 8u);

  for (size_t i = 0; i < result.size(); i++) {
    auto & p = result[i];
    auto & q = footprint_edges[i];
    EXPECT_NEAR(p.x, q.x, 1e-3);
    EXPECT_NEAR(p.y, q.y, 1e-3);
  }
}

TEST(create_marker, test_createMarker)
{
  geometry_msgs::Point p1;
  p1.x = 1.0;
  p1.y = 1.0;

  geometry_msgs::Point p2;
  p2.x = 1.0;
  p2.y = -1.0;

  geometry_msgs::Point p3;
  p3.x = -1.0;
  p3.y = -1.0;

  geometry_msgs::Point p4;
  p4.x = -1.0;
  p4.y = 1.0;
  std::vector<geometry_msgs::Point> edges{p1, p2, p3, p4};

  auto marker1 = createMarker(edges, 10u, "test_frame", ros::Time(0.));
  EXPECT_EQ(marker1.header.frame_id, "test_frame");
  EXPECT_EQ(marker1.header.stamp.toNSec(), 0);
  EXPECT_EQ(marker1.ns, "planned_footprint");
  EXPECT_EQ(marker1.id, 10u);
  EXPECT_EQ(marker1.points.size(), 4u);

  edges.clear();
  auto marker2 = createMarker(edges, 8u, "test_frame2", ros::Time(1.0, 0.0));
  EXPECT_EQ(marker2.header.frame_id, "test_frame2");
  EXPECT_EQ(marker2.header.stamp.toNSec(), 1e9);
  EXPECT_EQ(marker2.id, 8u);
  EXPECT_EQ(marker2.points.size(), 0u);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
