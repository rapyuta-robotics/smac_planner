// Copyright (c) 2021, Samsung Research America
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

#include "smac_planner/utils.hpp"

#include "smac_planner/collision_checker.hpp"

namespace smac_planner
{

GridCollisionChecker::GridCollisionChecker(
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros,
  unsigned int num_quantizations)
{
  // Convert number of regular bins into angles
  float bin_size = 2 * M_PI / static_cast<float>(num_quantizations);
  angles_.clear();
  angles_.reserve(num_quantizations);
  for (unsigned int i = 0; i != num_quantizations; i++) {
    angles_.push_back(bin_size * i);
  }

  if (costmap_ros) {
    costmap_ros_ = costmap_ros;
    setFootprint(
        costmap_ros_->getRobotFootprint(),
        costmap_ros_->getUseRadius(),
        Utils::findCircumscribedCost(costmap_ros_.get()));
  }
}

void GridCollisionChecker::setCostmap(costmap_2d::Costmap2D* costmap)
{
  costmap_ = costmap;
  world_model_ = std::make_unique<base_local_planner::CostmapModel>(*costmap_);
}

void GridCollisionChecker::setFootprint(
    const Footprint & footprint,
    const bool & radius,
    const double & possible_collision_cost)
{
  possible_collision_cost_ = static_cast<float>(possible_collision_cost);
  footprint_is_radius_ = radius;

  // Use radius, no caching required
  if (radius) {
    return;
  }

  // No change, no updates required
  if (footprint == unoriented_footprint_) {
    return;
  }

  oriented_footprints_.clear();
  oriented_footprints_.reserve(angles_.size());
  double sin_th, cos_th;
  geometry_msgs::Point new_pt;
  const unsigned int footprint_size = footprint.size();

  // Precompute the orientation bins for checking to use
  for (unsigned int i = 0; i != angles_.size(); i++) {
    sin_th = sin(angles_[i]);
    cos_th = cos(angles_[i]);
    Footprint oriented_footprint;
    oriented_footprint.reserve(footprint_size);

    for (unsigned int j = 0; j < footprint_size; j++) {
      new_pt.x = footprint[j].x * cos_th - footprint[j].y * sin_th;
      new_pt.y = footprint[j].x * sin_th + footprint[j].y * cos_th;
      oriented_footprint.push_back(new_pt);
    }

    oriented_footprints_.push_back(oriented_footprint);
  }

  unoriented_footprint_ = footprint;
  costmap_2d::calculateMinAndMaxDistances(footprint, inscribed_radius_, circumscribed_radius_);
}

bool GridCollisionChecker::inCollision(
  const float & x,
  const float & y,
  const float & angle_bin,
  const bool & traverse_unknown)
{
  // Check to make sure cell is inside the map
  if (outsideRange(costmap_->getSizeInCellsX(), x) ||
    outsideRange(costmap_->getSizeInCellsY(), y))
  {
    return false;
  }

  // Assumes setFootprint already set
  center_cost_ = static_cast<float>(costmap_->getCost(
      static_cast<unsigned int>(x + 0.5f), static_cast<unsigned int>(y + 0.5f)));
  if (!footprint_is_radius_) {
    // if footprint, then we check for the footprint's points, but first see
    // if the robot is even potentially in an inscribed collision
    if (center_cost_ < possible_collision_cost_) {
      if (possible_collision_cost_ > 0.0f) {
        return false;
      } else {
        ROS_ERROR_THROTTLE(1000,
          "Inflation layer either not found or inflation is not set sufficiently for "
          "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
          " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
          "github.com/ros-planning/navigation2/tree/main/smac_planner#potential-fields"
          " for full instructions. This will substantially impact run-time performance.");
      }
    }

    // If it's inscribed, in collision, or unknown in the middle,
    // no need to even check the footprint, its invalid
    if (center_cost_ == UNKNOWN && !traverse_unknown) {
      return true;
    }

    if (center_cost_ == INSCRIBED || center_cost_ == OCCUPIED) {
      return true;
    }

    geometry_msgs::Point robot_position;
    costmap_->mapToWorld(static_cast<unsigned int>(x), static_cast<unsigned int>(y),
                         robot_position.x, robot_position.y);

    // if possible inscribed, need to check actual footprint pose.
    // Use precomputed oriented footprints are done on initialization,
    // offset by translation value to collision check
    Footprint current_footprint = oriented_footprints_[angle_bin];
    for (auto& pt: current_footprint) {
      pt.x += robot_position.x;
      pt.y += robot_position.y;
    }
    float footprint_cost = world_model_->footprintCost(robot_position, current_footprint, inscribed_radius_, circumscribed_radius_);
    if (footprint_cost <= -2.0)
      footprint_cost = UNKNOWN; // also for outside (-3), but we have no constant
    else if (footprint_cost == -1.0)
      footprint_cost = OCCUPIED;

    if (footprint_cost == UNKNOWN && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return footprint_cost >= OCCUPIED;
  } else {
    // if radius, then we can check the center of the cost assuming inflation is used
    if (center_cost_ == UNKNOWN && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return center_cost_ >= INSCRIBED;
  }
}

bool GridCollisionChecker::inCollision(
  const unsigned int & i,
  const bool & traverse_unknown)
{
  center_cost_ = costmap_->getCost(i);
  if (center_cost_ == UNKNOWN && traverse_unknown) {
    return false;
  }

  // if occupied or unknown and not to traverse unknown space
  return center_cost_ >= INSCRIBED;
}

float GridCollisionChecker::getCost()
{
  // Assumes inCollision called prior
  return static_cast<float>(center_cost_);
}

bool GridCollisionChecker::outsideRange(const unsigned int & max, const float & value)
{
  return value < 0.0f || value > max;
}

}  // namespace smac_planner
