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

#include "nav2_smac_planner/utils.hpp"

#include "nav2_smac_planner/collision_checker.hpp"

namespace nav2_smac_planner
{

GridCollisionChecker::GridCollisionChecker(
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros,
  unsigned int num_quantizations)
{
  if (costmap_ros) {
    costmap_ros_ = costmap_ros;
    costmap_ = std::shared_ptr<costmap_2d::Costmap2D>(costmap_ros_->getCostmap());
    world_model_ = std::make_unique<base_local_planner::CostmapModel>(*costmap_);
    setFootprint(
        costmap_ros_->getRobotFootprint(),
        costmap_ros_->getUseRadius(),
        Utils::findCircumscribedCost(costmap_ros_.get()));
  }

  // Convert number of regular bins into angles
  float bin_size = 2 * M_PI / static_cast<float>(num_quantizations);
  angles_.clear();
  angles_.reserve(num_quantizations);
  for (unsigned int i = 0; i != num_quantizations; i++) {
    angles_.push_back(bin_size * i);
  }
}

void GridCollisionChecker::setCostmap(costmap_2d::Costmap2D* costmap)
{
  costmap_ = std::shared_ptr<costmap_2d::Costmap2D>(costmap);
  world_model_ = std::make_unique<base_local_planner::CostmapModel>(*costmap_);
}

void GridCollisionChecker::setFootprint(
    const Footprint & footprint,
    const bool & radius,
    const double & possible_collision_cost)
{
  possible_collision_cost_ = static_cast<float>(possible_collision_cost);
  unoriented_footprint_ = footprint;
  footprint_is_radius_ = radius;

  // ROS 2 version precomputes here rotated footprints for all the orientation bins, I suppose to speedup checking
  // the cost. We cannot do the same, as the WorldModel::footprintCost method that we use rotates the footprint itself.
  // TODO: It would be interesting to check how much we can save for complex footprints, though
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
  double wx, wy;
  costmap_->mapToWorld(static_cast<double>(x), static_cast<double>(y), wx, wy);

  if (!footprint_is_radius_) {
    // if footprint, then we check for the footprint's points, but first see
    // if the robot is even potentially in an inscribed collision
    footprint_cost_ = static_cast<float>(costmap_->getCost(
        static_cast<unsigned int>(x + 0.5f), static_cast<unsigned int>(y + 0.5f)));

    if (footprint_cost_ < possible_collision_cost_) {
      if (possible_collision_cost_ > 0.0f) {
        return false;
      } else {
        ROS_ERROR_THROTTLE(1000,
          "Inflation layer either not found or inflation is not set sufficiently for "
          "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
          " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
          "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
          " for full instructions. This will substantially impact run-time performance.");
      }
    }

    // If it's inscribed, in collision, or unknown in the middle,
    // no need to even check the footprint, its invalid
    if (footprint_cost_ == UNKNOWN && !traverse_unknown) {
      return true;
    }

    if (footprint_cost_ == INSCRIBED || footprint_cost_ == OCCUPIED) {
      return true;
    }

    // if possible inscribed, need to check actual footprint pose
    float theta = angles_[angle_bin];
    double cost = world_model_->footprintCost(wx, wy, theta, unoriented_footprint_);
    if (cost <= -2.0)
      footprint_cost_ = UNKNOWN; // also for outside (-3), but we have no constant
    else if (cost == -1.0)
      footprint_cost_ = OCCUPIED;
    else
      footprint_cost_ = static_cast<float>(cost);

    if (footprint_cost_ == UNKNOWN && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return footprint_cost_ >= OCCUPIED;
  } else {
    // if radius, then we can check the center of the cost assuming inflation is used
    footprint_cost_ = static_cast<float>(costmap_->getCost(
        static_cast<unsigned int>(x + 0.5f), static_cast<unsigned int>(y + 0.5f)));

    if (footprint_cost_ == UNKNOWN && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return footprint_cost_ >= INSCRIBED;
  }
}

bool GridCollisionChecker::inCollision(
  const unsigned int & i,
  const bool & traverse_unknown)
{
  footprint_cost_ = costmap_->getCost(i);
  if (footprint_cost_ == UNKNOWN && traverse_unknown) {
    return false;
  }

  // if occupied or unknown and not to traverse unknown space
  return footprint_cost_ >= INSCRIBED;
}

float GridCollisionChecker::getCost()
{
  // Assumes inCollision called prior
  return static_cast<float>(footprint_cost_);
}

bool GridCollisionChecker::outsideRange(const unsigned int & max, const float & value)
{
  return value < 0.0f || value > max;
}

}  // namespace nav2_smac_planner
