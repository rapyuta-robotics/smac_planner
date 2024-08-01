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

#include <vector>
#include <memory>

#include "base_local_planner/costmap_model.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "smac_planner/constants.hpp"

#ifndef SMAC_PLANNER__COLLISION_CHECKER_HPP_
#define SMAC_PLANNER__COLLISION_CHECKER_HPP_

namespace smac_planner
{

/**
 * @class smac_planner::GridCollisionChecker
 * @brief A costmap grid collision checker
 */
class GridCollisionChecker
{
public:
  typedef std::vector<geometry_msgs::Point> Footprint;

  /**
   * @brief A constructor for smac_planner::GridCollisionChecker
   * for use when regular bin intervals are appropriate
   * @param costmap The costmap to collision check against
   * @param num_quantizations The number of quantizations to precompute footprint
   * orientations for to speed up collision checking
   */
  GridCollisionChecker(
    std::shared_ptr<costmap_2d::Costmap2DROS> costmap,
    unsigned int num_quantizations);

  /**
   * @brief Get the current costmap object
   */
  costmap_2d::Costmap2D* getCostmap()
  {
    return costmap_.get();
  }

  /**
   * @brief Set the current costmap object to use for collision detection
   */
  void setCostmap(costmap_2d::Costmap2D* costmap);

  /**
   * @brief Set the footprint to use with collision checker, replacing costmap one.
   * @param footprint The footprint to collision check against
   * @param radius Whether or not the footprint is a circle and use radius collision checking
   */
  void setFootprint(
    const Footprint & footprint,
    const bool & radius,
    const double & possible_collision_cost);

  /**
   * @brief Check if in collision with costmap and footprint at pose
   * @param x X coordinate of pose to check against
   * @param y Y coordinate of pose to check against
   * @param theta Angle bin number of pose to check against (NOT radians)
   * @param traverse_unknown Whether or not to traverse in unknown space
   * @return boolean if in collision or not.
   */
  bool inCollision(
    const float & x,
    const float & y,
    const float & theta,
    const bool & traverse_unknown);

  /**
   * @brief Check if in collision with costmap and footprint at pose
   * @param i Index to search collision status of
   * @param traverse_unknown Whether or not to traverse in unknown space
   * @return boolean if in collision or not.
   */
  bool inCollision(
    const unsigned int & i,
    const bool & traverse_unknown);

  /**
   * @brief Get cost at footprint pose in costmap
   * @return the cost at the pose in costmap
   */
  float getCost();

  /**
   * @brief Get the angles of the precomputed footprint orientations
   * @return the ordered vector of angles corresponding to footprints
   */
  std::vector<float> & getPrecomputedAngles()
  {
    return angles_;
  }

  /**
   * @brief Get costmap ros object for inflation layer params
   * @return Costmap ros
   */
  std::shared_ptr<costmap_2d::Costmap2DROS> getCostmapROS() {return costmap_ros_;}

private:
  /**
   * @brief Check if value outside the range
   * @param min Minimum value of the range
   * @param max Maximum value of the range
   * @param value the value to check if it is within the range
   * @return boolean if in range or not
   */
  bool outsideRange(const unsigned int & max, const float & value);

protected:
  std::unique_ptr<base_local_planner::CostmapModel> world_model_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<costmap_2d::Costmap2D> costmap_;
  std::vector<Footprint> oriented_footprints_;
  Footprint unoriented_footprint_;
  double inscribed_radius_;
  double circumscribed_radius_;
  float footprint_cost_;
  bool footprint_is_radius_;
  std::vector<float> angles_;
  float possible_collision_cost_{-1};
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__COLLISION_CHECKER_HPP_
