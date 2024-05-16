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

#ifndef SMAC_PLANNER__SMOOTHER_HPP_
#define SMAC_PLANNER__SMOOTHER_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>

#include "costmap_2d/costmap_2d.h"
#include <dynamic_reconfigure/server.h>
#include "smac_planner/types.hpp"
#include "smac_planner/constants.hpp"
#include "smac_planner/SmootherConfig.h"
#include "nav_msgs/Path.h"
#include "angles/angles.h"
#include "tf2/utils.h"
#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/DubinsStateSpace.h"

namespace smac_planner
{

/**
 * @class smac_planner::PathSegment
 * @brief A segment of a path in start/end indices
 */
struct PathSegment
{
  unsigned int start;
  unsigned int end;
};

/**
 * @struct smac_planner::BoundaryPoints
 * @brief Set of boundary condition points from expansion
 */
struct BoundaryPoints
{
  /**
   * @brief A constructor for BoundaryPoints
   */
  BoundaryPoints(double & x_in, double & y_in, double & theta_in)
  : x(x_in), y(y_in), theta(theta_in)
  {}

  double x;
  double y;
  double theta;
};

/**
 * @struct smac_planner::BoundaryExpansion
 * @brief Boundary expansion state
 */
struct BoundaryExpansion
{
  double path_end_idx{0.0};
  double expansion_path_length{0.0};
  double original_path_length{0.0};
  std::vector<BoundaryPoints> pts;
  bool in_collision{false};
};

typedef std::vector<BoundaryExpansion> BoundaryExpansions;
typedef std::vector<geometry_msgs::PoseStamped>::iterator PathIterator;
typedef std::vector<geometry_msgs::PoseStamped>::reverse_iterator ReversePathIterator;

/**
 * @class smac_planner::Smoother
 * @brief A path smoother implementation
 */
class Smoother
{
public:
  /**
   * @brief A constructor for smac_planner::Smoother
   */
  explicit Smoother();

  /**
   * @brief A destructor for smac_planner::Smoother
   */
  ~Smoother() {}

  /**
   * @brief Initialization of the smoother
   * @param parent_nh Parent planner private namespace
   */
  void initialize(ros::NodeHandle& parent_nh);

  void setMinTurningRadius(const double & min_turning_radius);

  void reconfigureCB(SmootherConfig& config, uint32_t level);

  /**
   * @brief Smoother API method
   * @param path Reference to path
   * @param costmap Pointer to minimal costmap
   * @param max_time Maximum time to compute, stop early if over limit
   * @return If smoothing was successful
   */
  bool smooth(
    nav_msgs::Path & path,
    const costmap_2d::Costmap2D * costmap,
    const double & max_time);

protected:
  /**
   * @brief Smoother method - does the smoothing on a segment
   * @param path Reference to path
   * @param reversing_segment Return if this is a reversing segment
   * @param costmap Pointer to minimal costmap
   * @param max_time Maximum time to compute, stop early if over limit
   * @return If smoothing was successful
   */
  bool smoothImpl(
    nav_msgs::Path & path,
    bool & reversing_segment,
    const costmap_2d::Costmap2D * costmap,
    const double & max_time);

  /**
   * @brief Get the field value for a given dimension
   * @param msg Current pose to sample
   * @param dim Dimension ID of interest
   * @return dim value
   */
  inline double getFieldByDim(
    const geometry_msgs::PoseStamped & msg,
    const unsigned int & dim);

  /**
   * @brief Set the field value for a given dimension
   * @param msg Current pose to sample
   * @param dim Dimension ID of interest
   * @param value to set the dimention to for the pose
   */
  inline void setFieldByDim(
    geometry_msgs::PoseStamped & msg, const unsigned int dim,
    const double & value);

  /**
   * @brief Finds the starting and end indices of path segments where
   * the robot is traveling in the same direction (e.g. forward vs reverse)
   * @param path Path in which to look for cusps
   * @return Set of index pairs for each segment of the path in a given direction
   */
  std::vector<PathSegment> findDirectionalPathSegments(const nav_msgs::Path & path);

  /**
   * @brief Enforced minimum curvature boundary conditions on plan output
   * the robot is traveling in the same direction (e.g. forward vs reverse)
   * @param start_pose Start pose of the feasible path to maintain
   * @param path Path to modify for curvature constraints on start / end of path
   * @param costmap Costmap to check for collisions
   * @param reversing_segment Whether this path segment is in reverse
   */
  void enforceStartBoundaryConditions(
    const geometry_msgs::Pose & start_pose,
    nav_msgs::Path & path,
    const costmap_2d::Costmap2D * costmap,
    const bool & reversing_segment);

  /**
   * @brief Enforced minimum curvature boundary conditions on plan output
   * the robot is traveling in the same direction (e.g. forward vs reverse)
   * @param end_pose End pose of the feasible path to maintain
   * @param path Path to modify for curvature constraints on start / end of path
   * @param costmap Costmap to check for collisions
   * @param reversing_segment Whether this path segment is in reverse
   */
  void enforceEndBoundaryConditions(
    const geometry_msgs::Pose & end_pose,
    nav_msgs::Path & path,
    const costmap_2d::Costmap2D * costmap,
    const bool & reversing_segment);

  /**
   * @brief Given a set of boundary expansion, find the one which is shortest
   * such that it is least likely to contain a loop-de-loop when working with
   * close-by primitive markers. Instead, select a further away marker which
   * generates a shorter `
   * @param boundary_expansions Set of boundary expansions
   * @return Idx of the shorest boundary expansion option
   */
  unsigned int findShortestBoundaryExpansionIdx(const BoundaryExpansions & boundary_expansions);

  /**
   * @brief Populate a motion model expansion from start->end into expansion
   * @param start Start pose of the feasible path to maintain
   * @param end End pose of the feasible path to maintain
   * @param expansion Expansion object to populate
   * @param costmap Costmap to check for collisions
   * @param reversing_segment Whether this path segment is in reverse
   */
  void findBoundaryExpansion(
    const geometry_msgs::Pose & start,
    const geometry_msgs::Pose & end,
    BoundaryExpansion & expansion,
    const costmap_2d::Costmap2D * costmap);

  /**
   * @brief Generates boundary expansions with end idx at least strategic
   * distances away, using either Reverse or (Forward) Path Iterators.
   * @param start iterator to start search in path for
   * @param end iterator to end search for
   * @return Boundary expansions with end idxs populated
   */
  template<typename IteratorT>
  BoundaryExpansions generateBoundaryExpansionPoints(IteratorT start, IteratorT end);

  /**
   * @brief For a given path, update the path point orientations based on smoothing
   * @param path Path to approximate the path orientation in
   * @param reversing_segment Return if this is a reversing segment
   */
  inline void updateApproximatePathOrientations(
    nav_msgs::Path & path,
    bool & reversing_segment);

  std::unique_ptr<dynamic_reconfigure::Server<SmootherConfig>> dsrv_;
  double min_turning_rad_, tolerance_, data_w_, smooth_w_;
  int max_its_, refinement_ctr_, refinement_num_;
  bool is_holonomic_, do_refinement_;
  ompl::base::StateSpacePtr state_space_;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__SMOOTHER_HPP_
