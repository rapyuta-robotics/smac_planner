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

#ifndef SMAC_PLANNER__TYPES_HPP_
#define SMAC_PLANNER__TYPES_HPP_

#include <vector>
#include <utility>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <optional>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include <mbf_msgs/GetPathResult.h>
namespace smac_planner
{

typedef std::pair<float, unsigned int> NodeHeuristicPair;

struct Rectangle {
  Rectangle() = default;
  Rectangle(geometry_msgs::Point diagonal_corner_1, geometry_msgs::Point diagonal_corner_2);

  bool pointInside(const geometry_msgs::Point& point) const;

  private:
    geometry_msgs::Point diagonal_corner_1;
    geometry_msgs::Point diagonal_corner_2;
    double _max_x, _max_y, _min_x, _min_y;
};

/**
 * @struct smac_planner::SearchInfo
 * @brief Search properties and penalties
 */
struct SearchInfo
{
  float minimum_turning_radius{8.0};
  float non_straight_penalty{1.05};
  float change_penalty{0.0};
  float reverse_penalty{2.0};
  float cost_penalty{2.0};
  float retrospective_penalty{0.015};
  float rotation_penalty{5.0};
  float analytic_expansion_ratio{3.5};
  float analytic_expansion_max_length{60.0};
  float analytic_expansion_max_cost{200.0};
  float goal_align_distance{0.0};
  bool analytic_expansion_max_cost_override{false};
  std::string lattice_filepath;
  bool cache_obstacle_heuristic{false};
  bool allow_reverse_expansion{false};
  bool allow_primitive_interpolation{false};
  bool allow_goal_overshoot{true};
  bool downsample_obstacle_heuristic{true};
  bool use_quadratic_cost_penalty{false};
  void setStart(const geometry_msgs::Point& start);
  geometry_msgs::Pose getSearchBound();
  void setSearchBound(const geometry_msgs::Pose& search_bound);
  void setSearchSpace(const Rectangle& space);
  std::optional<Rectangle> getSearchSpace();
  bool isSearchSpaceSet();
  void removeSearchSpace();
  bool isStartBehindSearchBounds();

private:
  // smac_planner::Rectangle search_space;
  geometry_msgs::Point _start_pose;
  geometry_msgs::Pose _search_bound;
  std::optional<bool> is_start_behind_goal;
  std::optional<Rectangle> _search_space;
};

struct PlanResult {
  uint32_t result_code = mbf_msgs::GetPathResult::SUCCESS;
  double cost = 0;
  std::string message = "";

  PlanResult() = default;
  PlanResult(uint32_t result_code, double cost, const std::string& message);

  const std::vector<geometry_msgs::PoseStamped>& path() const;
  void setPath(const std::vector<geometry_msgs::PoseStamped>& new_path);
  double length() const;
  bool isValid() const;
  PlanResult operator+(const PlanResult& other_result) const;

private:
  double _length = 0;
  std::vector<geometry_msgs::PoseStamped> _path{};
};

/**
 * @struct smac_planner::TurnDirection
 * @brief A struct with the motion primitive's direction embedded
 */
enum struct TurnDirection
{
  UNKNOWN = 0,
  FORWARD = 1,
  LEFT = 2,
  RIGHT = 3,
  REVERSE = 4,
  REV_LEFT = 5,
  REV_RIGHT = 6
};

/**
 * @struct smac_planner::MotionPose
 * @brief A struct for poses in motion primitives
 */
struct MotionPose
{
  /**
   * @brief A constructor for smac_planner::MotionPose
   */
  MotionPose() {}

  /**
   * @brief A constructor for smac_planner::MotionPose
   * @param x X pose
   * @param y Y pose
   * @param theta Angle of pose
   * @param TurnDirection Direction of the primitive's turn
   */
  MotionPose(const float & x, const float & y, const float & theta, const TurnDirection & turn_dir)
  : _x(x), _y(y), _theta(theta), _turn_dir(turn_dir)
  {}

  MotionPose operator-(const MotionPose & p2)
  {
    return MotionPose(
      this->_x - p2._x, this->_y - p2._y, this->_theta - p2._theta, TurnDirection::UNKNOWN);
  }

  float _x;
  float _y;
  float _theta;
  TurnDirection _turn_dir;
};

typedef std::vector<MotionPose> MotionPoses;

/**
 * @struct smac_planner::LatticeMetadata
 * @brief A struct of all lattice metadata
 */
struct LatticeMetadata
{
  float min_turning_radius;
  float grid_resolution;
  unsigned int number_of_headings;
  std::vector<float> heading_angles;
  unsigned int number_of_trajectories;
  std::string motion_model;
};

/**
 * @struct smac_planner::MotionPrimitive
 * @brief A struct of all motion primitive data
 */
struct MotionPrimitive
{
  unsigned int trajectory_id;
  float start_angle;
  float end_angle;
  float turning_radius;
  float trajectory_length;
  float arc_length;
  float straight_length;
  bool left_turn;
  MotionPoses poses;
};

typedef std::vector<MotionPrimitive> MotionPrimitives;
typedef std::vector<MotionPrimitive *> MotionPrimitivePtrs;

}  // namespace smac_planner

#endif  // SMAC_PLANNER__TYPES_HPP_
