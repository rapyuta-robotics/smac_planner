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

#ifndef SMAC_PLANNER__SMAC_PLANNER_LATTICE_HPP_
#define SMAC_PLANNER__SMAC_PLANNER_LATTICE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "smac_planner/a_star.hpp"
#include "smac_planner/smoother.hpp"
#include "smac_planner/utils.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include "mbf_costmap_core/costmap_planner.h"
#include "nav_msgs/Path.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "costmap_2d/costmap_2d.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/utils.h"

namespace smac_planner
{

class SmacPlannerLattice : public mbf_costmap_core::CostmapPlanner
{
public:
  /**
   * @brief constructor
   */
  SmacPlannerLattice();

  /**
   * @brief destructor
   */
  ~SmacPlannerLattice();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
   */
  void initialize(
    std::string name,
    costmap_2d::Costmap2DROS* costmap_ros) override;

  /**
   * @brief Creating a plan from start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @param cancel_checker Function to check if the action has been canceled
   * @return nav_msgs::Path of the generated path
   */
  nav_msgs::Path createPlan(
    const geometry_msgs::PoseStamped & start,
    const geometry_msgs::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

protected:
  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::SetParametersResult
  dynamicParametersCallback(std::vector<ros::Parameter> parameters);

  std::unique_ptr<AStarAlgorithm<NodeLattice>> _a_star;
  GridCollisionChecker _collision_checker;
  std::unique_ptr<Smoother> _smoother;
  costmap_2d::Costmap2D * _costmap;
  std::shared_ptr<costmap_2d::Costmap2DROS> _costmap_ros;
  MotionModel _motion_model;
  LatticeMetadata _metadata;
  std::string _global_frame, _name;
  SearchInfo _search_info;
  bool _allow_unknown;
  int _max_iterations;
  int _max_on_approach_iterations;
  int _terminal_checking_interval;
  float _tolerance;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::Path>::SharedPtr _raw_plan_publisher;
  double _max_planning_time;
  double _lookup_table_size;
  bool _debug_visualizations;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::MarkerArray>::SharedPtr
    _planned_footprints_publisher;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::PoseArray>::SharedPtr
    _expansions_publisher;
  std::mutex _mutex;
  rclcpp_lifecycle::LifecycleNode::WeakPtr _node;

  // Dynamic parameters handler
  ros::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _dyn_params_handler;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__SMAC_PLANNER_LATTICE_HPP_
