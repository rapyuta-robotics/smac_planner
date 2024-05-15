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

#ifndef NAV2_SMAC_PLANNER__SMAC_PLANNER_HYBRID_HPP_
#define NAV2_SMAC_PLANNER__SMAC_PLANNER_HYBRID_HPP_

#include <memory>
#include <vector>
#include <string>

#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/smoother.hpp"
#include "nav2_smac_planner/costmap_downsampler.hpp"
#include "nav2_smac_planner/SmacPlannerHybridConfig.h"
#include "nav_msgs/OccupancyGrid.h"
#include "mbf_costmap_core/costmap_planner.h"
#include "nav_msgs/Path.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "costmap_2d/costmap_2d.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "tf2/utils.h"

namespace nav2_smac_planner
{

class SmacPlannerHybrid : public mbf_costmap_core::CostmapPlanner
{
public:
  /**
   * @brief constructor
   */
  SmacPlannerHybrid();

  /**
   * @brief destructor
   */
  ~SmacPlannerHybrid();

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
   * @brief Creating a plan from start to goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @param tolerance If the goal is obstructed, how many meters the planner can relax the constraint
   *        in x and y before failing
   * @param plan The plan... filled by the planner
   * @param cost The cost for the the plan
   * @param message Optional more detailed outcome as a string
   * @return Result code as described on GetPath action result
   */
  uint32_t makePlan(
    const geometry_msgs::PoseStamped & start,
    const geometry_msgs::PoseStamped & goal,
    double tolerance,
    std::vector<geometry_msgs::PoseStamped> & plan,
    double & cost,
    std::string & message)  override;

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time.
   * @return Always True, as this plugin implements cancelling.
   */
  bool cancel() override;

protected:
  // Dynamic parameters handler
  /**
   * @brief Callback executed when a paramter change is detected
   * @param config Planner configuration
   */
  void reconfigureCB(SmacPlannerHybridConfig& config, uint32_t level);

  std::unique_ptr<dynamic_reconfigure::Server<SmacPlannerHybridConfig>> dsrv_;

  std::unique_ptr<AStarAlgorithm<NodeHybrid>> _a_star;
  GridCollisionChecker _collision_checker;
  Smoother _path_smoother;
  costmap_2d::Costmap2D * _costmap;
  std::shared_ptr<costmap_2d::Costmap2DROS> _costmap_ros;
  std::unique_ptr<CostmapDownsampler> _costmap_downsampler;
  std::string _global_frame, _name;
  SmacPlannerHybridConfig _config;
  float _lookup_table_dim;
  double _angle_bin_size;
  unsigned int _angle_quantizations;
  SearchInfo _search_info;
  bool _planning_canceled;
  MotionModel _motion_model;
  ros::Publisher _raw_plan_publisher;
  ros::Publisher _final_plan_publisher;
  ros::Publisher _planned_footprints_publisher;
  ros::Publisher _expansions_publisher;
  std::mutex _mutex;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__SMAC_PLANNER_HYBRID_HPP_
