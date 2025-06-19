// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2023, Open Navigation LLC
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

#include <string>
#include <memory>
#include <vector>
#include <limits>

#include "geometry_msgs/PoseStamped.h"
#include "mbf_msgs/GetPathResult.h"
#include "ros/console.h"
#include "smac_planner/utils.hpp"

#include "smac_planner/smac_planner_hybrid.hpp"

// #define BENCHMARK_TESTING

namespace smac_planner
{

SmacPlannerHybrid::SmacPlannerHybrid()
: _a_star(nullptr),
  _collision_checker(nullptr),
  _costmap(nullptr),
  _costmap_ros(nullptr),
  _costmap_downsampler(nullptr)
{
}

SmacPlannerHybrid::~SmacPlannerHybrid()
{
  ROS_INFO_NAMED("smac_planner_hybrid", "Destroying plugin %s of type SmacPlannerHybrid", _name.c_str());
}

void SmacPlannerHybrid::initialize(
  std::string name,
  costmap_2d::Costmap2DROS* costmap_ros) {
  _name = name;
  _costmap = costmap_ros->getCostmap();
  _costmap_ros = std::shared_ptr<costmap_2d::Costmap2DROS>(costmap_ros);
  _global_frame = costmap_ros->getGlobalFrameID();

  ROS_INFO_NAMED("smac_planner_hybrid", "Initializing %s of type SmacPlannerHybrid", name.c_str());

  ros::NodeHandle parent_nh("~");
  ros::NodeHandle private_nh(parent_nh, name);
  Utils::inflation_layer_name = private_nh.param("inflation_layer_name", std::string());
  _angle_quantizations = private_nh.param("angle_quantization", 72);
  _angle_bin_size = 2.0 * M_PI / _angle_quantizations;

  _path_smoother.initialize(private_nh);

  _collision_checker = std::make_unique<GridCollisionChecker>(_costmap_ros, _angle_quantizations);

  _raw_plan_publisher = private_nh.advertise<nav_msgs::Path>("unsmoothed_plan", 1);
  _final_plan_publisher = private_nh.advertise<nav_msgs::Path>("plan", 1);
  _expansions_publisher = private_nh.advertise<geometry_msgs::PoseArray>("expansions", 1);
  _planned_footprints_publisher = private_nh.advertise<visualization_msgs::MarkerArray>(
      "planned_footprints", 1);

  dsrv_ = std::make_unique<dynamic_reconfigure::Server<SmacPlannerHybridConfig>>(private_nh);
  dsrv_->setCallback(boost::bind(&SmacPlannerHybrid::reconfigureCB, this, _1, _2));
}

void SmacPlannerHybrid::reconfigureCB(SmacPlannerHybridConfig& config, uint32_t level)
{
  std::lock_guard<std::mutex> lock_reinit(_mutex);
  std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  _config = config;

  _motion_model = static_cast<MotionModel>(_config.motion_model_for_search);
  _search_info.non_straight_penalty = _config.non_straight_penalty;
  _search_info.change_penalty = _config.change_penalty;
  _search_info.reverse_penalty = _config.reverse_penalty;
  _search_info.cost_penalty = _config.cost_penalty;
  _search_info.retrospective_penalty = _config.retrospective_penalty;
  _search_info.analytic_expansion_ratio = _config.analytic_expansion_ratio;
  _search_info.analytic_expansion_max_length = _config.analytic_expansion_max_length / _costmap->getResolution();
  _search_info.analytic_expansion_max_cost = _config.analytic_expansion_max_cost;
  _search_info.analytic_expansion_max_cost_override = _config.analytic_expansion_max_cost_override;
  _search_info.cache_obstacle_heuristic = _config.cache_obstacle_heuristic;
  _search_info.allow_primitive_interpolation = _config.allow_primitive_interpolation;
  _search_info.downsample_obstacle_heuristic = _config.downsample_obstacle_heuristic;
  _search_info.use_quadratic_cost_penalty = _config.use_quadratic_cost_penalty;
  _search_info.allow_goal_overshoot = _config.allow_goal_overshoot;
  _search_info.goal_align_distance = _config.goal_align_distance;

  if (_config.max_on_approach_iterations <= 0) {
    ROS_WARN("On approach iteration selected as <= 0, "
      "disabling tolerance and on approach iterations.");
    _config.max_on_approach_iterations = std::numeric_limits<int>::max();
  }

  if (_config.max_iterations <= 0) {
    ROS_WARN("maximum iteration selected as <= 0, "
      "disabling maximum iterations.");
    _config.max_iterations = std::numeric_limits<int>::max();
  }

  if (_config.minimum_turning_radius < _costmap->getResolution() * _config.downsampling_factor) {
    ROS_WARN("Min turning radius cannot be less than the search grid cell resolution!");
    _config.minimum_turning_radius = _costmap->getResolution() * _config.downsampling_factor;
  }
  _search_info.minimum_turning_radius =
      _config.minimum_turning_radius / (_costmap->getResolution() * _config.downsampling_factor);

  _path_smoother.setMinTurningRadius(_config.minimum_turning_radius);

  // convert to grid coordinates
  if (!_config.downsample_costmap) {
    _config.downsampling_factor = 1;
  }

  _lookup_table_dim =
    static_cast<float>(_config.lookup_table_size) /
    static_cast<float>(_costmap->getResolution() * _config.downsampling_factor);

  // Make sure it's a whole number
  _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

  // Make sure it's an odd number
  if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
    ROS_INFO_NAMED("smac_planner_hybrid", "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd", _lookup_table_dim);
    _lookup_table_dim += 1.0;
  }

  // Initialize A* template
  _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _search_info);
  _a_star->initialize(
      _config.allow_unknown,
      _config.max_iterations,
      _config.max_on_approach_iterations,
      _config.terminal_checking_interval,
      _config.max_planning_time,
      _lookup_table_dim,
      _angle_quantizations);

  // Initialize costmap downsampler
  if (_config.downsample_costmap && _config.downsampling_factor > 1) {
    _costmap_downsampler = std::make_unique<CostmapDownsampler>();
    std::string topic_name = "downsampled_costmap";
    _costmap_downsampler->on_configure(
      _global_frame, topic_name, _costmap, _config.downsampling_factor);
  }

  ROS_INFO_NAMED("smac_planner_hybrid", "Configured plugin %s of type SmacPlannerHybrid with "
    "maximum iterations %i, max on approach iterations %i, and %s. Tolerance %.2f. "
    "Using motion model: %s.",
    _name.c_str(), _config.max_iterations, _config.max_on_approach_iterations,
    _config.allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
    _config.tolerance, toString(_motion_model).c_str());
}

  SmacPlannerHybrid::PlanResult SmacPlannerHybrid::planWithWaypoints(
    const geometry_msgs::PoseStamped& start,
    const std::vector<geometry_msgs::PoseStamped>& waypoints,
    const geometry_msgs::PoseStamped& end,
    const double tolerance)
  {
    PlanResult result;
    result.result_code = mbf_msgs::GetPathResult::SUCCESS;
    geometry_msgs::PoseStamped current_start = start;
    std::vector<geometry_msgs::PoseStamped> targets = waypoints;
    targets.push_back(end);  // Add final goal as last segment

    for (const auto& target : targets)
    {
      PlanResult segment_result;
      segment_result.result_code = getPath(current_start, target, tolerance, segment_result);

      if (!segment_result.isValid())
      {
        segment_result.result_code = mbf_msgs::GetPathResult::NO_PATH_FOUND;
        return segment_result;
      }
      else
      {
        result = result + segment_result;
      }
      current_start = target;  // Next segment starts where this one ends
    }
    return result;
  }


uint32_t SmacPlannerHybrid::makePlan(
  const geometry_msgs::PoseStamped & start,
  const geometry_msgs::PoseStamped & goal,
  double tolerance,
  std::vector<geometry_msgs::PoseStamped> & plan,
  double &cost,
  std::string &message)
{
  std::vector<geometry_msgs::PoseStamped> goal_align_poses;
  PlanResult plan_result;

    // If goal_align_distance is zero, proceed with normal planning
    if (_search_info.goal_align_distance == 0.0) {
      return getPath(start, goal, tolerance, plan_result);
    }

  geometry_msgs::PoseStamped align_pose_front, align_pose_back;
  align_pose_front.pose = Utils::getPoseAtDistanceAlongHeading(goal.pose, _search_info.goal_align_distance);
  align_pose_back.pose  = Utils::getPoseAtDistanceAlongHeading(goal.pose,  -_search_info.goal_align_distance);

  if (!_search_info.allow_goal_overshoot) {
    _search_info.setSearchBound(goal.pose);
    _search_info.setStart(start.pose.position);

    if (_search_info.isStartBehindSearchBounds()) {
      ROS_INFO_NAMED("smac_planner_hybrid", "Robot will align %f meters back of the goal pose", _search_info.goal_align_distance);
      goal_align_poses.push_back(align_pose_back);
    } else {
      ROS_INFO_NAMED("smac_planner_hybrid", "Robot will align %f meters front of the goal pose", _search_info.goal_align_distance);
      goal_align_poses.push_back(align_pose_front);
    }
  } else {
    ROS_INFO_NAMED("smac_planner_hybrid", "Robot may align either %f meters before or after the goal pose", _search_info.goal_align_distance);
    goal_align_poses = {align_pose_front, align_pose_back};
  }

  // For single align pose
  if (goal_align_poses.size() == 1) {
    PlanResult result = planWithWaypoints(start, goal_align_poses, goal, tolerance);
    plan = result.path;
    cost = result.cost;
    message = result.message;
    return result.result_code;
  }

  // For two align poses (choose the path with fewer poses)
  if (goal_align_poses.size() >= 2) {
    PlanResult result_option_1 = planWithWaypoints(start, {goal_align_poses[0]}, goal, tolerance);
    PlanResult result_option_2 = planWithWaypoints(start, {goal_align_poses[1]}, goal, tolerance);

    if (!result_option_1.isValid() && !result_option_2.isValid()) {
      message = "Could not plan to either of the goal align poses";
      return mbf_msgs::GetPathResult::NO_PATH_FOUND;
    }

    if (result_option_1.isValid() && (!result_option_2.isValid() || result_option_1.length <= result_option_2.length)) {
      // Use first option if it's valid and either the only valid option or has smaller path length
      plan = result_option_1.path;
      cost = result_option_1.cost;
      message = result_option_1.message;
    } else {
      // Use second option
      plan = result_option_2.path;
      cost = result_option_2.cost;
      message = result_option_2.message;
    }
    return mbf_msgs::GetPathResult::SUCCESS;
  }

  // Default case (shouldn't reach here)
  return getPath(start, goal, tolerance, plan_result);
}


uint32_t SmacPlannerHybrid::getPath(
    const geometry_msgs::PoseStamped & start,
    const geometry_msgs::PoseStamped & goal,
    double tolerance,
    PlanResult& plan_result)
{
  _planning_canceled = false;

  std::lock_guard<std::mutex> lock_reinit(_mutex);
  ros::Time a = ros::Time::now();

  std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  // Downsample costmap, if required
  costmap_2d::Costmap2D * costmap = _costmap;
  if (_costmap_downsampler) {
    costmap = _costmap_downsampler->downsample(_config.downsampling_factor);
  }
  _collision_checker->setCostmap(costmap);
  // Set collision checker and costmap information
  _collision_checker->setFootprint(
      _costmap_ros->getRobotFootprint(),
      _costmap_ros->getUseRadius(),
      Utils::findCircumscribedCost(_costmap_ros.get()));
  _a_star->setCollisionChecker(_collision_checker.get());
  _a_star->setSearchBounds(goal.pose, start.pose.position, _search_info.allow_goal_overshoot);

  // Set starting point, in A* bin search coordinates
  float mx, my;
  if (!costmap->worldToMapContinuous(start.pose.position.x, start.pose.position.y, mx, my)) {
    plan_result.message = "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
            std::to_string(start.pose.position.y) + ") was outside bounds";
    return mbf_msgs::GetPathResult::OUT_OF_MAP;
  }

  double orientation_bin = std::round(tf2::getYaw(start.pose.orientation) / _angle_bin_size);
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  // This is needed to handle precision issues
  if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
    orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  unsigned int orientation_bin_id = static_cast<unsigned int>(orientation_bin);

  if (_collision_checker->inCollision(mx, my, orientation_bin_id, _config.allow_unknown)) {
    plan_result.message = "Start pose is blocked";
    return mbf_msgs::GetPathResult::BLOCKED_START;
  }

  _a_star->setStart(mx, my, orientation_bin_id);

  // Set goal point, in A* bin search coordinates
  if (!costmap->worldToMapContinuous(goal.pose.position.x, goal.pose.position.y, mx, my)) {
    plan_result.message = "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") was outside bounds";
    return mbf_msgs::GetPathResult::OUT_OF_MAP;
  }

  orientation_bin = round(tf2::getYaw(goal.pose.orientation) / _angle_bin_size);
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  // This is needed to handle precision issues
  if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
    orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  orientation_bin_id = static_cast<unsigned int>(orientation_bin);

  if (_collision_checker->inCollision(mx, my, orientation_bin_id, _config.allow_unknown)) {
    plan_result.message = "Goal pose is blocked";
    return mbf_msgs::GetPathResult::BLOCKED_GOAL;
  }

  _a_star->setGoal(mx, my, orientation_bin_id);

  // Setup message
  nav_msgs::Path output_path;
  output_path.header.stamp = ros::Time::now();
  output_path.header.frame_id = _global_frame;
  geometry_msgs::PoseStamped pose;
  pose.header = output_path.header;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  // Compute output_path
  NodeHybrid::CoordinateVector path;
  int num_iterations = 0;
  std::string error;
  std::unique_ptr<std::vector<std::tuple<float, float, float>>> expansions = nullptr;
  if (_config.debug_visualizations) {
    expansions = std::make_unique<std::vector<std::tuple<float, float, float>>>();
  }

  if (const auto result = _a_star->createPath(
      path, num_iterations,
      _config.tolerance / static_cast<float>(costmap->getResolution()), [&](){ return _planning_canceled; }, expansions.get());
      result != mbf_msgs::GetPathResult::SUCCESS)
  {
    if (_config.debug_visualizations) {
      geometry_msgs::PoseArray msg;
      geometry_msgs::Pose msg_pose;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = _global_frame;
      for (auto & e : *expansions) {
        msg_pose.position.x = std::get<0>(e);
        msg_pose.position.y = std::get<1>(e);
        msg_pose.orientation = Utils::getWorldOrientation(std::get<2>(e));
        msg.poses.push_back(msg_pose);
      }
      _expansions_publisher.publish(msg);
    }

    // Note: If the start is blocked only one iteration will occur before failure,
    // but this should not happen because we check the start pose before planning
    if (num_iterations == 1) {
      if (*_a_star->getStart() == *_a_star->getGoal())
      {
        ROS_ERROR_NAMED(
            "smac_planner",
            "Start and goal are the same according to costmap resolution and angle bin quantization; but goal tolerance is not met");
        plan_result.message = "Start and goal are the same";
        return mbf_msgs::GetPathResult::INTERNAL_ERROR;
      }
      plan_result.message = "Start pose is blocked";
      return mbf_msgs::GetPathResult::BLOCKED_START;
    }

    if (result == mbf_msgs::GetPathResult::CANCELED) {
      plan_result.message = "Planner was cancelled";
    }
    else if (result == mbf_msgs::GetPathResult::PAT_EXCEEDED) {
      plan_result.message = "Exceeded maximum planning time";
    }
    else if (num_iterations >= _a_star->getMaxIterations()) {
      plan_result.message = "Exceeded maximum iterations";
    } else {
      plan_result.message = "No valid path found";
    }
    return result;
  }

  // Convert to world coordinates
  output_path.poses.reserve(path.size());
  for (int i = path.size() - 1; i >= 0; --i) {
    pose.pose = Utils::getWorldCoords(path[i].x, path[i].y, costmap);
    pose.pose.orientation = Utils::getWorldOrientation(path[i].theta);
    output_path.poses.push_back(pose);
  }

  if (_config.debug_visualizations) {
    // Publish expansions for debug
    geometry_msgs::PoseArray msg;
    geometry_msgs::Pose msg_pose;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = _global_frame;
    for (auto & e : *expansions) {
      msg_pose.position.x = std::get<0>(e);
      msg_pose.position.y = std::get<1>(e);
      msg_pose.orientation = Utils::getWorldOrientation(std::get<2>(e));
      msg.poses.push_back(msg_pose);
    }
    _expansions_publisher.publish(msg);

    // plot footprint path planned for debug
    if (_planned_footprints_publisher.getNumSubscribers() > 0) {
      visualization_msgs::Marker clear_all_marker;
      clear_all_marker.action = visualization_msgs::Marker::DELETEALL;
      visualization_msgs::MarkerArray marker_array;
      marker_array.markers.push_back(clear_all_marker);
      for (size_t i = 0; i < output_path.poses.size(); i++) {
        const std::vector<geometry_msgs::Point> edge =
            Utils::transformFootprintToEdges(output_path.poses[i].pose, _costmap_ros->getRobotFootprint());
        marker_array.markers.push_back(Utils::createMarker(edge, i, _global_frame, ros::Time::now()));
      }
      _planned_footprints_publisher.publish(marker_array);
    }
  }

  // Find how much time we have left to do smoothing
  ros::Time b = ros::Time::now();
  double time_remaining = _config.max_planning_time - (b - a).toSec();

#ifdef BENCHMARK_TESTING
  std::cout << "It took " << time_span.count() * 1000 <<
    " milliseconds with " << num_iterations << " iterations." << std::endl;
#endif

  // Smooth output_path
  if (_config.smooth_path) {
    // Publish raw path for comparison
    if (_raw_plan_publisher.getNumSubscribers() > 0) {
      _raw_plan_publisher.publish(output_path);
    }
    // overwrite start and goal poses to eliminate quantization-induced deviations
    output_path.poses.front() = start;
    output_path.poses.back() = goal;
    _path_smoother.smooth(output_path, costmap, time_remaining);
  }

  if (_final_plan_publisher.getNumSubscribers() > 0) {
    _final_plan_publisher.publish(output_path);
  }

#ifdef BENCHMARK_TESTING
  ros::Time c = ros::Time::now();
  std::cout << "It took " << (c - b).toSec() * 1000 <<
    " milliseconds to smooth path." << std::endl;
#endif
  plan_result.path = std::move(output_path.poses);
  plan_result.length = Utils::length(plan_result.path);
  return mbf_msgs::GetPathResult::SUCCESS;
}

bool SmacPlannerHybrid::cancel() {
  _planning_canceled = true;
  return true;
};

}  // namespace smac_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(smac_planner::SmacPlannerHybrid, mbf_costmap_core::CostmapPlanner)
