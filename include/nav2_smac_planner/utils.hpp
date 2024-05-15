// Copyright (c) 2021, Samsung Research America
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

#ifndef NAV2_SMAC_PLANNER__UTILS_HPP_
#define NAV2_SMAC_PLANNER__UTILS_HPP_

#include <vector>
#include <memory>
#include <string>

#include "nlohmann/json.hpp"
#include "Eigen/Core"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose.h"
#include "tf2/utils.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "costmap_2d/inflation_layer.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav2_smac_planner/types.hpp"
#include <ros/ros.h>

namespace nav2_smac_planner
{

class Utils {
public:
  /**
  * @brief Create an Eigen Vector2D of world poses from continuous map coords
  * @param mx float of map X coordinate
  * @param my float of map Y coordinate
  * @param costmap Costmap pointer
  * @return Eigen::Vector2d eigen vector of the generated path
  */
  static inline geometry_msgs::Pose getWorldCoords(
      const float &mx, const float &my, const costmap_2d::Costmap2D *costmap) {
    geometry_msgs::Pose msg;
    msg.position.x =
        static_cast<float>(costmap->getOriginX()) + (mx + 0.5) * costmap->getResolution();
    msg.position.y =
        static_cast<float>(costmap->getOriginY()) + (my + 0.5) * costmap->getResolution();
    return msg;
  }

  /**
  * @brief Create quaternion from radians
  * @param theta continuous bin coordinates angle
  * @return quaternion orientation in map frame
  */
  static inline geometry_msgs::Quaternion getWorldOrientation(
      const float &theta) {
    // theta is in radians already
    tf2::Quaternion q;
    q.setEuler(0.0, 0.0, theta);
    return tf2::toMsg(q);
  }

  static inline std::string inflation_layer_name;

  /**
   * @brief Find and return the costmap's inflation layer. If inflation_layer_name is not
   * provided, we return the first layer that can be casted to costmap_2d::InflationLayer.
   * @param costmap Pointer to the global costmap.
   * @return Pointer to the inflation layer or nullptr if not found.
   */
  static inline costmap_2d::InflationLayer *findInflationLayer(costmap_2d::Costmap2DROS *costmap) {
    if (inflation_layer_name.empty()) {
      ROS_WARN_ONCE("Inflation layer name not provided");
      ROS_WARN_ONCE("We will use the first layer that can be casted to costmap_2d::InflationLayer");
    }

    for (auto layer = costmap->getLayeredCostmap()->getPlugins()->begin();
         layer != costmap->getLayeredCostmap()->getPlugins()->end(); ++layer) {
      auto inflation_layer = boost::dynamic_pointer_cast<costmap_2d::InflationLayer>(*layer);
      if (inflation_layer && (inflation_layer_name.empty() || inflation_layer->getName() == inflation_layer_name)) {
        if (inflation_layer_name.empty()) {
          ROS_INFO_STREAM_ONCE("Using " << inflation_layer->getName() << " as inflation layer");
        }
        return inflation_layer.get();
      }
    }
    return nullptr;
  }

  /**
  * @brief Find the min cost of the inflation decay function for which the robot MAY be
  * in collision in any orientation
  * @param costmap Costmap2DROS to get minimum inscribed cost (e.g. 128 in inflation layer documentation)
  * @return double circumscribed cost, any higher than this and need to do full footprint collision checking
  * since some element of the robot could be in collision
  */
  static inline double findCircumscribedCost(costmap_2d::Costmap2DROS *costmap) {
    double result = -1.0;
    std::vector<std::shared_ptr<costmap_2d::Layer>>::iterator layer;

    costmap_2d::InflationLayer *inflation_layer = findInflationLayer(costmap);
    if (inflation_layer != nullptr) {
      double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
      double resolution = costmap->getCostmap()->getResolution();
      result = static_cast<double>(inflation_layer->computeCost(circum_radius / resolution));
    } else {
      ROS_WARN("No inflation layer found in costmap configuration. "
               "If this is an SE2-collision checking plugin, it cannot use costmap potential "
               "field to speed up collision checking by only checking the full footprint "
               "when robot is within possibly-inscribed radius of an obstacle. This may "
               "significantly slow down planning times!");
    }

    return result;
  }

  /**
   * @brief convert json to lattice metadata
   * @param[in] json json object
   * @param[out] lattice meta data
   */
  static inline void fromJsonToMetaData(const nlohmann::json &json, LatticeMetadata &lattice_metadata) {
    json.at("turning_radius").get_to(lattice_metadata.min_turning_radius);
    json.at("grid_resolution").get_to(lattice_metadata.grid_resolution);
    json.at("num_of_headings").get_to(lattice_metadata.number_of_headings);
    json.at("heading_angles").get_to(lattice_metadata.heading_angles);
    json.at("number_of_trajectories").get_to(lattice_metadata.number_of_trajectories);
    json.at("motion_model").get_to(lattice_metadata.motion_model);
  }

  /**
   * @brief convert json to pose
   * @param[in] json json object
   * @param[out] pose
   */
  static inline void fromJsonToPose(const nlohmann::json &json, MotionPose &pose) {
    pose._x = json[0];
    pose._y = json[1];
    pose._theta = json[2];
  }

  /**
   * @brief convert json to motion primitive
   * @param[in] json json object
   * @param[out] motion primitive
   */
  static inline void fromJsonToMotionPrimitive(
      const nlohmann::json &json, MotionPrimitive &motion_primitive) {
    json.at("trajectory_id").get_to(motion_primitive.trajectory_id);
    json.at("start_angle_index").get_to(motion_primitive.start_angle);
    json.at("end_angle_index").get_to(motion_primitive.end_angle);
    json.at("trajectory_radius").get_to(motion_primitive.turning_radius);
    json.at("trajectory_length").get_to(motion_primitive.trajectory_length);
    json.at("arc_length").get_to(motion_primitive.arc_length);
    json.at("straight_length").get_to(motion_primitive.straight_length);
    json.at("left_turn").get_to(motion_primitive.left_turn);

    for (unsigned int i = 0; i < json["poses"].size(); i++) {
      MotionPose pose;
      fromJsonToPose(json["poses"][i], pose);
      motion_primitive.poses.push_back(pose);
    }
  }

  /**
   * @brief transform footprint into edges
   * @param[in] robot position , orientation and  footprint
   * @param[out] robot footprint edges
   */
  static inline std::vector<geometry_msgs::Point> transformFootprintToEdges(
      const geometry_msgs::Pose &pose,
      const std::vector<geometry_msgs::Point> &footprint) {
    const double &x = pose.position.x;
    const double &y = pose.position.y;
    const double &yaw = tf2::getYaw(pose.orientation);

    std::vector<geometry_msgs::Point> out_footprint;
    out_footprint.resize(2 * footprint.size());
    for (unsigned int i = 0; i < footprint.size(); i++) {
      out_footprint[2 * i].x = x + cos(yaw) * footprint[i].x - sin(yaw) * footprint[i].y;
      out_footprint[2 * i].y = y + sin(yaw) * footprint[i].x + cos(yaw) * footprint[i].y;
      if (i == 0) {
        out_footprint.back().x = out_footprint[i].x;
        out_footprint.back().y = out_footprint[i].y;
      } else {
        out_footprint[2 * i - 1].x = out_footprint[2 * i].x;
        out_footprint[2 * i - 1].y = out_footprint[2 * i].y;
      }
    }
    return out_footprint;
  }

  /**
   * @brief initializes marker to visualize shape of linestring
   * @param edge       edge to mark of footprint
   * @param i          marker ID
   * @param frame_id   frame of the marker
   * @param timestamp  timestamp of the marker
   * @return marker populated
   */
  static inline visualization_msgs::Marker createMarker(
      const std::vector<geometry_msgs::Point> edge,
      unsigned int i, const std::string &frame_id, const ros::Time &timestamp) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = timestamp;
    marker.frame_locked = false;
    marker.ns = "planned_footprint";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.lifetime = ros::Duration(0, 0);

    marker.id = i;
    for (auto &point: edge) {
      marker.points.push_back(point);
    }

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.3f;
    return marker;
  }
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__UTILS_HPP_
