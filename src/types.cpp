#include <smac_planner/types.hpp>
#include <geometry_msgs/Point.h>
#include <smac_planner/utils.hpp>

namespace smac_planner
{

void SearchInfo::setStart(const geometry_msgs::Point& start){
  _start_pose = start;
  is_start_behind_goal.reset();
}

geometry_msgs::Pose SearchInfo::getSearchBound(){
  return _search_bound;
}

void SearchInfo::setSearchBound(const geometry_msgs::Pose& search_bound){
  _search_bound = search_bound;
  is_start_behind_goal.reset();
}

bool SearchInfo::isStartBehindSearchBounds(){
  assert(_start_pose.header.stamp != ros::Time(0) && "Start pose not set");
  assert(_search_bound.header.stamp != ros::Time(0) && "Search bound pose not set");

  if (is_start_behind_goal){
    return *is_start_behind_goal;
  }
  else{
    is_start_behind_goal = smac_planner::Utils::isBehindPose(_start_pose, _search_bound);
    return *is_start_behind_goal;
  }
}

}  // namespace smac_planner
