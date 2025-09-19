#include <algorithm>
#include <optional>
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

void SearchInfo::setSearchSpace(const Rectangle& space) {
  _search_space = space;
}

void SearchInfo::removeSearchSpace(){
  _search_space.reset();
}

std::optional<Rectangle >SearchInfo::getSearchSpace() {
  return _search_space;
}

bool SearchInfo::isSearchSpaceSet()
{
  return (_search_space.has_value());
}

bool SearchInfo::isStartBehindSearchBounds(){

  if (is_start_behind_goal){
    return *is_start_behind_goal;
  }
  else{
    is_start_behind_goal = smac_planner::Utils::isBehindPose(_start_pose, _search_bound);
    return *is_start_behind_goal;
  }
}

PlanResult::PlanResult(uint32_t result_code, double cost, const std::string& message)
    : result_code(result_code), cost(cost), message(message) {}

// Getter for path
const std::vector<geometry_msgs::PoseStamped>& PlanResult::path() const {
  return _path;
}

// Setter for path
void PlanResult::setPath(const std::vector<geometry_msgs::PoseStamped>& new_path) {
  _path = new_path;
  _length = Utils::length(_path);
}

// Getter for length
double PlanResult::length() const {
  return _length;
}

bool PlanResult::isValid() const {
  return result_code == mbf_msgs::GetPathResult::SUCCESS;
}

PlanResult PlanResult::operator+(const PlanResult& other_result) const {
  if (!isValid() || !other_result.isValid()) {
    return PlanResult(
      mbf_msgs::GetPathResult::FAILURE,
      0.0,
      "One of the segments is invalid, cannot add paths");
  }

  PlanResult combined = *this;
  combined.cost += other_result.cost;

  const auto& other_path = other_result.path();
  std::vector<geometry_msgs::PoseStamped> new_combined_path = path();

  if (!new_combined_path.empty() && !other_path.empty() &&
      new_combined_path.back() == other_path.front()) {
        new_combined_path.insert(
          new_combined_path.end(),
          other_path.begin() + 1,
          other_path.end());
  } else {
    new_combined_path.insert(
      new_combined_path.end(),
      other_path.begin(),
      other_path.end());
    }
  combined.setPath(std::move(new_combined_path));
  return combined;
  }

Rectangle::Rectangle(geometry_msgs::Point diagonal_corner_1,
                     geometry_msgs::Point diagonal_corner_2,
                     double width)
    : _corner1(diagonal_corner_1),
      _corner2(diagonal_corner_2),
      _width(width)
{
    double dx = _corner2.x - _corner1.x;
    double dy = _corner2.y - _corner1.y;
    _length = std::sqrt(dx * dx + dy * dy);

    // Direction unit vector
    _dir.x = dx / _length;
    _dir.y = dy / _length;

    // visualize();
}

// void Rectangle::visualize() {
    // get_corners()
// }

// get_corners(){

// }
bool Rectangle::pointInside(const geometry_msgs::Point& point) const {
    // Translate point relative to corner1
    double px = point.x - _corner1.x;
    double py = point.y - _corner1.y;

    // Project onto direction and perpendicular axes
    double proj_along = px * _dir.x + py * _dir.y;
    double proj_perp  = px * -_dir.y + py * _dir.x;

    // Check bounds
    return (proj_along >= 0 && proj_along <= _length &&
            std::abs(proj_perp) <= _width / 2.0);
}
}  // namespace smac_planner
