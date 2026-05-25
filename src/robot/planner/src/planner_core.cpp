#include "planner_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

CellIndex PlannerCore::worldToGrid(const nav_msgs::msg::OccupancyGrid& map, double wx, double wy) const
{
  int x = static_cast<int>((wx - map.info.origin.position.x) / map.info.resolution);
  int y = static_cast<int>((wy - map.info.origin.position.y) / map.info.resolution);
  return CellIndex(x, y);
}

bool PlannerCore::inBounds(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& idx) const
{
  return idx.x >= 0 && idx.x < static_cast<int>(map.info.width) &&
         idx.y >= 0 && idx.y < static_cast<int>(map.info.height);
}

bool PlannerCore::isObstacle(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& idx) const
{
  int8_t val = map.data[static_cast<size_t>(idx.y) * map.info.width + static_cast<size_t>(idx.x)];
  return val > 20;
}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) const
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

std::vector<CellIndex> PlannerCore::getNeighbors(const CellIndex& idx) const
{
  return {
    CellIndex(idx.x + 1, idx.y),
    CellIndex(idx.x - 1, idx.y),
    CellIndex(idx.x,     idx.y + 1),
    CellIndex(idx.x,     idx.y - 1),
    CellIndex(idx.x + 1, idx.y + 1),
    CellIndex(idx.x - 1, idx.y + 1),
    CellIndex(idx.x + 1, idx.y - 1),
    CellIndex(idx.x - 1, idx.y - 1),
  };
}

nav_msgs::msg::Path PlannerCore::planPath(
  const nav_msgs::msg::OccupancyGrid& map,
  double start_x, double start_y,
  double goal_x, double goal_y)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = map.header.frame_id;

  CellIndex start = worldToGrid(map, start_x, start_y);
  CellIndex goal  = worldToGrid(map, goal_x, goal_y);

  if (!inBounds(map, start) || !inBounds(map, goal)) {
    RCLCPP_WARN(logger_, "Start or goal is out of map bounds");
    return path;
  }
  if (isObstacle(map, goal)) {
    bool snapped = false;
    for (int r = 1; r <= 20 && !snapped; ++r) {
      for (int dx = -r; dx <= r && !snapped; ++dx) {
        for (int dy = -r; dy <= r && !snapped; ++dy) {
          if (std::abs(dx) < r && std::abs(dy) < r) continue;
          CellIndex c(goal.x + dx, goal.y + dy);
          if (inBounds(map, c) && !isObstacle(map, c)) {
            goal = c;
            snapped = true;
          }
        }
      }
    }
    if (!snapped) {
      RCLCPP_WARN(logger_, "Goal is in an obstacle zone with no free cells nearby");
      return path;
    }
  }

  if (isObstacle(map, start)) {
    bool escaped = false;
    for (int r = 1; r <= 20 && !escaped; ++r) {
      for (int dx = -r; dx <= r && !escaped; ++dx) {
        for (int dy = -r; dy <= r && !escaped; ++dy) {
          if (std::abs(dx) < r && std::abs(dy) < r) continue;
          CellIndex c(start.x + dx, start.y + dy);
          if (inBounds(map, c) && !isObstacle(map, c)) {
            start = c;
            escaped = true;
          }
        }
      }
    }
    if (!escaped) {
      RCLCPP_WARN(logger_, "Robot is trapped; no free cell found nearby");
      return path;
    }
  }

  using OpenSet = std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF>;
  using IndexMap = std::unordered_map<CellIndex, CellIndex, CellIndexHash>;
  using ScoreMap = std::unordered_map<CellIndex, double, CellIndexHash>;
  using ClosedSet = std::unordered_map<CellIndex, bool, CellIndexHash>;

  OpenSet open_set;
  IndexMap came_from;
  ScoreMap g_score;
  ClosedSet closed;

  g_score[start] = 0.0;
  open_set.push(AStarNode(start, heuristic(start, goal)));

  bool found = false;
  while (!open_set.empty()) {
    AStarNode current = open_set.top();
    open_set.pop();

    if (current.index == goal) {
      found = true;
      break;
    }
    if (closed[current.index]) continue;
    closed[current.index] = true;

    for (const auto& nb : getNeighbors(current.index)) {
      if (!inBounds(map, nb) || isObstacle(map, nb) || closed[nb]) continue;

      bool diagonal = (nb.x != current.index.x && nb.y != current.index.y);
      double step = diagonal ? 1.414 : 1.0;
      double tentative_g = g_score[current.index] + step;

      auto it = g_score.find(nb);
      if (it == g_score.end() || tentative_g < it->second) {
        g_score[nb] = tentative_g;
        came_from[nb] = current.index;
        open_set.push(AStarNode(nb, tentative_g + heuristic(nb, goal)));
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(logger_, "A* could not find a path to the goal");
    return path;
  }

  std::vector<CellIndex> cell_path;
  for (CellIndex cur = goal; cur != start; cur = came_from[cur]) {
    cell_path.push_back(cur);
  }
  cell_path.push_back(start);
  std::reverse(cell_path.begin(), cell_path.end());

  const double res = map.info.resolution;
  const double ox  = map.info.origin.position.x;
  const double oy  = map.info.origin.position.y;

  for (const auto& cell : cell_path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = map.header.frame_id;
    pose.pose.position.x = ox + (cell.x + 0.5) * res;
    pose.pose.position.y = oy + (cell.y + 0.5) * res;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  return path;
}

}  // namespace robot
