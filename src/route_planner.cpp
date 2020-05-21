#include "route_planner.h"

#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  float heuristic = node->distance(*end_node);
  return heuristic;
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (auto neighbor : current_node->neighbors) {
    neighbor->parent = current_node;
    neighbor->g_value =
        current_node->g_value + current_node->distance(*neighbor);
    neighbor->h_value = CalculateHValue(neighbor);
    open_list.emplace_back(neighbor);
    neighbor->visited = true;
  }
}

RouteModel::Node *RoutePlanner::NextNode() {
  // Sort the open list descendingly by g + h;
  std::sort(open_list.begin(), open_list.end(),
            [&](RouteModel::Node *a, RouteModel::Node *b) {
              return a->g_value + a->h_value > b->g_value + b->h_value;
            });
  // Create a pointer to the node with the lowest sum;
  RouteModel::Node *lowest_sum = open_list.back();
  // Remove the above node from the open list;
  open_list.pop_back();

  return lowest_sum;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(
    RouteModel::Node *current_node) {
  // Create path_found vector
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;

  while (current_node->parent) {
    // Add the distance
    distance += current_node->parent->distance(*current_node);
    path_found.emplace_back(*current_node);
    current_node = current_node->parent;
  }

  // Push back the starting node, which has no parent thus left out from the
  // while loop
  path_found.emplace_back(*current_node);
  distance *= m_Model.MetricScale();  // Multiply the distance by the scale of
                                      // the map to get meters.
  // Reverse path as currently from end to start
  std::reverse(path_found.begin(), path_found.end());
  return path_found;
}

void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = nullptr;
  open_list.emplace_back(start_node);
  start_node->visited = true;

  while (open_list.size() > 0) {
    current_node = NextNode();

    // Exit while loop if current node is end node
    if (current_node == end_node)
      m_Model.path = ConstructFinalPath(current_node);

    AddNeighbors(current_node);
  }
}