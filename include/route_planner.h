#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    // Add public variables or methods declarations here.
    float GetDistance() const {return distance;}
    void AStarSearch();

    // added Dijkstra's algorithm
    void Dijkstra();
    // std::vector<RouteModel::Node> ConstructFinalPath_Dijkstra(RouteModel::Node *);

    // The following methods have been made public so we can test them individually.
    void AddNeighbors(RouteModel::Node *current_node);
    float CalculateHValue(RouteModel::Node const *node);
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *);
    RouteModel::Node *NextNode();

    // added my own compare function
    // bool CompareNodes(const RouteModel::Node *node1, const RouteModel::Node *node2);
    // bool CompareNodes_Dijkstra(const RouteModel::Node *node1, const RouteModel::Node *node2);

  private:
    // Add private variables or methods declarations here.
    std::vector<RouteModel::Node*> open_list;   // vector of node pointers
    RouteModel::Node *start_node;     // ptr to start_node
    RouteModel::Node *end_node;       // ptr to end node

    float distance = 0.0f;
    RouteModel &m_Model;
};

#endif