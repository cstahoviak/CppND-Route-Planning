#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

/* "Remember that inheritance in this case will allow you to use all of the public methods
 * and attributes of the Model class and Node struct in the derived RouteModel and
 * RouteModel::Node classes."
 * 
 * "The reason for extending the existing Model class and Node struct is to include additional
 * methods and variables which are useful for A* search. In particular, the new RouteModel::Node
 * class now allows nodes to store the following:
 * 
 *    - the h-value
 *    - the g-value
 *    - a "visited" flag
 *    - a vector of pointers to neighboring nodes
 * 
 * In addition, there are now methods for:
 * 
 *    - finding neighboring Node objects of a Node
 *    - getting the distance to other nodes
 *    - finding the closest node to a given (x, y) coordinate pair
*/

// RouteModel class inherits from the IO2D Model class
class RouteModel : public Model {

  public:
    // the RouteModel "sub-class" Node inherits from the Model struct also called Node
    class Node : public Model::Node {
      public:
        Node * parent = nullptr;          // ptr to node parent
        float h_value = std::numeric_limits<float>::max();
        float g_value = 0.0;
        bool visited = false;
        std::vector<Node *> neighbors;    // vector of node ptrs

        void FindNeighbors();             // used to populate the neighbors vector
        float distance(Node other) const {
            return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2));
        }

        // overloaded contructor
        Node(){}
        // uses a contructor list (: operator) initializes all variables
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}

      private:
        int index;
        Node * FindNeighbor(std::vector<int> node_indices);
        RouteModel * parent_model = nullptr;
    };

    RouteModel(const std::vector<std::byte> &xml);
    Node &FindClosestNode(float x, float y);
    auto &SNodes() { return m_Nodes; }
    std::vector<Node> path;     // stores final path from start node to end node
    
  private:
    void CreateNodeToRoadHashmap();
    std::unordered_map<int, std::vector<const Model::Road *>> node_to_road;
    std::vector<Node> m_Nodes;    // all nodes in Model

};

#endif
