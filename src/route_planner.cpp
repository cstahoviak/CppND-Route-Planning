#include "route_planner.h"
#include <algorithm>
#include <queue>
#include <functional>

// this is the RoutePlanner class "constructor"
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.

    /* NOTE: According to the declaration of RouteModel::FindClosestNode(), it returns
    * a reference to a RouteModel::Node onject. This reference is assigned to
    * start_node and end_node.
    */
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // dereference the pointer end_node to pass the node object itself to distance()
    return node->distance(*end_node);
    // return (*node).distance(*end_node);      // another way to dereference
}


/* TODO 4: Complete the AddNeighbors method to expand the current node by adding all
 * unvisited neighbors to the open list.
 * Tips:
 * - Use the FindNeighbors() method of the current_node to populate
 *   current_node.neighbors vector with all the neighbors.
 * - For each node in current_node.neighbors, set the parent, the h_value, the
 *   g_value. 
 * - Use CalculateHValue below to implement the h-Value calculation.
 * - For each node in current_node.neighbors, add the neighbor to open_list and set
 *   the node's visited attribute to true.
 */

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    // populate the current_node.neighbors vector
    current_node->FindNeighbors();

    for( RouteModel::Node *neighbor : current_node->neighbors ) {
        if( !(neighbor->visited) ) {
            // set the parent node to the current node
            neighbor->parent = current_node;

            // get h-value
            neighbor->h_value = CalculateHValue( neighbor );

            // get g-value - current_node's g_value + distance from current to neighbor
            neighbor->g_value = current_node->g_value + current_node->distance( *neighbor );

            // set neighbor node's visited attibute to true
            neighbor->visited = true;

            // add neighbor node to open list
            open_list.push_back( neighbor );
        }
    }
}


// TODO 5: Complete the NextNode method to sort the open_list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

bool CompareNodes(const RouteModel::Node *node1, const RouteModel::Node *node2) {
    float f1 = node1->h_value + node1->g_value;
    float f2 = node2->h_value + node2->g_value;
    return f1 > f2;
}

RouteModel::Node *RoutePlanner::NextNode() {

    // sort the open_list
    std::sort(open_list.begin(), open_list.end(), CompareNodes);

    // returns a reference to the last element in the vector.
    RouteModel::Node *next_node = open_list.back();

    // removes the last element in the vector, effectively reducing the container size by one.
    open_list.pop_back();

    return next_node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // NOTE: current_node should be final node in search (end_node)

    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found {};

    // TODO: Implement your solution here.
    // 1. iterate through the parents of each node to construct complete path (path_found)
    // 2. at each iteration push parent node onto back of path_found and increment distance
    // 3. At the end, path_found will be in the reverse order, so it will need to be reversed

    // add current_node (end_node) to path_found
    path_found.push_back( *(current_node) );

    while( current_node->parent != nullptr ) {

        // add parent node to path_found vector
        path_found.push_back( *(current_node->parent) );
        
        // increment distance by the distance from the current node to its parent node
        distance += current_node->distance( *(current_node->parent) );

        current_node = current_node->parent;
    }

    // reverse the order of nodes in the path
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

/*
* TODO 7: Write the A* Search algorithm here.
* Tips:
* - Use the AddNeighbors method to add all of the neighbors of the current node to
*   the open_list.
* - Use the NextNode() method to sort the open_list and return the next node.
* - When the search has reached the end_node, use the ConstructFinalPath method to
*   return the final path that was found.
* - Store the final path in the m_Model.path attribute before the method exits. This
*   path will then be displayed on the map tile.
*/

void RoutePlanner::AStarSearch() {

    // TODO: Implement your solution here.

    // initialize the vector of open nodes
    open_list = {};

    // set current_node ptr to start_node ptr
    RouteModel::Node *current_node = start_node;

    // initialize the starting node and add start_node to open_list
    current_node->g_value = 0.0f;
    current_node->h_value = CalculateHValue( current_node );
    current_node->visited = true;
    open_list.push_back( current_node );

    while(open_list.size() > 0) {
        //sort the open_list and return the next node
        current_node = NextNode();

        if( current_node->distance( *end_node ) == 0 ) {
            m_Model.path = ConstructFinalPath( current_node );
            return;
        }
        else {
            // add neighbors of current_node to open_list
            AddNeighbors( current_node );
        }
    }

    // We've run out of new nodes to explore and haven't found a path.
    std::cout << "No path found!" << "\n";
    return;
}

bool CompareNodes_Dijkstra(const RouteModel::Node *node1, const RouteModel::Node *node2) {
    return node1->dist < node2->dist;
}

void RoutePlanner::Dijkstra() {

    std::cout << "Djistra's Algorithm:" << std::endl;
    std::cout << "start_node\t" << start_node << std::endl;

    // create lambda function for priority queue comparison
    auto compare =  [](RouteModel::Node* node1, RouteModel::Node* node2) {
        return node1->dist > node2->dist;
    };

    // create priority queue of Nodes
    std::priority_queue<RouteModel::Node*, std::vector<RouteModel::Node*>,
        decltype(compare)> Q(compare);

    // inititalization
    start_node->dist = 0.0f;
    start_node->visited = true;
    Q.push( start_node );

    // this declaration of the priority_queue isn't working
    // std::priority_queue<RouteModel::Node, std::vector<RouteModel::Node>, 
    //     std::function<bool(RouteModel::Node*, RouteModel::Node*)>> Q2(CompareNodes_Dijkstra);

    /* NOTE: The problem with what I'm doing below is that I'm effectively
    * creating a copy of every Node in RouteModel::m_Nodes, and adding each of them
    * to the priority_queue. I want to use pointers to the Nodes in m_Nodes rather
    * than creating copies of them.
    * 
    * UPDATE: I think I fixed it! - by using begin()/end() and NOT cbegin()/cend()
    * in the iterator-based for loop below.
    */

    // add all Nodes to priority_queue
    // for( auto it = m_Model.SNodes().begin(); it != m_Model.SNodes().end(); ++it ) {
    //     Q.push( &(*it) );
    // }

    while( !Q.empty() ) {

        // get Node off top of priority_queue
        RouteModel::Node* current_node = Q.top();
        std::cout << "\ntop node\t" << current_node << ",\tdist = " <<
            current_node->dist << std::endl;

        // mark current_node as 'visited' and remove from priority_queue
        current_node->visited = true;
        Q.pop();
        
        if( current_node->distance( *end_node ) == 0 ) {
            m_Model.path = ConstructFinalPath( current_node );
            return;
        }
        else {
            // populate the current_node->neighbors vector
            current_node->FindNeighbors();

            for( RouteModel::Node* neighbor : current_node->neighbors ) {
                double dist = current_node->distance( *neighbor );
                std::cout << "  neighbor " << neighbor << ", dist = " << std::to_string(dist) << std::endl;
                double alt = current_node->dist + current_node->distance( *neighbor );
                if( alt < neighbor->dist ) {
                    neighbor->dist = alt;
                    neighbor->parent = current_node;

                    // add neighbor to priority_queue
                    Q.push( neighbor );
                }
            }
        }
        
    }

    // We've run out of new nodes to explore and haven't found a path.
    std::cout << "No path found!" << "\n";
    return;
}

// std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath_Dijkstra(RouteModel::Node* current_node) {
//     // NOTE: current_node should be final node in search (end_node)

//     // Create path_found vector
//     std::vector<RouteModel::Node> path_found {};

//     // TODO: Implement your solution here.
//     // 1. iterate through the parents of each node to construct complete path (path_found)
//     // 2. at each iteration push parent node onto back of path_found and increment distance
//     // 3. At the end, path_found will be in the reverse order, so it will need to be reversed

//     // add current_node (end_node) to path_found
//     path_found.push_back( *(current_node) );

//     while( current_node->parent != nullptr ) {

//         // add parent node to path_found vector
//         path_found.push_back( *(current_node->parent) );
        
//         // increment distance by the distance from the current node to its parent node
//         distance += current_node->distance( *(current_node->parent) );

//         current_node = current_node->parent;
//     }

//     // reverse the order of nodes in the path
//     std::reverse(path_found.begin(), path_found.end());

//     distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
//     return path_found;
// } 