#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <algorithm>
#include <common/grid_utils.hpp>

double h_cost(Node* from, Node* goal){
    int delta_x = std::abs(goal->cell.x - from->cell.x);
    int delta_y = std::abs(goal->cell.y - from->cell.y);
    if(delta_x < delta_y){
        return delta_y + 0.414 * delta_x;
    }
    else{
        return delta_x + 0.414 * delta_y;
    }
}

double g_cost(Node* from, Node* to, const ObstacleDistanceGrid& distances, const SearchParams params){
    int delta_x = std::abs(to->cell.x - from->cell.x);
    int delta_y = std::abs(to->cell.y - from->cell.y);
    double delta = 0.0;
    if(delta_x < delta_y){
        delta = 0.414 * delta_x + delta_y;
    }
    else{
        delta = 0.414 * delta_y + delta_x;
    }
    /*
    float cellDistance = distances(to->cell.x, to->cell.y);
    if(cellDistance < params.maxDistanceWithCost){
        delta += std::pow(params.maxDistanceWithCost - cellDistance, params.distanceCostExponent);
    }
    */
    return delta;
}

robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////

    Point<double> startPoint;
    startPoint.x = start.x;
    startPoint.y = start.y;
    cell_t startCell = global_position_to_grid_cell(startPoint, distances);

    Point<double> goalPoint;
    goalPoint.x = goal.x;
    goalPoint.y = goal.y;
    cell_t goalCell = global_position_to_grid_cell(goalPoint, distances);

    Node start_node(startCell.x, startCell.y);
    Node goal_node(goalCell.x, goalCell.y);
    PriorityQueue open_list;
    NodeList closed_list;

    open_list.push(&start_node);
    Node* current_node = open_list.pop();

    robot_path_t path;
    path.utime = start.utime;
    path.path_length = path.path.size();


    if((!current_node->is_in_map(distances) || current_node->is_obstacle(distances, params.minDistanceToObstacle)))
        return path;
    int count = 0;
    while(!(*current_node==goal_node)){
        closed_list.push(current_node);
        expand_node(current_node, distances, params, closed_list, open_list, goal_node);
        current_node = open_list.pop();
        count++;
        if(count>10000) return path;
    }

    std::vector<Node*> nodePath = extract_node_path(current_node);
    std::reverse(nodePath.begin(), nodePath.end());
    extract_pose_path(path, nodePath, distances);
    path.path_length = path.path.size();
    return path;
}

void expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params, NodeList& closed_list, PriorityQueue& open_list, Node& goal_node){
    const int xDeltas[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int yDeltas[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    for(int i = 0; i < 8; i++){
        cell_t cell(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i]);

        Node* neighbor;
        if(open_list.is_member(cell))
            neighbor = open_list.get_member(cell);
        else
            neighbor = new Node(cell.x, cell.y);

        if (!closed_list.is_member(neighbor->cell) && neighbor->is_in_map(distances) && !neighbor->is_obstacle(distances, params.minDistanceToObstacle)) {
            if (!open_list.is_member(neighbor->cell)) {
                neighbor->g_cost = node->g_cost + g_cost(node, neighbor, distances, params);
                neighbor->h_cost = h_cost(neighbor, &goal_node);
                neighbor->parent = node;
                open_list.push(neighbor);
            }
            else if (neighbor->g_cost > node->g_cost + g_cost(node, neighbor, distances, params)) {
                neighbor->g_cost = node->g_cost + g_cost(node, neighbor, distances, params);
                neighbor->parent = node;
            }
        }

    }
}


std::vector<Node*> extract_node_path(Node* node){
    std::vector<Node*> node_path;
    while(node != NULL){
        node_path.push_back(node);
        node = node->parent;
    }
    return node_path;
}

void extract_pose_path(robot_path_t& path, std::vector<Node*> nodePath, const ObstacleDistanceGrid& distances){
    int64_t utime = path.utime;
    for(auto& n: nodePath){
        Point<double> temp;
        temp.x = n->cell.x;
        temp.y = n->cell.y;


        Point<double> temp2 = grid_position_to_global_position(temp, distances);

        pose_xyt_t pose;
        pose.utime = utime;
        pose.x = temp2.x;
        pose.y = temp2.y;
        pose.theta = 0.0;
        path.path.push_back(pose);
    }

}