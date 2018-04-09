#include "dlian.h"

DLian::DLian() {}
DLian::DLian(double HW) { hweight = HW; }
DLian::~DLian()
{
    if (!NODES.empty()) NODES.erase(NODES.begin(), NODES.end());
}

inline bool CutOrSqueeze(Node* to, Node* from) {
    if (to->cell.x == from->cell.x) {
        return (to->parent->cell == Cell(from->cell.x - 1, from->cell.y) ||
                to->parent->cell == Cell(from->cell.x + 1, from->cell.y));
    }
    else if (to->cell.y == from->cell.y) {
        return (to->parent->cell == Cell(from->cell.x, from->cell.y - 1) ||
                to->parent->cell == Cell(from->cell.x, from->cell.y + 1));
    }
    else return false;
}

double DLian::getCost(int a_i, int a_j, int b_i, int b_j) const {
    return sqrt(abs(a_i - b_i) * abs(a_i - b_i) + abs(a_j - b_j) * abs(a_j - b_j));
}

Key DLian::CalculateKey(Node& vertex) {
    Key res(std::min(vertex.g, vertex.rhs + hweight * getCost(vertex.i, vertex.j, goal->i, goal->j)), std::min(vertex.g, vertex.rhs));
    return res;
}

Node* DLian::getFromNodes(Node current_node, int width, Node* parent) {
    auto it = NODES.find(current_node.convolution(width));
    if (it != NODES.end()) {
        auto range = NODES.equal_range(it->first);
        for (auto it = range.first; it != range.second; ++it)
            if (it->second.parent == nullptr || (it->second.parent->i == parent->i && it->second.parent->j == parent->j))
                return &(it->second);
    } else {
        return nullptr;
    }
}

void DLian::Initialize(Map &map)
{
    Node start_node = Node(map.start_i, map.start_j);
    Node goal_node = Node(map.goal_i, map.goal_j);

    NODES.insert({goal_node.convolution(map.get_width()), goal_node});
    goal = getFromNodes(goal_node, map.get_width());

    start_node.rhs = 0;
    start_node.key = CalculateKey(start_node);
    NODES.insert({start_node.convolution(map.get_width()), start_node});
    start = getFromNodes(start_node, map.get_width());

    OPEN.resize(map.get_height());
    OPEN.put(start); //add start cell to OPEN list

    current_result.goal_became_unreachable = false;
}


//The main pathbuilding function
LPASearchResult DLian::FindThePath(Map &map, EnvironmentOptions options)
{
    std::chrono::time_point<std::chrono::system_clock> startt, end;
    startt = std::chrono::system_clock::now();
    number_of_steps = 0;
    Initialize(map, options.metrictype); //algorithm initialization

    if(!ComputeShortestPath(map, options)) {
        current_result.pathfound = false;
        current_result.pathlength = 0;
        end = std::chrono::system_clock::now();
        current_result.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - startt).count()) / 1000000000;
        std::cout << "THE PATH DOES NOT EXIST ON THE INITIAL MAP\n";
        return current_result;
    }
    Changes changes = map.DamageTheMap(path); //force map to change (sufficient for the correct testing)
    for (auto dam : changes.occupied) { //for each damaged (0 -> 1) cell recounting values for it's neighbors
        if (NODES.count(vertex(dam, map.get_height()))) {
            Node* d = &(NODES.find(vertex(dam, map.get_height()))->second);
            OPEN.remove_if(d);
            for (auto neighbor: GetSurroundings(d, map, options)) {
                if (!(neighbor->cell == start->cell) && (neighbor->parent->cell == dam || CutOrSqueeze(neighbor, d))) {
                    Node min_val = GetMinPredecessor(neighbor, map, options);
                    if (!min_val.parent) { //means that neighbor is now unreachable
                        OPEN.remove_if(neighbor);
                        if(neighbor->cell == goal->cell) { //means that after changes goal cell became unreachable
                            current_result.pathfound = false;
                            current_result.pathlength = 0;
                            end = std::chrono::system_clock::now();
                            current_result.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - startt).count()) / 1000000000;
                            std::cout << "AFTER THE FIRST MAP CHANGE THE GOAL CELL BECAME UNREACHABLE. PATH DOES NOT EXIST\n";
                            current_result.goal_became_unreachable = true;
                            return current_result;
                        }
                    } else {
                        neighbor->rhs = min_val.rhs;
                        neighbor->parent = min_val.parent;
                        UpdateVertex(neighbor, options.metrictype);
                    }
                }
            }
        }
    }
    if(!ComputeShortestPath(map, options)) {
        current_result.pathfound = false;
        current_result.pathlength = 0;
        end = std::chrono::system_clock::now();
        current_result.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - startt).count()) / 1000000000;
        std::cout << "AFTER THE FIRST MAP CHANGE THE PATH DOES NOT EXIST\n";
        return current_result;
    }
    changes = map.ClearTheMap(changes.occupied); //force map to change (here: clear all previous damage)
    for (auto cleared : changes.cleared) { //for each cell, that was cleared (1 -> 0) recounting values for it & it's neighbors
       Node new_node(cleared);
       new_node.rhs = std::numeric_limits<double>::infinity();
       new_node.g = std::numeric_limits<double>::infinity();
       NODES[vertex(cleared, map.get_height())] = new_node;
       Node * cl = &(NODES.find(vertex(cleared, map.get_height()))->second);
       Node min_val = GetMinPredecessor(cl, map, options);
       if (min_val.parent) { //means that neighbor is reachable
           cl->rhs = min_val.rhs;
           cl->parent = min_val.parent;
           cl->g = min_val.parent->g + GetCost(cl->cell, min_val.parent->cell);
           UpdateVertex(cl, options.metrictype);
       } else {
           break; //means that this cell is unreachable and there's no need to recount values for it's neighbors
       }
       for (auto neighbor : GetSuccessors(cl, map, options)) {
           if (neighbor->rhs > cl->g + GetCost(neighbor->cell, cl->cell)) {
               neighbor->parent = cl;
               neighbor->rhs = cl->g + GetCost(neighbor->cell, cl->cell);
               UpdateVertex(neighbor, options.metrictype);
           }
           if (neighbor->cell.x == cl->cell.x || neighbor->cell.y == cl->cell.y) {
               Node min_val = GetMinPredecessor(neighbor, map, options);
               if (!min_val.parent) { //means that neighbor is now unreachable
                   OPEN.remove_if(neighbor);
                   if(neighbor->cell == goal->cell) { //means that goal became unreachable
                       current_result.pathfound = false;
                       current_result.pathlength = 0;
                       end = std::chrono::system_clock::now();
                       current_result.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - startt).count()) / 1000000000;
                       std::cout << "AFTER THE SECOND MAP CHANGE THE GOAL CELL BECAME UNREACHABLE. PATH DOES NOT EXIST\n";
                       current_result.goal_became_unreachable = true;
                       return current_result;
                   }
               } else {
                   neighbor->rhs = min_val.rhs;
                   neighbor->parent = min_val.parent;
                   UpdateVertex(neighbor, options.metrictype);
               }
           }
       }
    }
    if(!ComputeShortestPath(map, options)){
        current_result.pathfound = false;
        current_result.pathlength = 0;
        end = std::chrono::system_clock::now();
        current_result.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - startt).count()) / 1000000000;
        std::cout << "AFTER THE SECOND MAP CHANGE THE PATH DOES NOT EXIST\n";
        return current_result;
    }
    end = std::chrono::system_clock::now();
    current_result.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - startt).count()) / 1000000000;
    if (current_result.pathfound) {
        makeSecondaryPath();
        current_result.hppath = &hpath;
    }
    return current_result;
}



void DLian::UpdateVertex(Node* node) //adding and removing vertices from OPEN list
{
    if (!(node->IsConsistent())) {
        node->key = CalculateKey(*node);
        OPEN.put(node); //add vertex u in OPEN list or change it's key if it is already there
    } else {
        OPEN.remove_if(node); //if vertex u is in OPEN list, remove it
    }
}

void DLian::update(const Node* current_node, Node new_node, bool &successors, const Map &map) {
    if (!checkLineSegment(map, *current_node, new_node)) return;
    if (pivotRadius > 0 && (new_node.i != map.goal_i || new_node.j != map.goal_j) && !checkPivotCircle(map, new_node)) return;

    Node* node = getFromNodes(new_node, map.get_width(), current_node);
    if (node == nullptr) {
        NODES.insert({new_node.convolution(map.get_width()), new_node});
        node = getFromNodes(new_node, map.get_width(), current_node);
    }
    node->rhs = std::min(node->rhs, current_node->g + getCost(node->i, node->j, current_node->i, current_node->j));

    if (!(node->IsConsistent())) {
        new_node->key = CalculateKey(*node);
        OPEN.put(node); //add vertex u in OPEN list or change it's key if it is already there
    } else {
        OPEN.remove_if(node); //if vertex u is in OPEN list, remove it
    }
    successors = true;
}

bool DLian::expand(const Node* curNode, const Map &map) {
    bool successors_are_fine = false;
    auto parent = &(NODES.find(curNode->convolution(map.get_width()))->second);
    if (curNode->parent != nullptr) {
        int node_straight_ahead = (int)round(curNode->angle * circle_nodes.size() / 360) % circle_nodes.size();
        double angle = fabs(curNode->angle - circle_nodes[node_straight_ahead].heading);
        if ((angle <= 180 && angle <= angleLimit) || (angle > 180 && 360 - angle <= angleLimit)) {
            int new_pos_i = curNode->i + circle_nodes[node_straight_ahead].i;
            int new_pos_j = curNode->j + circle_nodes[node_straight_ahead].j;
            if (map.CellOnGrid(new_pos_i, new_pos_j) && map.CellIsTraversable(new_pos_i, new_pos_j)) {
                Node newNode = Node(new_pos_i, new_pos_j, parent);
                newNode.angle = circleNodes[current_distance][node_straight_ahead].heading;
                newNode.radius = curNode.radius;
                //newNode.parent = parent;

                update(parent, newNode, successors_are_fine, map);
             }
        } // now we will expand neighbors that are closest to the node that lies straight ahead

        std::vector<int> candidates = std::vector<int>{node_straight_ahead, node_straight_ahead};
        bool limit1 = true;
        bool limit2 = true;
        while (++candidates[0] != --candidates[1] && (limit1 || limit2)) { // untill the whole circle is explored or we exessed anglelimit somewhere
            if (candidates[0] >= circle_nodes.size()) candidates[0] = 0;
            if (candidates[1] < 0) candidates[1] = circle_nodes.size() - 1;

            for (auto cand : candidates) {
                double angle = fabs(curNode->angle - circle_nodes[cand].heading);
                if ((angle <= 180 && angle <= angleLimit) || (angle > 180 && 360 - angle <= angleLimit)) {
                    int new_pos_i = curNode->i + circle_nodes[cand].i;
                    int new_pos_j = curNode->j + circle_nodes[cand].j;

                    if (!map.CellOnGrid(new_pos_i, new_pos_j)) continue;
                    if (map.CellIsObstacle(new_pos_i, new_pos_j)) continue;

                    Node newNode = Node(new_pos_i, new_pos_j, parent);
                    newNode.angle = circle_nodes[cand].heading;
                    newNode.radius = curNode->radius;
                    //newNode.parent = parent;

                    update(curNode, newNode, successors_are_fine, map);
                } else {
                    if (cand == candidates[0]) limit1 = false;
                    else limit2 = false;
                }
            }
        }
    } else { // when we do not have parent, we should explore all neighbors
        int angle_position(-1), new_pos_i, new_pos_j;
        for (auto node : circle_nodes) {
            new_pos_i = curNode->i + node.i;
            new_pos_j = curNode->j + node.j;
            angle_position++;

            if (!map.CellOnGrid(new_pos_i, new_pos_j)) continue;
            if (map.CellIsObstacle(new_pos_i, new_pos_j)) continue;

            Node newNode = Node(new_pos_i, new_pos_j, parent);
            newNode.radius = curNode.radius;
            newNode.angle = circleNodes[current_distance][angle_position].heading;
            //newNode.parent = parent;

            update(curNode, newNode, successors_are_fine, map);
        }
    }

    // when we are near goal point, we should try to reach it
    if (getCost(curNode.i, curNode.j, map.goal_i, map.goal_j) <= curNode->radius) {
        double angle = calcAngle(*curNode->parent, *curNode, *goal);

        if (fabs(angle * 180 / CN_PI_CONSTANT) <= angleLimit) {
            Node newNode = Node(map.goal_i,  map.goal_j, parent);
            newNode.angle = curNode->angle;
            newNode.radius = curNode->radius;

            update(curNode, newNode, successors_are_fine, map);
        }
    }
    return successors_are_fine;
}


//the main process of opening vertices and recalculating g- and rhs-values
bool DLian::ComputeShortestPath(Map &map)
{
    while (OPEN.top_key_less_than(CalculateKey(*goal)) || goal->rhs != goal->g) {
        ++number_of_steps;
        Node* current = OPEN.get(); //returns element from OPEN with smalest key value
        if (current->g > current->rhs) {
            current->g = current->rhs;

            //EXPAND FUNCTION
            expand(current, map);
            /*for (auto elem : GetSuccessors(current, map, opt)) { //for each successor(neighbor) recalculate it's rhs value
                if (elem->rhs > current->g + GetCost(elem->cell, current->cell)) {
                    elem->parent = current;
                    elem->rhs = current->g + GetCost(elem->cell, current->cell);
                }
                UpdateVertex(elem);
            }*/
        } else {
            current->g = std::numeric_limits<double>::infinity();
            std::vector<Node* > succ = GetSurroundings(current, map);
            succ.push_back(current);
            for (auto elem : succ) {
                if (!(elem->i == start->i && elem->j == start->j) && elem->parent == current) {

                   //RESET PARENT
                    Node min_val = GetMinPredecessor(elem, map, opt);
                    elem->rhs = min_val.rhs;
                    elem->parent = min_val.parent;
                }
                UpdateVertex(elem);
            }
        }
    }
    if (goal->g != std::numeric_limits<double>::infinity()) { //if path exists
        current_result.pathfound = true;
        current_result.numberofsteps = number_of_steps;
        current_result.nodescreated = NODES.size();
        current_result.pathlength = goal->g;
        MakePrimaryPath(goal); //build path from parent pointers
        current_result.lppath = &path;
        //map.PrintPath(path); //can use this function to build path in console
        return true;
    }
    return false;
}

//function returns "dummy" Node for current Node, that consists of parent - predecessor with the best(minimum)
//cost of getting from it to the current and rhs - this best(minimum) cost.
//when current node has no predecessors function returns parent=nullptr - which means that current vertex is now unreachable
Node DLian::GetMinPredecessor(Node* current, Map &map, EnvironmentOptions opt) {
    std::vector<Node *> all_neighbors = GetSuccessors(current, map, opt);
    Node dummy_current; //"dummy" for current, we will use only parent and rhs
    Node* neighbor; //neighbour of current node, we will be looking for neighbour with minimal cost from it to current
    if (!all_neighbors.empty()) {
        neighbor = all_neighbors.front();
        dummy_current.rhs = std::numeric_limits<double>::infinity(); //setting parameters for "dummy"
        dummy_current.parent = neighbor; //setting parameters for "dummy"
        for (auto n: all_neighbors) {
            if (dummy_current.rhs > n->g + GetCost(n->cell, current->cell)) {
                dummy_current.rhs = n->g + GetCost(n->cell, current->cell);
                dummy_current.parent = n;
            }
        }
    } else {
        dummy_current.parent = nullptr; //means that current vertex is now unreachable
    }
    return dummy_current;
}

//function returns Nodes of successors of current vertex. Already with their g- and rhs-values
std::vector<Node* > DLian::GetSuccessors(Node* current, Map &map, EnvironmentOptions opt) {
    std::vector<Node*> result;
    for(auto elem : FindNeighbors(current, map, opt)) {
        if(!NODES.count(vertex(elem, map.get_height()))) { //if vertex wasn't previously examined
            NODES[vertex(elem, map.get_height())] = Node(elem, current);
        }
        result.push_back(&(NODES.find(vertex(elem, map.get_height()))->second));
    }
    return result;
}

//function returns list of map neighbors to the current node depending on the environmental conditions
std::list<Cell> DLian::FindNeighbors(Node* n, Map &map, EnvironmentOptions opt) const {
    Cell new_cell;
    Cell curNode = n->cell;
    std::list<Cell> successors;
    for (int i = -1; i <= +1; i++) {
        for (int j = -1; j <= +1; j++) {
            if ((i != 0 || j != 0) && map.CellOnGrid(Cell(curNode.x + j, curNode.y + i)) &&
                    (map.CellIsTraversable(Cell(curNode.x + j, curNode.y + i)))) {
                if (i != 0 && j != 0) {
                    if (!opt.allowdiagonal)
                        continue;
                    else if (!opt.cutcorners) {
                        if (map.CellIsObstacle(Cell(curNode.x + j, curNode.y)) ||
                                map.CellIsObstacle(Cell(curNode.x, curNode.y + i)))
                            continue;
                    }
                    else if (!opt.allowsqueeze) {
                        if (map.CellIsObstacle(Cell( curNode.x + j, curNode.y)) &&
                                map.CellIsObstacle(Cell( curNode.x, curNode.y + i)))
                            continue;
                    }
                }
                new_cell = Cell(curNode.x + j, curNode.y + i);
                successors.push_front(new_cell);
            }
        }
    }
    return successors;
}

//function returns surroundings - neighbors of the current cell, but not depending on envinonmental options (cutcorners, allowsqueeze)
std::list<Node*> DLian::GetSurroundings(Node *current, Map &map, EnvironmentOptions opt) {
    std::list<Node> result1;
    int x = current->cell.x;
    int y = current->cell.y;
    if(opt.allowdiagonal) {
        for (int k = y - 1; k <= y + 1; ++k) {
            for (int l = x - 1; l <= x + 1; ++l) {
                if (!(k == y && l == x) && map.CellOnGrid(Cell(l, k)) && map.CellIsTraversable(Cell(l, k))) {
                    result1.push_front(Node(Cell(l, k)));
                }
            }
        }
    } else {
        for (int k = x - 1; k <= x + 1; ++k)
            if (k != x && map.CellOnGrid(Cell(k, y)) && map.CellIsTraversable(Cell(k, y)))
                result1.push_front(Node(Cell(k, y)));
        for (int l = y - 1; l <= y + 1; ++l)
            if (l != y && map.CellOnGrid(Cell(x, l)) && map.CellIsTraversable(Cell(x, l)))
                result1.push_front(Node(Cell(x, l)));
    }
    std::list<Node*> result;
    for(auto elem : result1) {
        if(!NODES.count(vertex(elem.cell, map.get_height()))) { //if vertex wasn't previously examined
            continue;
        } else {
            result.push_back(&(NODES.find(vertex(elem.cell, map.get_height()))->second));
        }
    }
    return result;
}

void DLian::MakePrimaryPath(Node *curNode) //build path by cells
{
    path.clear();
    Node current = *curNode;
    while (current.parent) {
        path.push_front(current);
        current = *current.parent;
    }
    path.push_front(current);
}

void DLian::makeSecondaryPath() //build path by sections
{
    std::list<Node>::const_iterator iter = path.begin();
    int curI, curJ, nextI, nextJ, moveI, moveJ;
    hpath.push_back(*iter);
    while (iter != --path.end()) {
        curI = iter->cell.y;
        curJ = iter->cell.x;
        ++iter;
        nextI = iter->cell.y;
        nextJ = iter->cell.x;
        moveI = nextI - curI;
        moveJ = nextJ - curJ;
        ++iter;
        if ((iter->cell.y - nextI) != moveI || (iter->cell.x - nextJ) != moveJ)
            hpath.push_back(*(--iter));
        else
            --iter;
    }
}
