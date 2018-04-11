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

double calcAngle(const Node &dad, const Node &node, const Node &son) const {
    double cos_angle = (node.j - dad.j) * (son.j - node.j) +
                       (node.i - dad.i) * (son.i - node.i);
    cos_angle /= getCost(son.i, son.j, node.i, node.j);
    cos_angle /= getCost(node.i, node.j, dad.i, dad.j);

    if (cos_angle < -1) cos_angle = -1;
    if (cos_angle > 1) cos_angle = 1;

    return acos(cos_angle);
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

std::list<Node*> DLian::getAllNodes(Node current_node, int width) {
    std::list<Node*> all;
    auto it = NODES.find(current_node.convolution(width));
    if (it != NODES.end()) {
        auto range = NODES.equal_range(it->first);
        for (auto it = range.first; it != range.second; ++it)
            all.push_back(&(it->second));
    }
    return all;
}

void DLian::Initialize(Map &map)
{
    Node start_node = Node(map.start_i, map.start_j);
    start_node.radius = distance;
    Node goal_node = Node(map.goal_i, map.goal_j);
    goal_node.radius = distance;

    NODES.insert({goal_node.convolution(map.get_width()), goal_node});
    goal = getFromNodes(goal_node, map.get_width());

    start_node.rhs = 0;
    start_node.key = CalculateKey(start_node);
    NODES.insert({start_node.convolution(map.get_width()), start_node});
    start = getFromNodes(start_node, map.get_width());

    OPEN.resize(map.get_height());
    OPEN.put(start); //add start cell to OPEN list

    current_result.goal_became_unreachable = false;

    calculateCircle((int) distance);
    calculatePivotCircle();
}


//The main pathbuilding function
SearchResult DLian::FindThePath(Map &map)
{
    std::chrono::time_point<std::chrono::system_clock> startt, end;
    startt = std::chrono::system_clock::now();
    number_of_steps = 0;
    Initialize(map); //algorithm initialization

    if(!ComputeShortestPath(map)) {
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
            for (auto neighbor: GetSurroundings(d, map)) {
                if (!(neighbor->cell == start->cell) && (neighbor->parent->cell == dam || CutOrSqueeze(neighbor, d))) {
                    Node min_val = GetMinPredecessor(neighbor, map);
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
                        UpdateVertex(neighbor);
                    }
                }
            }
        }
    }
    if(!ComputeShortestPath(map)) {
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
       Node min_val = GetMinPredecessor(cl, map);
       if (min_val.parent) { //means that neighbor is reachable
           cl->rhs = min_val.rhs;
           cl->parent = min_val.parent;
           cl->g = min_val.parent->g + GetCost(cl->cell, min_val.parent->cell);
           UpdateVertex(cl);
       } else {
           break; //means that this cell is unreachable and there's no need to recount values for it's neighbors
       }
       for (auto neighbor : GetSuccessors(cl, map)) {
           if (neighbor->rhs > cl->g + GetCost(neighbor->cell, cl->cell)) {
               neighbor->parent = cl;
               neighbor->rhs = cl->g + GetCost(neighbor->cell, cl->cell);
               UpdateVertex(neighbor);
           }
           if (neighbor->cell.x == cl->cell.x || neighbor->cell.y == cl->cell.y) {
               Node min_val = GetMinPredecessor(neighbor, map);
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
                   UpdateVertex(neighbor);
               }
           }
       }
    }
    if(!ComputeShortestPath(map)){
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

    UpdateVertex(node);
    successors = true;
}

bool DLian::expand(const Node* curNode, const Map &map) {
    bool successors_are_fine = false;
    auto parent = getFromNodes(curNode, map.get_width(), curNode->parent);
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
                newNode.rhs = curNode->g + getCost(newNode.i, newNode.j, curNode->i, curNode->j);
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
                    newNode.rhs = curNode->g + getCost(newNode.i, newNode.j, curNode->i, curNode->j);
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
            newNode.rhs = curNode->g + getCost(newNode.i, newNode.j, curNode->i, curNode->j);
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
            std::vector<Node* > succ = GetSuccessors(current, map);
            succ.push_back(current);
            for (auto elem : succ) {

                if (!(elem->i == start->i && elem->j == start->j) &&
                    !(elem->parent->i == start->i && elem->parent->j == start->j) && elem->parent == current) {

                   //RESET PARENT
                    ResetParent(elem, current, map);
                }
                UpdateVertex(elem);
            }
        }
    }
    current_result.nodescreated = NODES.size();
    current_result.numberofsteps = number_of_steps;
    if (goal->g != std::numeric_limits<double>::infinity()) { //if path exists
        current_result.pathfound = true;
        current_result.pathlength = goal->g;
        makePrimaryPath(goal);
        if (postsmoother) hppath = smoothPath(hppath, map);
        current_result.hppath = hppath;
        makeSecondaryPath();
        current_result.lppath = lppath;
        current_result.max_angle = makeAngles();
        current_result.angles = angles;
        current_result.sections = hppath.size() - 1;
        return true;
    }
    return false;
}


void DLian::ResetParent(Node* current, Node* parent, const LocalMap &map) {
    bool parent_found = false;
    int node_straight_behind = circle_nodes.size() - (int)round(parent->angle * circle_nodes.size() / 360) % circle_nodes.size();
    double angle = 360 - fabs(current->angle - circle_nodes[node_straight_behind].heading);
    if ((angle <= 180 && angle <= angleLimit) || (angle > 180 && 360 - angle <= angleLimit)) {
        int new_pos_i = current->i + circle_nodes[node_straight_behind].i;
        int new_pos_j = current->j + circle_nodes[node_straight_behind].j;
        if (map.CellOnGrid(new_pos_i, new_pos_j) && map.CellIsTraversable(new_pos_i, new_pos_j)) {

            for (auto node : getAllNodes(Node(new_pos_i, new_pos_j), map.get_width())) {
                double angle_prev = fabs(parent->angle - node->angle);
                if ((angle_prev <= 180 && angle_prev <= angleLimit) || (angle_prev > 180 && 360 - angle_prev <= angleLimit)) {
                    if (!checkLineSegment(map, *node, *parent)) continue;
                    if (parent->rhs > node->g + getCost(node->i, node->j, parent->i, parent->j)) {
                        parent_found = true;
                        parent->parent = node;
                        parent->rhs = node->g + getCost(node->i, node->j, parent->i, parent->j);
                    }
                }
            }
         }
    }

    std::vector<int> candidates = std::vector<int>{node_straight_behind, node_straight_behind};
    bool limit1 = true;
    bool limit2 = true;
    while (++candidates[0] != --candidates[1] && (limit1 || limit2)) { // untill the whole circle is explored or we exessed anglelimit somewhere
        if (candidates[0] >= circle_nodes.size()) candidates[0] = 0;
        if (candidates[1] < 0) candidates[1] = circle_nodes.size() - 1;

        for (auto cand : candidates) {
            double angle = 360 - fabs(current->angle - circle_nodes[cand].heading);
            if ((angle <= 180 && angle <= angleLimit) || (angle > 180 && 360 - angle <= angleLimit)) {
                int new_pos_i = parent->i + circle_nodes[cand].i;
                int new_pos_j = parent->j + circle_nodes[cand].j;

                if (!map.CellOnGrid(new_pos_i, new_pos_j)) continue;
                if (map.CellIsObstacle(new_pos_i, new_pos_j)) continue;

                for (auto node : getAllNodes(Node(new_pos_i, new_pos_j), map.get_width())) {
                    double angle_prev = fabs(parent->angle - node->angle);
                    if ((angle_prev <= 180 && angle_prev <= angleLimit) || (angle_prev > 180 && 360 - angle_prev <= angleLimit)) {
                        if (!checkLineSegment(map, *node, *parent)) continue;
                        if (parent->rhs > node->g + getCost(node->i, node->j, parent->i, parent->j)) {
                            parent_found = true;
                            parent->parent = node;
                            parent->rhs = node->g + getCost(node->i, node->j, parent->i, parent->j);
                        }
                    }
                }
            }
        }
    }
    if (!parent_found) {
        parent->rhs = std::numeric_limits<float>::infinity();
        parent->parent = nullptr;
    }
    current->rhs = parent->rhs + getCost(current->i, current->j, parent->i, parent->j);
}

//function returns Nodes of successors of current vertex. Already with their g- and rhs-values
std::vector<Node* > DLian::GetSuccessors(Node* current, Map &map) {
    std::vector<Node*> result;
    if (current->parent != nullptr) {
        int node_straight_ahead = (int)round(current->angle * circle_nodes.size() / 360) % circle_nodes.size();
        double angle = fabs(current->angle - circle_nodes[node_straight_ahead].heading);
        if ((angle <= 180 && angle <= angleLimit) || (angle > 180 && 360 - angle <= angleLimit)) {
            int new_pos_i = current->i + circle_nodes[node_straight_ahead].i;
            int new_pos_j = current->j + circle_nodes[node_straight_ahead].j;
            if (map.CellOnGrid(new_pos_i, new_pos_j) && map.CellIsTraversable(new_pos_i, new_pos_j)) {
                Node* succ = getFromNodes(Node(new_pos_i, new_pos_j), map.get_width(), current);
                if (succ) result.push_back(succ);
             }
        }

        std::vector<int> candidates = std::vector<int>{node_straight_ahead, node_straight_ahead};
        bool limit1 = true;
        bool limit2 = true;
        while (++candidates[0] != --candidates[1] && (limit1 || limit2)) { // untill the whole circle is explored or we exessed anglelimit somewhere
            if (candidates[0] >= circle_nodes.size()) candidates[0] = 0;
            if (candidates[1] < 0) candidates[1] = circle_nodes.size() - 1;

            for (auto cand : candidates) {
                double angle = fabs(current->angle - circle_nodes[cand].heading);
                if ((angle <= 180 && angle <= angleLimit) || (angle > 180 && 360 - angle <= angleLimit)) {
                    int new_pos_i = current->i + circle_nodes[cand].i;
                    int new_pos_j = current->j + circle_nodes[cand].j;

                    if (!map.CellOnGrid(new_pos_i, new_pos_j)) continue;
                    if (map.CellIsObstacle(new_pos_i, new_pos_j)) continue;

                    Node* succ = getFromNodes(Node(new_pos_i, new_pos_j), map.get_width(), current);
                    if (succ) result.push_back(succ);
                } else {
                    if (cand == candidates[0]) limit1 = false;
                    else limit2 = false;
                }
            }
        }
    } else { // when we do not have parent, we should explore all neighbors
        int angle_position(-1), new_pos_i, new_pos_j;
        for (auto node : circle_nodes) {
            new_pos_i = current->i + node.i;
            new_pos_j = current->j + node.j;
            angle_position++;

            if (!map.CellOnGrid(new_pos_i, new_pos_j)) continue;
            if (map.CellIsObstacle(new_pos_i, new_pos_j)) continue;

            Node* succ = getFromNodes(Node(new_pos_i, new_pos_j), map.get_width(), current);
            if (succ) result.push_back(succ);
        }
    }

    // when we are near goal point, we should try to reach it
    if (getCost(current.i, current.j, map.goal_i, map.goal_j) <= current->radius) {
        double angle = calcAngle(*current->parent, *current, *goal);

        if (fabs(angle * 180 / CN_PI_CONSTANT) <= angleLimit) {
            Node* succ = getFromNodes(Node(new_pos_i, new_pos_j), map.get_width(), current);
            if (succ) result.push_back(succ);
        }
    }
    return result;
}

void DLian::makePrimaryPath(Node* curNode) {
    hppath.push_front(*curNode);
    curNode = *curNode->parent;
    do {
        hppath.push_front(curNode);
        curNode = *curNode->parent;

    } while (curNode->parent != nullptr);
    hppath.push_front(*curNode);
}

bool DLian::checkAngle(const Node &dad, const Node &node, const Node &son) const {
    double angle = calcAngle(dad, node, son) * 180 /  CN_PI_CONSTANT;
    if (fabs(angle) <= angleLimit) {
        return true;
    }
    return false;
}


std::list<Node> DLian::smoothPath(const std::list<Node>& path, const Map& map) {
    std::list<Node> new_path;
    sresult.pathlength = 0;
    auto it = path.begin();
    auto curr_it = path.begin();
    Node start_section = path.front();
    Node end_section = path.front();
    bool first = true;
    Node previous = *it++;
    while (end_section != path.back()) {
        for (it; it != path.end(); ++it) {
            auto next = ++it;
            --it;
            if (!first && !checkAngle(previous, start_section, *it)) continue;
            if ((next != path.end() && checkAngle(start_section, *it, *next) ||
                 next == path.end()) && checkLineSegment(map, start_section, *it)) {
                end_section = *it;
                curr_it = it;
            }
        }
        sresult.pathlength += (double)getCost(previous.i, previous.j, start_section.i, start_section.j);
        new_path.push_back(start_section);
        previous = start_section;
        first = false;
        start_section = end_section;
        it = ++curr_it;
    }
    sresult.pathlength += (double)getCost(previous.i, previous.j, end_section.i, end_section.j);
    new_path.push_back(end_section);
    return new_path;
}

void DLian::makeSecondaryPath() {
    std::vector<Node> lineSegment;
    auto it = hppath.begin();
    Node parent = *it++;
    while (it != hppath.end()) {
        calculateLineSegment(lineSegment, parent, *it);
        std::reverse(std::begin(lineSegment), std::end(lineSegment));
        lppath.insert(lppath.begin(), ++lineSegment.begin(), lineSegment.end());
        parent = *it++;
    }
    lppath.push_front(hppath.back());
    std::reverse(std::begin(lppath), std::end(lppath));
}


double DLian::makeAngles() {
    angles.clear();
    double max_angle = 0;
    sresult.accum_angle = 0;
    auto pred = hppath.begin();
    auto current = ++hppath.begin();
    auto succ = ++(++hppath.begin());

    while(succ != hppath.end()) {
        double angle = calcAngle(*pred++, *current++, *succ++);
        angle = angle * 180 / CN_PI_CONSTANT;
        if (angle > max_angle) max_angle = angle;
        sresult.accum_angle += angle;
        angles.push_back(angle);
    }
    std::reverse(std::begin(angles), std::end(angles));
    return max_angle;
}
