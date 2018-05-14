#include "dlian.h"

DLian::DLian() {}
DLian::DLian(float angleLimit_, int distance_, float hweight_, bool postsmoother_) {
    this->angleLimit = angleLimit_;
    this->distance = distance_;
    this->hweight = hweight_;
    this->postsmoother = postsmoother_;
}

DLian::~DLian()
{
    if (!NODES.empty()) NODES.erase(NODES.begin(), NODES.end());
}

float getCost(int a_i, int a_j, int b_i, int b_j) {
    return sqrt(abs(a_i - b_i) * abs(a_i - b_i) + abs(a_j - b_j) * abs(a_j - b_j));
}

Key DLian::CalculateKey(Node& vertex) {
    Key res(std::min(vertex.g, vertex.rhs + hweight * getCost(vertex.i, vertex.j, goal->i, goal->j)), std::min(vertex.g, vertex.rhs));
    return res;
}

void DLian::calculateCircle(int radius) { //here radius - radius of the circle in cells
    circle_nodes.clear();
    std::vector<circleNode> circle(0);
    int x = 0;
    int y = radius;
    int delta = 2 - 2 * radius;
    int error = 0;
    while (y >= 0) {
        if(x > radius) x = radius;
        else if(x < -radius) x = -radius;
        if(y > radius) y = radius;
        else if(y < -radius) y = -radius;
        double dist = getCost(0,0,x,y);
        circle.push_back(circleNode(x, y, dist));
        circle.push_back(circleNode(x, -y, dist));
        circle.push_back(circleNode(-x, y, dist));
        circle.push_back(circleNode(-x, -y, dist));

        error = 2 * (delta + y) - 1;
        if (delta < 0 && error <= 0) {
            delta += 2 * ++x + 1;
            continue;
        }

        error = 2 * (delta - x) - 1;
        if (delta > 0 && error > 0) {
            delta += 1 - 2 * --y;
            continue;
        }
        delta += 2 * (++x - y--);
    }

    for (int i = 0; i < circle.size(); i += 4)
        circle_nodes.push_back(circle[i]);
    for (int i = circle.size() - 7; i > 0; i -= 4)
        circle_nodes.push_back(circle[i]);
    for (int i = 7; i < circle.size(); i += 4)
        circle_nodes.push_back(circle[i]);
    for (int i = circle.size() - 6; i > 0; i -= 4)
        circle_nodes.push_back(circle[i]);
    circle_nodes.pop_back();
    for (size_t i = 0; i < circle_nodes.size(); ++i) {
        double angle = acos((circle_nodes[0].i * circle_nodes[i].i + circle_nodes[0].j * circle_nodes[i].j)
                       / (sqrt(pow(circle_nodes[0].i, 2) + pow(circle_nodes[0].j, 2))
                       * sqrt(pow(circle_nodes[i].i, 2) + pow(circle_nodes[i].j, 2))));
        if(i < circle_nodes.size() / 2)
            circle_nodes[i].heading = angle * 180 / CN_PI_CONSTANT;
        else
            circle_nodes[i].heading = 360 - angle * 180 / CN_PI_CONSTANT;
         //std::cout << circleNodes[k][i].heading  << std::endl;
    }
}

double calcAngle(const Node &dad, const Node &node, const Node &son) {
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
    Node* found = nullptr;
    if (it != NODES.end()) {
        auto range = NODES.equal_range(it->first);
        for (auto it = range.first; it != range.second; ++it) {
            if (it->second.parent == nullptr) {
                found =  &(it->second);
                continue;
            }
            if (it->second.parent->i == parent->i && it->second.parent->j == parent->j) {
                return &(it->second);
            }
        }
    }
    return found;
}

std::vector<Node*> DLian::getAllNodes(Node current_node, int width) {
    std::vector<Node*> all;
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

    calculateCircle((int) distance);
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
        end = std::chrono::system_clock::now();
        current_result.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - startt).count()) / 1000000000;
        std::cout << "THE PATH DOES NOT EXIST ON THE INITIAL MAP\n";
        return current_result;
    }
    for (auto elem : hppath)
        std::cout << elem << "->";
    std::cout << std::endl;

    Changes changes = map.DamageTheMap(lppath); //force map to change (sufficient for the correct testing)
    //map.PrintMap();
    for (auto dam : changes.occupied) { //for each damaged (0 -> 1) cell recounting values for it's neighbors
        OPEN.remove_all(dam);
    }
    auto cmp = [](Node* a, Node* b) { return *a < *b; };
    std::set<Node*, decltype(cmp)> surr(cmp);
    //std::vector<Node*> surr;
    for (auto dam : changes.occupied) {
        std::vector<Node*> new_ = GetSurroundings(dam, map);
        surr.insert(new_.begin(), new_.end());
    }
    for (auto elem : surr) {
        std::cout << *elem << *elem->parent << ' ';
        if (elem->parent->parent) std::cout << *elem->parent->parent << std::endl;
        else std::cout << std::endl;
        ResetParent(elem, elem->parent, map);
        if (elem->parent != nullptr) UpdateVertex(elem);
    }

    if(!ComputeShortestPath(map)) {
        current_result.pathfound = false;
        end = std::chrono::system_clock::now();
        current_result.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - startt).count()) / 1000000000;
        std::cout << "AFTER THE FIRST MAP CHANGE THE PATH DOES NOT EXIST\n";
        return current_result;
    }
    for (auto elem : hppath)
        std::cout << elem << "->";
    std::cout << std::endl;
    end = std::chrono::system_clock::now();
    current_result.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - startt).count()) / 1000000000;
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

void DLian::update(Node* current_node, Node new_node, bool &successors, const Map &map) {
    if (!checkLineSegment(map, *current_node, new_node)) return;

    Node* node = getFromNodes(new_node, map.get_width(), current_node);
    if (node == nullptr || node->parent == nullptr) {
        NODES.insert({new_node.convolution(map.get_width()), new_node});
        node = getFromNodes(new_node, map.get_width(), current_node);
    }

    UpdateVertex(node);
    successors = true;
}

bool DLian::expand(Node* curNode, const Map &map) {
    bool successors_are_fine = false;
    auto parent = getFromNodes(*curNode, map.get_width(), curNode->parent);
    if (curNode->parent != nullptr) {
        int node_straight_ahead = (int)round(curNode->angle * circle_nodes.size() / 360) % circle_nodes.size();
        double angle = fabs(curNode->angle - circle_nodes[node_straight_ahead].heading);
        if ((angle <= 180 && angle <= angleLimit) || (angle > 180 && 360 - angle <= angleLimit)) {
            int new_pos_i = curNode->i + circle_nodes[node_straight_ahead].i;
            int new_pos_j = curNode->j + circle_nodes[node_straight_ahead].j;
            if (map.CellOnGrid(new_pos_i, new_pos_j) && map.CellIsTraversable(new_pos_i, new_pos_j)) {
                Node newNode = Node(new_pos_i, new_pos_j, parent);
                newNode.angle = circle_nodes[node_straight_ahead].heading;
                newNode.radius = curNode->radius;
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
            newNode.radius = curNode->radius;
            newNode.angle = circle_nodes[angle_position].heading;
            //newNode.parent = parent;
            newNode.rhs = curNode->g + getCost(newNode.i, newNode.j, curNode->i, curNode->j);
            update(curNode, newNode, successors_are_fine, map);
        }
    }

    // when we are near goal point, we should try to reach it
    if (getCost(curNode->i, curNode->j, map.goal_i, map.goal_j) <= curNode->radius) {
        double angle = calcAngle(*curNode->parent, *curNode, *goal);

        if (fabs(angle * 180 / CN_PI_CONSTANT) <= angleLimit) {
            goal->parent = parent;
            goal->angle = curNode->angle;
            goal->radius = curNode->radius;
            goal->rhs = curNode->g + getCost(map.goal_i, map.goal_j, curNode->i, curNode->j);
            goal->g = goal->rhs;
            //update(curNode, *goal, successors_are_fine, map);
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
            //OPEN.print_elements();
            current->g = std::numeric_limits<double>::infinity();
            std::vector<Node* > succ = GetSuccessors(current, map);
            succ.push_back(current);
            for (auto elem : succ) {
                if (elem->parent == nullptr) continue;
                if (!(elem->i == start->i && elem->j == start->j) &&
                    !(elem->parent->i == start->i && elem->parent->j == start->j) &&
                      elem->parent->i == current->i && elem->parent->j == current->j) {

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
        //map.PrintPath(lppath);
        current_result.lppath = lppath;
        current_result.max_angle = makeAngles();
        current_result.angles = angles;
        current_result.sections = hppath.size() - 1;
        return true;
    }
    return false;
}


void DLian::ResetParent(Node* current, Node* parent, const Map &map) {
    if (parent->old_parent != nullptr) parent->old_parent = parent->parent;
    bool parent_found = false;
    Node new_parent;
    int node_straight_behind = (int)(circle_nodes.size() / 2) - (int)round(current->angle * circle_nodes.size() / 360) % circle_nodes.size();
    double angle = fabs(current->angle - fabs(180 - circle_nodes[node_straight_behind].heading));
    if ((angle <= 180 && angle <= angleLimit) || (angle > 180 && 360 - angle <= angleLimit)) {
        int new_pos_i = parent->i + circle_nodes[node_straight_behind].i;
        int new_pos_j = parent->j + circle_nodes[node_straight_behind].j;
        if (map.CellOnGrid(new_pos_i, new_pos_j) && map.CellIsTraversable(new_pos_i, new_pos_j)) {

            for (auto node : getAllNodes(Node(new_pos_i, new_pos_j), map.get_width())) {
                double angle_prev = fabs(fabs(180 - circle_nodes[node_straight_behind].heading) - node->angle);
                if ((angle_prev <= 180 && angle_prev <= angleLimit) || (angle_prev > 180 && 360 - angle_prev <= angleLimit)) {
                    if (!checkLineSegment(map, *node, *parent)) continue;
                    if (parent->rhs > node->rhs + getCost(node->i, node->j, parent->i, parent->j)) {
                        parent_found = true;
                        new_parent.parent = node;
                        new_parent.angle = 180 - circle_nodes[node_straight_behind].heading;
                        new_parent.rhs = node->rhs + getCost(node->i, node->j, parent->i, parent->j);
                    }
                }
            }
         }
    }

    std::vector<int> candidates = std::vector<int>{node_straight_behind, node_straight_behind};
    bool limit1 = true;
    bool limit2 = true;
    while (limit1 || limit2) { // untill the whole circle is explored or we exessed anglelimit somewhere
        if (candidates[0] >= circle_nodes.size()) candidates[0] = 0;
        if (candidates[1] < 0) candidates[1] = circle_nodes.size() - 1;
        if (candidates[0] == candidates[1]) break;

        for (auto cand : candidates) {
            double angle = fabs(current->angle - fabs(180 - circle_nodes[cand].heading));
            if ((angle <= 180 && angle <= angleLimit) || (angle > 180 && 360 - angle <= angleLimit)) {
                int new_pos_i = parent->i + circle_nodes[cand].i;
                int new_pos_j = parent->j + circle_nodes[cand].j;

                if (!map.CellOnGrid(new_pos_i, new_pos_j)) continue;
                if (map.CellIsObstacle(new_pos_i, new_pos_j)) continue;

                for (auto node : getAllNodes(Node(new_pos_i, new_pos_j), map.get_width())) {
                    double angle_prev = fabs(fabs(180 - circle_nodes[cand].heading) - node->angle);
                    if ((angle_prev <= 180 && angle_prev <= angleLimit) || (angle_prev > 180 && 360 - angle_prev <= angleLimit)) {
                        if (!checkLineSegment(map, *node, *parent)) continue;
                        if (parent->rhs > node->rhs + getCost(node->i, node->j, parent->i, parent->j)) {
                            parent_found = true;

                            new_parent.parent = node;
                            new_parent.angle = 180 - circle_nodes[node_straight_behind].heading;
                            new_parent.rhs = node->rhs + getCost(node->i, node->j, parent->i, parent->j);
                        }
                    }
                }
            } else {
                if (cand == candidates[0]) limit1 = false;
                if (cand == candidates[1]) limit2 = false;
            }
        }
    }
    if (!parent_found) {
        parent->rhs = std::numeric_limits<float>::infinity();
        parent->parent = nullptr;
    } else {
        Node new_node = *parent;
        new_node.parent = new_parent.parent;
        new_node.angle = new_parent.angle;
        new_node.rhs = new_parent.rhs;
        Node* node = getFromNodes(new_node, map.get_width(), new_parent.parent);
        if (node == nullptr || node->parent == nullptr) {
            NODES.insert({parent->convolution(map.get_width()), new_node});
            node = getFromNodes(new_node, map.get_width(), new_parent.parent);
        }
        current->parent = node;
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
    if (getCost(current->i, current->j, map.goal_i, map.goal_j) <= current->radius) {
        double angle = calcAngle(*current->parent, *current, *goal);

        if (fabs(angle * 180 / CN_PI_CONSTANT) <= angleLimit) {
            goal->parent = current;
            Node* succ = getFromNodes(Node(map.goal_i, map.goal_j), map.get_width(), current);
            if (succ) result.push_back(succ);
        }
    }
    return result;
}

std::vector<Node *> DLian::GetSurroundings(Node current, Map &map) {
    std::vector<Node *> surr;
    for (int i = std::max(0, current.i - 2 * distance); i <= current.i + 2 * distance; ++i) {
        for (int j = std::max(0, current.j - 2 * distance); j <= current.j + 2 * distance; ++j) {
            if (!map.CellOnGrid(i, j) || map.CellIsObstacle(i, j)) continue;
            for (auto elem : getAllNodes(Node(i, j), map.get_width())) {
                if (elem->parent == nullptr) continue;
                if (elem->parent->parent == nullptr) continue;
                if (map.CellIsObstacle(elem->parent->i, elem->parent->j)) {
                    OPEN.remove_if(elem);
                    continue;
                }
                if (checkLineSegment(map, *elem->parent, *elem->parent->parent) &&
                    map.CellIsTraversable(elem->parent->parent->i, elem->parent->parent->j)) continue;
                surr.push_back(elem);
            }
        }
    }
    return surr;
}


void DLian::makePrimaryPath(Node* curNode) {
    hppath.clear();
    hppath.push_front(*curNode);
    curNode = curNode->parent;
    do {
        hppath.push_front(*curNode);
        curNode = curNode->parent;

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
    current_result.pathlength = 0;
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
        current_result.pathlength += (double)getCost(previous.i, previous.j, start_section.i, start_section.j);
        new_path.push_back(start_section);
        previous = start_section;
        first = false;
        start_section = end_section;
        it = ++curr_it;
    }
    current_result.pathlength += (double)getCost(previous.i, previous.j, end_section.i, end_section.j);
    new_path.push_back(end_section);
    return new_path;
}

void DLian::makeSecondaryPath() {
    lppath.clear();
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
    current_result.accum_angle = 0;
    auto pred = hppath.begin();
    auto current = ++hppath.begin();
    auto succ = ++(++hppath.begin());

    while(succ != hppath.end()) {
        double angle = calcAngle(*pred++, *current++, *succ++);
        angle = angle * 180 / CN_PI_CONSTANT;
        if (angle > max_angle) max_angle = angle;
        current_result.accum_angle += angle;
        angles.push_back(angle);
    }
    std::reverse(std::begin(angles), std::end(angles));
    return max_angle;
}
