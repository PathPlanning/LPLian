#include "DLian.h"

DLian::DLian() {}
DLian::DLian(float angleLimit_, int distance_, float weight_,
             unsigned int steplimit_, float curvatureHeuristicWeight_, bool postsmoother_,
             float decreaseDistanceFactor_, int distanceMin_,
             double PivotRadius_, int numOfParentsToIncreaseRadius_) {
    this->angleLimit = angleLimit_;
    this->distance = distance_;
    this->weight = weight_;
    this->stepLimit = steplimit_;
    this->curvatureHeuristicWeight = curvatureHeuristicWeight_;
    this->postsmoother = postsmoother_;
    this->decreaseDistanceFactor = decreaseDistanceFactor_;
    this->distanceMin = distanceMin_;
    this->pivotRadius = PivotRadius_;
    this->numOfParentsToIncreaseRadius = numOfParentsToIncreaseRadius_;
    closeSize = 0;
    srand(time(NULL));
}

DLian::~DLian()
{
    if (!NODES.empty()) NODES.erase(NODES.begin(), NODES.end());
}

/* inline int vertex(Cell item, int size) {
    return item.x * size + item.y;
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
}*/

Key DLian::CalculateKey(Node& vertex, int metrics) {
    Key res(std::min(vertex.g, vertex.rhs + hweight * heuristic(vertex.cell, goal->cell, metrics)), std::min(vertex.g, vertex.rhs));
    return res;
}

void DLian::calculateCircle(int radius) { //here radius - radius of the circle in cells
    circleNodes.clear();
    circleNodes.resize(listOfDistancesSize);
    for (int k = 0; k < listOfDistancesSize; ++k) {
        radius = listOfDistances[k];
        circleNodes[k].clear();
        std::vector<circleNode> circle_nodes(0);
        int x = 0;
        int y = radius;
        int delta = 2 - 2 * radius;
        int error = 0;
        while (y >= 0) {
            if(x > radius) x = radius;
            else if(x < -radius) x = -radius;
            if(y > radius) y = radius;
            else if(y < -radius) y = -radius;
            double dist(getCost(0,0,x,y));
            circle_nodes.push_back(circleNode(x, y, dist));
            circle_nodes.push_back(circleNode(x, -y, dist));
            circle_nodes.push_back(circleNode(-x, y, dist));
            circle_nodes.push_back(circleNode(-x, -y, dist));

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

        for (int i = 0; i < circle_nodes.size(); i += 4)
            circleNodes[k].push_back(circle_nodes[i]);
        for (int i = circle_nodes.size() - 7; i > 0; i -= 4)
            circleNodes[k].push_back(circle_nodes[i]);
        for (int i = 7; i < circle_nodes.size(); i += 4)
            circleNodes[k].push_back(circle_nodes[i]);
        for (int i = circle_nodes.size() - 6; i > 0; i -= 4)
            circleNodes[k].push_back(circle_nodes[i]);
        circleNodes[k].pop_back();
        for (size_t i = 0; i < circleNodes[k].size(); ++i) {
            double angle = acos((circleNodes[k][0].i * circleNodes[k][i].i + circleNodes[k][0].j * circleNodes[k][i].j)
                           / (sqrt(pow(circleNodes[k][0].i, 2) + pow(circleNodes[k][0].j, 2))
                           * sqrt(pow(circleNodes[k][i].i, 2) + pow(circleNodes[k][i].j, 2))));
            if(i<circleNodes[k].size()/2)
                circleNodes[k][i].heading = angle * 180 / CN_PI_CONSTANT;
            else
                circleNodes[k][i].heading = 360 - angle * 180 / CN_PI_CONSTANT;
        }
    }
}

void DLian::calculatePivotCircle() {
    pivotCircle.clear();
    int add_i, add_j, num(pivotRadius + 0.5 - CN_EPSILON);
    for (int i = -num; i <= +num; i++) {
        for (int j = -num; j <= +num; j++) {
            add_i = i != 0 ? 1 : 0;
            add_j = j != 0 ? 1 : 0;
            if ((pow(2 * abs(i) - add_i, 2) + pow(2 * abs(j) - add_j, 2)) < pow(2 * pivotRadius, 2))
                pivotCircle.push_back({i, j});
        }
    }
    if (pivotCircle.empty())
        pivotCircle.push_back({0, 0});
}

bool DLian::checkPivotCircle(const Map &map, const Node &center) {
    int i, j;
    for (int k = 0; k < pivotCircle.size(); k++) {
        i = center.i + pivotCircle[k].first;
        j = center.j + pivotCircle[k].second;
        if (!map.CellOnGrid(i, j) || map.CellIsObstacle(i,j))
            return false;
    }
    return true;
}

void DLian::calculateDistances() {
    int curDistance = distance;
    if(decreaseDistanceFactor > 1) {
        while(curDistance >= distanceMin) {
            listOfDistances.push_back(curDistance);
            curDistance = ceil(curDistance / decreaseDistanceFactor);
        }
    } else {
        listOfDistances.push_back(curDistance);
    }
    listOfDistancesSize = listOfDistances.size();
}

void DLian::calculateLineSegment(std::vector<Node> &line, const Node &start, const Node &goal) {
    int x1 = start.i;
    int x2 = goal.i;
    int y1 = start.j;
    int y2 = goal.j;

    int x,y;
    int dx, dy;
    int StepVal = 0;
    int Rotate = 0;

    line.clear();

    if (x1 > x2 && y1 > y2) {
        std::swap(x1, x2);
        std::swap(y1, y2);

        dx = x2 - x1;
        dy = y2 - y1;
    } else {
        dx = x2 - x1;
        dy = y2 - y1;

        if (dx >= 0 && dy >= 0) Rotate = 2;
        else if (dy < 0) {
            dy = -dy;
            std::swap(y1, y2);

            Rotate = 1;
        } else if (dx < 0) {
            dx = -dx;
            std::swap(x1, x2);

            Rotate = 3;
        }
    }

    if (Rotate == 1) {
        if(dx >= dy) {
            for(x = x1; x <= x2; ++x) {
                line.push_back(Node(x, y2));
                StepVal += dy;
                if(StepVal >= dx) {
                    --y2;
                    StepVal -= dx;
                }
            }
        } else {
            for(y = y1; y <= y2; ++y) {
                line.insert(line.begin(),Node(x2, y));
                StepVal += dx;
                if(StepVal >= dy) {
                    --x2;
                    StepVal -= dy;
                }
            }
        }
        return;
    } else if(Rotate == 2) {
        if(dx >= dy) {
            for(x = x1; x <= x2; ++x) {
                line.push_back(Node(x, y1));
                StepVal += dy;
                if(StepVal >= dx) {
                    ++y1;
                    StepVal -= dx;
                }
            }
            return;
        } else {
            for(y = y1; y <= y2; ++y) {
                line.push_back(Node(x1, y));
                StepVal += dx;
                if(StepVal >= dy) {
                    ++x1;
                    StepVal -= dy;
                }
            }
            return;
        }
    } else if (Rotate == 3) {
        if(dx >= dy) {
            for(x = x1; x <= x2; ++x) {
                line.insert(line.begin(),Node(x, y2));
                StepVal += dy;
                if(StepVal >= dx){
                    --y2;
                    StepVal -= dx;
                }
            }
        } else {
            for(y = y1; y <= y2; ++y) {
                line.push_back(Node(x2, y));
                StepVal += dx;
                if(StepVal >= dy) {
                    --x2;
                    StepVal -= dy;
                }
            }
        }
        return;
    }

    if(dx >= dy) {
        for(x = x1; x <= x2; ++x) {
            line.insert(line.begin(),Node(x, y1));
            StepVal += dy;
            if(StepVal >= dx){
                ++y1;
                StepVal -= dx;
            }
        }
    } else {
        for(y = y1; y <= y2; ++y) {
            line.insert(line.begin(),Node(x1, y));
            StepVal += dx;
            if(StepVal >= dy) {
                ++x1;
                StepVal -= dy;
            }
        }
    }
}

bool DLian::checkLineSegment(const Map &map, const Node &start, const Node &goal) {
    int x1 = start.i;
    int x2 = goal.i;
    int y1 = start.j;
    int y2 = goal.j;

    int x,y;
    int dx, dy;
    int StepVal = 0;
    int Rotate = 0;

    if (x1 > x2 && y1 > y2) {
        std::swap(x1, x2);
        std::swap(y1, y2);

        dx = x2 - x1;
        dy = y2 - y1;
    } else {
        dx = x2 - x1;
        dy = y2 - y1;

        if (dx >= 0 && dy >= 0) Rotate = 2;
        else if (dy < 0) {
            dy = -dy;
            std::swap(y1, y2);
            Rotate = 1;
        } else if (dx < 0) {
            dx = -dx;
            std::swap(x1, x2);
            Rotate = 3;
        }
    }

    if (Rotate == 1) {
        if (dx >= dy) {
            for(x = x1; x <= x2; ++x) {
                if (map.CellIsObstacle(x, y2)) return false;
                StepVal += dy;
                if (StepVal >= dx){
                    --y2;
                    StepVal -= dx;
                }
            }
        } else {
            for (y = y1; y <= y2; ++y) {
                if (map.CellIsObstacle(x2, y)) return false;
                StepVal += dx;
                if (StepVal >= dy) {
                    --x2;
                    StepVal -= dy;
                }
            }
        }
        return true;
    } else if(Rotate == 2) {
        if (dx >= dy) {
            y = y1;
            for (x = x1; x <= x2; ++x) {
                if (map.CellIsObstacle(x, y1)) return false;
                StepVal += dy;
                if (StepVal >= dx) {
                    ++y1;
                    StepVal -= dx;
                }
            }
            return true;
        } else {
            for (y = y1; y <= y2; ++y) {
                if (map.CellIsObstacle(x1, y)) return false;
                StepVal += dx;
                if (StepVal >= dy) {
                    ++x1;
                    StepVal -= dy;
                }
            }
            return true;
        }
    } else if (Rotate == 3) {
        if (dx >= dy) {
            for (x = x1; x <= x2; ++x) {
                if (map.CellIsObstacle(x, y2)) return false;
                StepVal += dy;
                if (StepVal >= dx) {
                    --y2;
                    StepVal -= dx;
                }
            }
        } else {
            for(y = y1; y <= y2; ++y) {
                if (map.CellIsObstacle(x2, y)) return false;
                StepVal += dx;
                if (StepVal >= dy) {
                    --x2;
                    StepVal -= dy;
                }
            }
        }
        return true;
    }

    if(dx >= dy) {
        for(x = x1; x <= x2; ++x) {
            if (map.CellIsObstacle(x, y1)) return false;
            StepVal += dy;
            if(StepVal >= dx){
                ++y1;
                StepVal -= dx;
            }
        }
    } else {
        for(y = y1; y <= y2; ++y) {
            if (map.CellIsObstacle(x1, y)) return false;
            StepVal += dx;
            if (StepVal >= dy) {
                ++x1;
                StepVal -= dy;
            }
        }
    }
    return true;
}

double DLian::getCost(int a_i, int a_j, int b_i, int b_j) const {
    return sqrt((a_i - b_i) * (a_i - b_i) + (a_j - b_j) * (a_j - b_j));
}

double DLian::calcAngle(const Node &dad, const Node &node, const Node &son) const {
    double cos_angle = (node.j - dad.j) * (son.j - node.j) +
                       (node.i - dad.i) * (son.i - node.i);
    cos_angle /= getCost(son.i, son.j, node.i, node.j);
    cos_angle /= getCost(node.i, node.j, dad.i, dad.j);

    if (cos_angle < -1) cos_angle = -1;
    if (cos_angle > 1) cos_angle = 1;

    return acos(cos_angle);
}

//The main pathbuilding function
SearchResult DLian::FindThePath(Map &map, EnvironmentOptions options)
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

    RemoveEdges(); // function to remove edges -- reset all parents

    /*for (auto dam : changes.occupied) { //for each damaged (0 -> 1) cell recounting values for it's neighbors
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
    } */

    if(!ComputeShortestPath(map, options)) {
        current_result.pathfound = false;
        current_result.pathlength = 0;
        end = std::chrono::system_clock::now();
        current_result.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - startt).count()) / 1000000000;
        std::cout << "AFTER THE FIRST MAP CHANGE THE PATH DOES NOT EXIST\n";
        return current_result;
    }
    changes = map.ClearTheMap(changes.occupied); //force map to change (here: clear all previous damage)

    AddEdges(); // function to add edges -- reset all parents

    /*for (auto cleared : changes.cleared) { //for each cell, that was cleared (1 -> 0) recounting values for it & it's neighbors
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
    } */

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



void DLian::Initialize(Map &map, int metrics)
{
    calculateDistances();

    std::cout << "List of distances :";
    for (auto dist : listOfDistances) {
        std::cout << " " << dist;
    }
    std::cout << std::endl;

    open.resize(map.getHeight());

    Node goal_node(map.goal_i, map.goal_j);
    goal_node.g = std::numeric_limits<double>::infinity();
    goal_node.rhs = std::numeric_limits<double>::infinity();

    NODES[goal_node.convolution(map.getHeight())] = goal_node;
    goal = &(NODES.find(goal_node.convolution(map.getHeight()))->second);

    Node start_node(map.start_i, map.start_j);
    start_node.g = std::numeric_limits<double>::infinity();
    start_node.rhs = 0;
    start_node.key = CalcuclateKey(map.start_i, map.start_j);
    start_node.radius = distance;

    NODES[start_node.convolution(map.getHeight())] = start_node;
    start = &(NODES.find(start_node.convolution(map.getHeight()))->second);

    open.put(start_node);

    calculateCircle((int) start_node.radius);
    calculatePivotCircle();

    current_result.goal_became_unreachable = false;
}

SearchResult LianSearch::startSearch(Logger *Log, const Map &map) {

    calculateDistances();

    std::cout << "List of distances :";
    for (auto dist : listOfDistances) {
        std::cout << " " << dist;
    }
    std::cout << std::endl;

    open.resize(map.getHeight());
    Node curNode(map.start_i,map.start_j, 0.0, 0.0, 0.0);
    curNode.radius = distance;
    curNode.F = weight * getCost(curNode.i,curNode.j, map.goal_i,map.goal_j);
    bool pathFound = false;
    open.add(curNode);
    calculateCircle((int) curNode.radius);
    calculatePivotCircle();

    std::chrono::time_point<std::chrono::system_clock> begin, end;
    begin = std::chrono::system_clock::now();

    /*
     * #ifdef __linux__
     *     timeval begin, end;
     *     gettimeofday(&begin, NULL);
     * #else
     *     LARGE_INTEGER begin,end,freq;
     *     QueryPerformanceCounter(&begin);
     *     QueryPerformanceFrequency(&freq);
     * #endif
     */

    while(!stopCriterion()) { // main cycle of the search
        curNode = open.getMin();
        close.insert({curNode.convolution(map.getWidth()),curNode});
        ++closeSize;

        if (curNode.i == map.goal_i && curNode.j == map.goal_j) { // if current point is goal point - end of the cycle
            pathFound = true;
            break;
        }

        if(!expand(curNode, map) && listOfDistancesSize>1)
            while(curNode.radius>listOfDistances[listOfDistancesSize-1])
                if(tryToDecreaseRadius(curNode,map.getWidth()))
                    if(expand(curNode, map))
                        break;
        if(Log->loglevel >= CN_LOGLVL_LOW) Log->writeToLogOpenClose(open, close, map.getHeight());
    }

    if(Log->loglevel==CN_LOGLVL_MED) Log->writeToLogOpenClose(open, close, map.getHeight());

    sresult.nodescreated = open.get_size() + closeSize;
    sresult.numberofsteps = closeSize;
    if (pathFound) {
        sresult.pathlength = curNode.g;
        makePrimaryPath(curNode);
        if (postsmoother) {
            hppath = smoothPath(hppath, map);
        }
        makeSecondaryPath();
        float max_angle = makeAngles();
        sresult.pathfound = true;
        sresult.hppath = hppath;
        sresult.lppath = lppath;
        sresult.angles = angles;
        sresult.max_angle = max_angle;
        sresult.sections = hppath.size()-1;

        end = std::chrono::system_clock::now();
        sresult.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count()) / 1000000000;
         /* // for more accurate time calculation
        #ifdef __linux__
            gettimeofday(&end, NULL);
            sresult.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
        #else
            QueryPerformanceCounter(&end);
            sresult.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
        #endif */

        return sresult;
    } else {
        sresult.pathfound = false;

        end = std::chrono::system_clock::now();
        sresult.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count()) / 1000000000;

        /* for more accurate time calculation
       #ifdef __linux__
           gettimeofday(&end, NULL);
           sresult.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
       #else
           QueryPerformanceCounter(&end);
           sresult.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
       #endif */

        return sresult;
    }
}


void DLian::UpdateVertex(Node* u) //adding and removing vertices from OPEN list
{
    if (!(u->IsConsistent())) {
        u->key = CalculateKey(*u);
        OPEN.put(u); //add vertex u in OPEN list or change it's key if it is already there
    } else {
        OPEN.remove_if(u); //if vertex u is in OPEN list, remove it
    }
}

//the main process of opening vertices and recalculating g- and rhs-values
bool DLian::ComputeShortestPath(Map &map, EnvironmentOptions opt)
{
    while (OPEN.top_key_less_than(CalculateKey(map.goal_i, map.goal_j)) || goal->rhs != goal->g) {
        ++number_of_steps;
        Node* current = OPEN.get(); //returns element from OPEN with smalest key value
        if (current->g > current->rhs) {
            current->g = current->rhs;
            for (auto elem : GetSuccessors(current, map, opt)) { //for each successor(neighbor) recalculate it's rhs value
                if (elem->rhs > current->g + getCost(elem->i, elem->j, current->i, current->j)) {
                    elem->parent = current;
                    current.have_children = true; // !!!!!!!!!!!!!!!!!!!!!!!!!
                    elem->rhs = current->g + getCost(elem->i, elem->j, current->i, current->j);
                }
                UpdateVertex(elem);
            }
        } else {
            current->g = std::numeric_limits<double>::infinity();
            std::vector<Node* > succ = GetSuccessors(current, map, opt);
            succ.push_back(current);
            for (auto elem : succ) {
                if (!(elem->i == start->i && elem->j == start->j) &&
                      elem->parent->i == current->i && elem->parent->j == current->j) {
                    ResetParent(elem, map);

                    /* Node min_val = GetMinPredecessor(elem, map, opt);
                    elem->rhs = min_val.rhs;
                    elem->parent = min_val.parent;*/
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


void DLian::ResetParent(Node* current, const LocalMap &map) {
    Node best;
    double best_rhs = std::numeric_limits<double>::infinity();
    for (auto n : GetSurroundings(current, map)) {
         if (!checkAngle(n.parent, n, current) || !checkLineSegment(map, n, current)) continue;
         if (best.rhs > n.g + getCost(n.i, n.j, current->i, current->j)) {
             best = n;
             best_rhs = n.g + getCost(n.i, n.j, current->i, current->j);
         }
    }
    if (best_rhs < std::numeric_limits<double>::infinity()) {
        current.parent = best;
        current.rhs = best_rhs;
    } else {
        return;
    }
    ReduceOrAddSuccessor(current, map);
}

void DLian::ReduceOrAddSuccessor(Node current, const LocalMap &map) {
    for (auto n : GetSurroundings(current, map)) {
        if (n.parent == current) {
            if (checkAngle(current.parent, current, n)) continue;
            else ResetParent(n, map);
        } else {
            if (checkAngle(current.parent, current, n) && checkLineSegment(map, current, n))
                OPEN.put(n);
        }
    }
}

int LianSearch::tryToIncreaseRadius(Node curNode) {
    bool change = false;
    int i, k = 0;
    while (k < numOfParentsToIncreaseRadius) {
        if (curNode.parent != NULL) {
            if (curNode.radius == curNode.parent->radius) {
                ++k;
                curNode = *curNode.parent;
                continue;
            }
        }
        break;
    }
    if (k == numOfParentsToIncreaseRadius) {
        for (i = listOfDistancesSize-1; i >= 0; --i)
            if (curNode.radius == listOfDistances[i]) break;
        if(i > 0) change=true;
    }
    if(change) return listOfDistances[i-1];
    else return curNode.radius;
}

void LianSearch::update(const Node current_node, Node new_node, bool &successors, std::vector<Node*> succs const Map &map) {
    if (!checkLineSegment(map, *new_node.parent, new_node)) return;
    if (pivotRadius > 0 && (new_node.i != map.goal_i || new_node.j != map.goal_j) && !checkPivotCircle(map, new_node)) return;

    auto it = close.find(new_node.convolution(map.getWidth()));
    if (it != close.end()) {
        auto range = close.equal_range(it->first);
        for (auto it = range.first; it != range.second; ++it)
            if (it->second.parent == nullptr || (it->second.parent->i == current_node.i && it->second.parent->j == current_node.j))
                return;
    }

    if(listOfDistancesSize > 1) new_node.radius = tryToIncreaseRadius(new_node);
    open.add(new_node);
    successors = true;
}


//function returns Nodes of successors of current vertex. Already with their g- and rhs-values
std::vector<Node* > DLian::GetSuccessors(Node* current, Map &map, EnvironmentOptions opt) {
    std::vector<Node*> result;

    int current_distance;
    for(current_distance = 0; current_distance < listOfDistancesSize; ++current_distance)
        if(listOfDistances[current_distance] == curNode.radius)
            break;

    std::vector<circleNode> circle_nodes = circleNodes[current_distance];

    bool successors_are_fine = false;
    auto parent = &(close.find(curNode.convolution(map.getWidth()))->second);
    if (curNode.parent != nullptr) {
        int node_straight_ahead = round(curNode.angle * circle_nodes.size() / 360);
        double angle = fabs(curNode.angle - circleNodes[current_distance][node_straight_ahead].heading);
        if ((angle <= 180 && angle <= angleLimit) || (angle > 180 && 360 - angle <= angleLimit)) {
            int new_pos_i = curNode.i + circle_nodes[node_straight_ahead].i;
            int new_pos_j = curNode.j + circle_nodes[node_straight_ahead].j;
            if (map.CellOnGrid(new_pos_i, new_pos_j) && map.CellIsTraversable(new_pos_i, new_pos_j)) {
                Node newNode = Node(new_pos_i, new_pos_j,
                               curNode.g + circle_nodes[node_straight_ahead].cost,
                               weight * getCost(new_pos_i, new_pos_j, map.goal_i, map.goal_j),
                               curNode.radius, parent, curvatureHeuristicWeight * distance,
                               circleNodes[current_distance][node_straight_ahead].heading);

                if (!checkLineSegment(map, *newNode.parent, newNode)) return;
                if (pivotRadius > 0 && (newNode.i != map.goal_i || newNode.j != map.goal_j) && !checkPivotCircle(map, newNode)) return;

                auto it = close.find(newNode.convolution(map.getWidth()));
                if (it != close.end()) {
                    auto range = close.equal_range(it->first);
                    for (auto it = range.first; it != range.second; ++it)
                        if (it->second.parent == nullptr || (it->second.parent->i == current_node.i && it->second.parent->j == current_node.j))
                            result.push_back(&it->second);
                            
                }

                if(listOfDistancesSize > 1) new_node.radius = tryToIncreaseRadius(new_node);
                result.push_back(new_node);
                successors = true;
             }
        } // now we will expand neighbors that are closest to the node that lies straight ahead

        std::vector<int> candidates = std::vector<int>{node_straight_ahead, node_straight_ahead};
        bool limit = true;

        while (++candidates[0] != --candidates[1] && limit) { // untill the whole circle is explored or we exessed anglelimit somewhere
            if (candidates[0] >= circle_nodes.size()) candidates[0] = 0;
            if (candidates[1] < 0) candidates[1] = circle_nodes.size() - 1;

            for (auto cand : candidates) {
                double angle = fabs(curNode.angle - circleNodes[current_distance][cand].heading);
                if ((angle <= 180 && angle <= angleLimit) || (angle > 180 && 360 - angle <= angleLimit)) {
                    int new_pos_i = curNode.i + circle_nodes[cand].i;
                    int new_pos_j = curNode.j + circle_nodes[cand].j;

                    if (!map.CellOnGrid(new_pos_i, new_pos_j)) continue;
                    if (map.CellIsObstacle(new_pos_i, new_pos_j)) continue;

                    Node newNode = Node(new_pos_i, new_pos_j,
                                        curNode.g + circle_nodes[cand].cost,
                                        weight * getCost(new_pos_i, new_pos_j, map.goal_i, map.goal_j),
                                        curNode.radius, parent, curvatureHeuristicWeight * distance,
                                        circleNodes[current_distance][cand].heading);
                    update(curNode, newNode, successors_are_fine, map);
                } else {
                    limit = false;
                }
            }
        }
    } else { // when we do not have parent, we should explore all neighbors
        int angle_position(-1), new_pos_i, new_pos_j;
        for (auto node : circle_nodes) {
            new_pos_i = curNode.i + node.i;
            new_pos_j = curNode.j + node.j;
            angle_position++;

            if (!map.CellOnGrid(new_pos_i, new_pos_j)) continue;
            if (map.CellIsObstacle(new_pos_i, new_pos_j)) continue;

            Node newNode = Node(new_pos_i, new_pos_j,
                           curNode.g + node.cost,
                           weight * getCost(new_pos_i, new_pos_j, map.goal_i, map.goal_j),
                           curNode.radius, parent, curvatureHeuristicWeight * distance,
                           circleNodes[current_distance][angle_position].heading);

            update(curNode, newNode, successors_are_fine, map);
        }
    }

    // when we are near goal point, we should try to reach it
    if (getCost(curNode.i, curNode.j, map.goal_i, map.goal_j) <= curNode.radius) {
        double angle = calcAngle(*curNode.parent, curNode, Node(map.goal_i, map.goal_j));

        if (fabs(angle * 180 / CN_PI_CONSTANT) <= angleLimit) {
            Node newNode = Node( map.goal_i,  map.goal_j,
                           curNode.g + getCost(curNode.i, curNode.j,  map.goal_i,  map.goal_j), 0.0,
                           curNode.radius, parent, curvatureHeuristicWeight * distance, 0.0);

            update(curNode, newNode, successors_are_fine, map);
        }
    }
    return successors_are_fine;
}


bool LianSearch::tryToDecreaseRadius(Node& curNode, int width) {
    int i;
    for(i = listOfDistancesSize - 1; i >= 0; --i)
        if (curNode.radius == listOfDistances[i]) break;
    if (i < listOfDistancesSize - 1) {
        curNode.radius = listOfDistances[i + 1];
        auto it = close.find(curNode.convolution(width));
        auto range = close.equal_range(it->first);
        for(auto it = range.first; it != range.second; ++it) {
            if(it->second.parent && it->second.parent->i == curNode.parent->i
                                 && it->second.parent->j == curNode.parent->j) {
                it->second.radius = listOfDistances[i + 1];
                break;
            }
        }
        return true;
    }
    return false;
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
