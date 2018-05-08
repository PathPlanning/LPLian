#ifndef LINEFUNCTIONS
#define LINEFUNCTIONS

#include "map.h"
#include "node.h"

#include <vector>

inline void calculateLineSegment(std::vector<Node> &line, const Node &start, const Node &goal) {
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

inline bool checkLineSegment(const Map &map, const Node &start, const Node &goal) {
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

#endif // LINEFUNCTIONS

