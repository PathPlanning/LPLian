#ifndef SEARCHRESULT_H
#define SEARCHRESULT_H
#include <list>
#include "node.h"

struct Changes {
    std::list<Cell> occupied;
    std::list<Cell> cleared;
};

struct SearchResult {

    bool pathfound;
    float pathlength;
    std::list<Node> hppath,lppath;
    unsigned int nodescreated;
    unsigned int numberofsteps;
    std::vector<float> angles;
    float accum_angle;
    double time;
    float max_angle;
    int sections;

    SearchResult() : pathfound(false), pathlength(0), nodescreated(0),
                     numberofsteps(0), time(0), max_angle(0), sections(0) {
        hppath.clear();
        lppath.clear();
        angles.clear();
    }
};


#endif
