#ifndef MAP_H
#define	MAP_H
#include <iostream>
#include "node.h"
#include "gl_const.h"
#include <list>
#include "searchresult.h"
#include "tinyxml2.h"
#include <sstream>
#include <string>
#include <algorithm>
#include <random>


class Map
{
    public:
        Map();
        Map(const Map& orig);
        ~Map();

        Changes DamageTheMap(std::list<Node> path);
        bool GetMap(const char *name);
        bool CellIsTraversable (int curr_i, int curr_j) const;
        bool CellOnGrid (int curr_i, int curr_j) const;
        bool CellIsObstacle(int curr_i, int curr_j) const;

        int * operator [] (int i);
        const int * operator [] (int i) const;

        void PrintPath(std::list<Node> path);
        void PrintMap();

        const int get_height() const;
        const int get_width() const;
        double get_cellsize() const;

        int start_i, start_j;
        int goal_i, goal_j;

    private:

        int **  Grid;

        void    BuildGrid();
        Node     damaged;
        int     height, width;
        double  CellSize;

        std::string filename;

};

#endif
