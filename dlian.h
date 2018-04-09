#ifndef DLIAN_H
#define DLIAN_H

#include "map.h"
#include "openlist.h"
#include "searchresult.h"
#include "heuristics.h"
#include "environmentoptions.h"
#include <unordered_map>
#include <set>
#include <chrono>

class DLian
{
public:
    DLian();
    DLian(double HW);
    ~DLian(void);

    //main function for the whole pathbuilding algorithm
    LPASearchResult FindThePath(Map &map, EnvironmentOptions options);

private:
    float angleLimit;

    Node *start;
    Node *goal;
    int number_of_steps;
    double hweight;

    //EnvironmentOptions opt;
    std::list<Node> path;
    std::list<Node> hpath;

    LPASearchResult current_result;
    std::vector<circleNode> circle_nodes;
    OpenList OPEN;
    std::unordered_multimap<int, Node> NODES;

    double getCost(int a_i, int a_j, int b_i, int b_j) const;

    void Initialize(Map &map);
    void UpdateVertex(Node* node);

    void update(const Node* current_node, Node new_node, bool &successors, const Map &map);

    bool ComputeShortestPath(Map &map);
    double GetCost(Cell from, Cell to) const;
    Key CalculateKey(Node &vertex);
    Node* getFromNodes(Node current_node, int width, Node *parent=nullptr);
    std::vector<Node *> GetSuccessors(Node *curr, Map &map, EnvironmentOptions opt);
    std::list<Node *> GetSurroundings(Node *current, Map &map, EnvironmentOptions opt);
    Node GetMinPredecessor(Node* curr, Map &map, EnvironmentOptions opt);
    std::list<Cell> FindNeighbors(Node* curr, Map &map, EnvironmentOptions opt) const;

    //functions for path building
    void MakePrimaryPath(Node* curNode);
    void makeSecondaryPath();
};

#endif // DLIAN_H
