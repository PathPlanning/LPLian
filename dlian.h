#ifndef DLIAN_H
#define DLIAN_H

#include "map.h"
#include "openlist.h"
#include "searchresult.h"
#include "heuristics.h"
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
    SearchResult FindThePath(Map &map);

private:
    float angleLimit;

    Node *start;
    Node *goal;
    int number_of_steps;
    double hweight;
    bool postsmoother; // Smoothing the path after the algorithm

    //EnvironmentOptions opt;
    std::list<Node> hppath;
    std::list<Node> lppath;

    std::vector<float> angles;

    SearchResult current_result;
    std::vector<circleNode> circle_nodes;
    OpenList OPEN;
    std::unordered_multimap<int, Node> NODES;

    double getCost(int a_i, int a_j, int b_i, int b_j) const;

    void Initialize(Map &map);
    void UpdateVertex(Node* node);

    void update(const Node* current_node, Node new_node, bool &successors, const Map &map);
    bool expand(const Node* curNode, const Map &map);

    bool ComputeShortestPath(Map &map);
    double GetCost(Cell from, Cell to) const;
    Key CalculateKey(Node &vertex);

    Node* getFromNodes(Node current_node, int width, Node *parent=nullptr);
    std::list<Node*> getAllNodes(Node current_node, int width);
    void ResetParent(Node* current, Node *parent, const LocalMap &map);

    std::vector<Node *> GetSuccessors(Node *curr, Map &map);

    bool checkAngle(const Node &dad, const Node &node, const Node &son) const;
    std::list<Node> smoothPath(const std::list<Node>& path, const Map& map);

    //functions for path building
    void makePrimaryPath(Node* curNode);
    void makeSecondaryPath();

    double makeAngles();
};

#endif // DLIAN_H
