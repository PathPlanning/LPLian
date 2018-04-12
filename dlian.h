#ifndef DLIAN_H
#define DLIAN_H

#include "linefunctions.h"
#include "map.h"
#include "openlist.h"
#include "searchresult.h"

#include <unordered_map>
#include <set>
#include <chrono>

class DLian
{
public:
    DLian();
    DLian(float angleLimit_, int distance_, float hweight_, bool postsmoother_);

    ~DLian(void);

    //main function for the whole pathbuilding algorithm
    SearchResult FindThePath(Map &map);

private:
    float angleLimit;
    int distance; // Minimal value of length of steps

    Node *start;
    Node *goal;
    int number_of_steps;
    float hweight;
    bool postsmoother; // Smoothing the path after the algorithm

    //EnvironmentOptions opt;
    std::list<Node> hppath;
    std::list<Node> lppath;

    std::vector<float> angles;

    SearchResult current_result;
    std::vector<circleNode> circle_nodes;

    void calculateCircle(int radius);

    OpenList OPEN;
    std::unordered_multimap<int, Node> NODES;

    void Initialize(Map &map);
    void UpdateVertex(Node* node);

    void update(Node* current_node, Node new_node, bool &successors, const Map &map);
    bool expand(Node* curNode, const Map &map);

    bool ComputeShortestPath(Map &map);
    Key CalculateKey(Node &vertex);

    Node* getFromNodes(Node current_node, int width, Node *parent=nullptr);
    std::vector<Node *> getAllNodes(Node current_node, int width);
    void ResetParent(Node* current, Node *parent, const Map &map);

    std::vector<Node *> GetSuccessors(Node *curr, Map &map);
    std::vector<Node *> GetSurroundings(Node current, Map &map);

    bool checkAngle(const Node &dad, const Node &node, const Node &son) const;
    std::list<Node> smoothPath(const std::list<Node>& path, const Map& map);

    //functions for path building
    void makePrimaryPath(Node* curNode);
    void makeSecondaryPath();

    double makeAngles();
};

#endif // DLIAN_H
