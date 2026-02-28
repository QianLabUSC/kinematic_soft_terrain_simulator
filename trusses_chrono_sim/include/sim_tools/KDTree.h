#ifndef KDTREE_H
#define KDTREE_H
#include "GroundStiffness.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>

/*
// GroundStiffness struct with x, y coordinates and stiffness value
struct GroundStiffness {
    double x;
    double y;
    double stiffness;
};
*/
struct Node {
    GroundStiffness data;
    Node* left;
    Node* right;
    int axis;

    Node(GroundStiffness gs, int ax) : data(gs), left(nullptr), right(nullptr), axis(ax) {}
};

class KDTree {
public:
    KDTree();

    void insert(const GroundStiffness& gs);
    GroundStiffness nearestNeighbor(const GroundStiffness& target);

private:
    Node* root;

    Node* insertRec(Node* node, const GroundStiffness& gs, int depth);
    double distanceSquared(const GroundStiffness& a, const GroundStiffness& b);
    GroundStiffness nearestRec(Node* node, const GroundStiffness& target, GroundStiffness best, double bestDist);
};

#endif // KDTREE_H
