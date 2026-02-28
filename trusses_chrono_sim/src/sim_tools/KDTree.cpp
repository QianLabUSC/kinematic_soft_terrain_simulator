#include "sim_tools/KDTree.h"
#include "sim_tools/GroundStiffness.h"

// Constructor
KDTree::KDTree() : root(nullptr) {}

// Insert a GroundStiffness point into the KDTree
void KDTree::insert(const GroundStiffness& gs) {
    root = insertRec(root, gs, 0);
}

// Find the nearest neighbor to the given GroundStiffness point
GroundStiffness KDTree::nearestNeighbor(const GroundStiffness& target) {
    if (!root) {
        throw std::runtime_error("The KDTree is empty.");
    }
    return nearestRec(root, target, root->data, std::numeric_limits<double>::max());
}

// Recursive function to insert a GroundStiffness point
Node* KDTree::insertRec(Node* node, const GroundStiffness& gs, int depth) {
    if (node == nullptr) {
        return new Node(gs, depth % 2);
    }

    int axis = node->axis;
    if ((axis == 0 && gs.x < node->data.x) || (axis == 1 && gs.y < node->data.y)) {
        node->left = insertRec(node->left, gs, depth + 1);
    } else {
        node->right = insertRec(node->right, gs, depth + 1);
    }

    return node;
}

// Calculate squared distance between two GroundStiffness points
double KDTree::distanceSquared(const GroundStiffness& a, const GroundStiffness& b) {
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

// Recursive function to find the nearest neighbor
GroundStiffness KDTree::nearestRec(Node* node, const GroundStiffness& target, GroundStiffness best, double bestDist) {
    if (node == nullptr) {
        return best;
    }

    double d = distanceSquared(node->data, target);
    if (d < bestDist) {
        bestDist = d;
        best = node->data;
    }

    int axis = node->axis;
    Node* nextBranch = nullptr;
    Node* oppositeBranch = nullptr;

    if ((axis == 0 && target.x < node->data.x) || (axis == 1 && target.y < node->data.y)) {
        nextBranch = node->left;
        oppositeBranch = node->right;
    } else {
        nextBranch = node->right;
        oppositeBranch = node->left;
    }

    best = nearestRec(nextBranch, target, best, bestDist);

    // Update the bestDist after the recursive call
    bestDist = distanceSquared(best, target);
    
    double splitDist = axis == 0 ? std::abs(node->data.x - target.x) : std::abs(node->data.y - target.y);
    if (splitDist * splitDist < bestDist) {
        best = nearestRec(oppositeBranch, target, best, bestDist);
    }

    return best;
}
