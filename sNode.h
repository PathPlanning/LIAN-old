#ifndef SNODE_H
#define SNODE_H

#include <list>
#include <functional>
#include <iostream>

#include "gl_const.h"

struct Node {
    int i, j, z;
    float F;
    float g;
    const Node *Parent;
    //bool pathToParent;
    int radius;

    float c;// curvature euristic component
    // компонент эвристики, которая учитывает "искревленность" построенного пути:
    // чем ближе путь к прямой, тем лучше

    Node() {
        i = -1;
        j = -1;
        F = -1;
        z = -1;
        g = -1;
        c = -1;
        Parent = nullptr;
        //pathToParent = false;
        radius = CN_PTD_D;
    }

    Node(int x, int y, int z, float f = 0, float G = 0, float C = 0) {
        i = x;
        j = y;
        this->z = z;
        F = f;
        g = G;
        c = C;
        Parent = nullptr;
        //pathToParent = false;
        radius = CN_PTD_D;
    }

    ~Node() {
        Parent = nullptr;
    }

};

namespace std {
    template <>
    struct hash<Node> {
        size_t operator()(const Node& node) const {
            size_t seed = 0;
            seed ^= std::hash<int>()(node.i) + 0x9e3779b9
                    + (seed << 6) + (seed >> 2);
            seed ^= std::hash<int>()(node.j) + 0x9e3779b9
                    + (seed << 6) + (seed >> 2);
            seed ^= std::hash<int>()(node.z) + 0x9e3779b9
                    + (seed << 6) + (seed >> 2);

            return seed;
        }
    };
}

struct NodeCoordEqual{
    bool operator()(const Node& lhs, const Node& rhs) const {
        return (lhs.i == rhs.i) && (lhs.j == rhs.j) && (lhs.z == rhs.z);
    }
};

#endif
