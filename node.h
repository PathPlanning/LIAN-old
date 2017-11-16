#ifndef NODE_H
#define NODE_H

#include "gl_const.h"

#include <limits>
#include <list>
#include <iostream>


struct Node {
    int     i, j;
    double  F;
    double  g;
    Node*   parent;
    bool    pathToParent;
    int     radius;

    double   c; // curvature euristic component

    Node() : i(-1), j(-1), F(std::numeric_limits<double>::infinity()), g(std::numeric_limits<double>::infinity()),
    c(-1), parent(nullptr), pathToParent(false), radius(CN_PTD_D) {}

    Node(int x, int y, double F_=std::numeric_limits<double>::infinity(), double g_=std::numeric_limits<double>::infinity(), double c_=-1) :  i(x), j(y), F(F_), g(g_),
        c(c_), parent(nullptr), pathToParent(false), radius(CN_PTD_D) {}

    ~Node() {
        parent = nullptr;
    }

    inline Node& operator=(const Node& other) {
        i = other.i;
        j = other.j;
        F = other.F;
        g = other.g;
        parent = other.parent;
        pathToParent = other.pathToParent;
        radius = other.radius;
        c = other.c;
        return *this;
    }

    inline bool operator==(const Node& p) const {
            return i == p.i && j == p.j && parent->i == p.parent->i && parent->j == p.parent->j;
    }

    inline bool operator!=(const Node& p) const {
            return !(*this == p);
    }

    bool lesser(const Node another, int BT) const {
        return F < another.F || (F == another.F && (BT == CN_BT_GMAX && g > another.g ||
                                                    BT == CN_BT_GMIN && g < another.g));
    }

    int convolution(int width) const {
            return i * width + j;
    }
};

#endif
