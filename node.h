#ifndef NODE_H
#define NODE_H

#include "gl_const.h"
#include "map"

#include <iostream>
#include <limits>
#include <vector>

struct Cell {
    int i, j;

    Cell() {}
    Cell(int i_, int j_) : i(i_), j(j_) {}
    Cell(Cell other, int i_, int j_) : i(other.i + i_), j(other.j + j_) {}

    inline bool operator==(const Cell& p) const {
        return i == p.i && j == p.j;
    }

    inline bool operator!=(const Cell& p) const {
        return !(*this == p);
    }

    inline bool operator<(const Cell& p) const {
        return i < p.i || (i == p.i && j < p.j);
    }

    Cell& operator=(const Cell& other) {
        if (this == &other) {
            return *this;
        }
        i = other.i;
        j = other.j;
        return *this;
    }
        Cell& operator+=(const Cell& other) {
        i += other.i;
        j += other.j;
        return *this;
    }
};

inline Cell operator+(const Cell& one, const Cell& other) {
    Cell newc;
    newc.i = one.i + other.i;
    newc.j = one.j + other.j;
    return newc;
}

inline Cell operator-(const Cell& one, const Cell& other) {
    Cell newc;
    newc.i = one.i - other.i;
    newc.j = one.j - other.j;
    return newc;
}

inline std::ostream& operator<< (std::ostream& out, const Cell &next) {
    out << "(" << next.i << "," << next.j << "); ";
    return out;
}


class Node {
public:
    double  F, g;
    Cell    cell;
    Node*   parent;
    bool    path_to_parent;
    int     radius;

    float   c; // curvature euristic component, that counts bending component of built path; the closer path to straight line - the better
    int     breakingties;


    Node() : breakingties(1), g(std::numeric_limits<double>::infinity()),
        F(std::numeric_limits<double>::infinity()), parent(nullptr)  {}
    Node(const Cell& p, int bt = 1, Node *c = nullptr) : g(std::numeric_limits<double>::infinity()),
        F(std::numeric_limits<double>::infinity()), cell(p), parent(c), breakingties(bt) {}

    Node& operator=(const Node& other) {
        cell = other.cell;
        F = other.F;
        g = other.g;
        parent = other.parent;
        path_to_parent = other.path_to_parent;
        radius = other.radius;
        c = other.c;
        return *this;
    }

    int convolution(int width) const {
        return cell.i * width + cell.j;
    }
};

inline bool operator<(const Node& one, const Node& another) {
    return one.F < another.F || (one.F == another.F && (another.breakingties == CN_BT_GMAX && one.g > another.g ||
                                                        another.breakingties == CN_BT_GMIN && one.g < another.g));
}

inline std::ostream& operator<<(std::ostream& out, const Node &next) {
    out << "(" << next.cell.i << "," << next.cell.j << "); ";
    return out;
}

#endif
