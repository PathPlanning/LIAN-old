#ifndef SNODE_H
#define SNODE_H

#include <list>
#include <iostream>

#include "gl_const.h"

struct Node {
    int i, j, z;
    float F;
    float g;
    Node *Parent;
    bool pathToParent;
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
        pathToParent = false;
        radius = CN_PTD_D;
    }

    Node(int x, int y, int z, float f = 0, float G = 0, float C = 0) {
        i = x;
        j = y;
        F = f;
        g = G;
        c = C;
        Parent = nullptr;
        pathToParent = false;
        radius = CN_PTD_D;
    }

    ~Node() {
        Parent = nullptr;
    }
};

#endif
