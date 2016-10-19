#ifndef CMAP_H
#define CMAP_H

#include<iostream>
#include"tinyxml.h"
#include"tinystr.h"
#include "gl_const.h"

class cMap {
public:
    int **Grid;
    int height, width, altitude_max;
    int min_altitude_limit, max_altitude_limit; // The lowest and highest possible altitude for the path
    int start_i, start_j, start_z;
    int goal_i, goal_j, goal_z;

public:
    cMap();

    ~cMap();

    bool getMap(const char *FileName);

    bool CellIsTraversable(int i, int j) const;

    bool CellIsTraversable(int i, int j, int h) const;

    bool CellOnGrid(int i, int j) const;

    bool CellOnGrid(int i, int j, int height) const;

    bool CellIsObstacle(int i, int j) const;

    bool CellIsObstacle(int i, int j, int h) const;

    int getValue(int i, int j) const;
};

#endif
