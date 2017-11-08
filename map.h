#ifndef MAP_H
#define MAP_H

#include <algorithm>

#include "gl_const.h"
#include "node.h"
#include "tinyxml.h"
#include "tinystr.h"

#include <iostream>
#include <sstream>
#include <string>

class Map
{
public: 
    Map();
    ~Map();
    bool getMap(const char* FileName);

    bool CellIsTraversable (Cell curr) const;
    bool CellOnGrid (Cell curr) const;
    bool CellIsObstacle(Cell cur) const;

    int* operator [] (int i);
    const int* operator [] (int i) const;

    int height, width;
    Cell start;
    Cell goal;

private:
    int ** Grid;

};

#endif
