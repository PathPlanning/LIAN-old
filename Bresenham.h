#ifndef BRESENHAM_H
#define BRESENHAM_H

#include "cMap.h"
#include "sNode.h"

#include <list>
#include <vector>

class iBresenham {
protected:
    void bresenham3d(int x1, int y1, int z1, const int x2, const int y2, const int z2); // 3D Bresenham algorithm
    virtual bool ProcessPoint(int x, int y, int z) = 0; // Processes point in line. Return false, if line processing should be stopped
};

class LineOfSight : public iBresenham {
private:
    bool force_stopped;
    const cMap& map;
protected:
    virtual bool ProcessPoint(int i, int j, int h);

public:
    LineOfSight(const cMap&);
    bool line_of_sight(int i0, int j0, int h0, int i1, int j1, int h1);
    bool line_of_sight(const Node& from, const Node& to);
};

class Liner : public iBresenham {
private:
    std::vector<Node> *path;

protected:
    virtual bool ProcessPoint(int i, int j, int h);

public:
    Liner(std::vector<Node> *init_path);
    void append_line(int i0, int j0, int h0, int i1, int j1, int h1);
    void append_line(const Node &from, const Node &to);
};

#endif //BRESENHAM_H
