#ifndef SEARCHRESULT_H
#define SEARCHRESULT_H

#include <vector>

struct SearchResult {
    bool pathfound;
    double pathlength;
    std::list<Node> hppath, lppath;
    unsigned int nodescreated;
    unsigned int numberofsteps;
    std::vector<float> angles;
    double time;
    double maxAngle;
    int sections;

    SearchResult() {
        pathfound = false;
        pathlength = 0;
        hppath.clear();
        lppath.clear();
        angles.clear();
        nodescreated = 0;
        numberofsteps = 0;
        time = 0;
        maxAngle = 0;
        sections = 0;
    }

};

#endif // SEARCHRESULT_H
