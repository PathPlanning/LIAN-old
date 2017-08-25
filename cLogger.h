#ifndef CLOGGER_H
#define CLOGGER_H

#include"cMap.h"
#include "Queues.h"

#include <list>
#include <vector>
#include <unordered_set>

class cLogger {
public:
    float loglevel;

public:
    cLogger();

    virtual ~cLogger();

    virtual bool getLog(const char *FileName) = 0;

    virtual void saveLog() = 0;

    virtual void writeToLogMap(const cMap &Map, const std::list<Node> &path) = 0;

    virtual void writeToLogOpenClose(const iOpen *open,
                                     const std::unordered_multiset<Node, std::hash<Node>, NodeCoordEqual> &close, const int size) = 0;

    virtual void writeToLogPath(const std::list<Node> &path, const std::vector<float> &angles) = 0;

    virtual void writeToLogHpLevel(const std::list<Node> &path) = 0;

    virtual void
    writeToLogSummary(const std::list<Node> &path, int numberofsteps, int nodescreated, float length, long double Time,
                      float maxAngle, int sections) = 0;
};

#endif
