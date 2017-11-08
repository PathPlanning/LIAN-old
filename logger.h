#ifndef CLOGGER_H
#define CLOGGER_H

#include "node.h"
#include "map.h"

#include <list>
#include <vector>
#include <unordered_map>

class Logger
{	
public:
    float loglevel;

public:
    Logger(): loglevel(-1) {}
    virtual ~Logger() {}
    virtual bool getLog(const char* FileName) = 0;
    virtual void saveLog() = 0;
    virtual void writeToLogMap(const Map &Map,const std::list<Node> &path) = 0;
    virtual void writeToLogOpenClose(const std::vector<std::list<Node> > &open, const std::unordered_multimap<int, Node>& close) = 0;
    virtual void writeToLogPath(const std::list<Node> &path, const std::vector<float> &angles) = 0;
    virtual void writeToLogHpLevel(const std::list<Node> &path) = 0;
    virtual void writeToLogSummary(const std::list<Node> &path, int numberofsteps, int nodescreated, float length, double Time, float maxAngle, int sections) = 0;};

#endif
