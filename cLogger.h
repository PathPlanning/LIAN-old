#ifndef CLOGGER_H
#define CLOGGER_H

#include"cMap.h"
#include"cList.h"
#include <vector>
#include <unordered_map>
class cLogger
{	
public:
    float loglevel;

public:
    cLogger();
    virtual ~cLogger();
    virtual bool getLog(const char* FileName) = 0;
    virtual void saveLog() = 0;
    virtual void writeToLogMap(const cMap &Map,const cList &path) = 0;
    virtual void writeToLogOpenClose(const std::vector<std::unordered_multimap<unsigned, Node>> &open, const std::unordered_multimap<int, Node>& close) = 0;
    virtual void writeToLogPath(const cList &path, const std::vector<float> &angles) = 0;
    virtual void writeToLogHpLevel(const cList &path) = 0;
    virtual void writeToLogSummary(const cList &path, int numberofsteps, int nodescreated, float length, double Time, float maxAngle, int sections) = 0;};

#endif
