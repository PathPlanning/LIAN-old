#ifndef CXMLLOGGER_H
#define CXMLLOGGER_H

#include"cLogger.h"
#include"tinyxml.h"
#include"tinystr.h"

#include<iostream>
#include<string>
#include <unordered_set>


class cXmlLogger:public cLogger
{
private:
    std::string LogFileName;
    TiXmlDocument *doc;

public:
    cXmlLogger(float loglvl);
    ~cXmlLogger();

    bool getLog(const char* FileName);
    void saveLog();
    void writeToLogMap(const cMap &Map,const std::list<Node> &path);
    void writeToLogOpenClose(const std::list<Node> *open, const std::unordered_multiset<Node> &close, const int size);
    void writeToLogPath(const std::list<Node> &path, const std::vector<float> &angles);
    void writeToLogHpLevel(const std::list<Node> &path);
    void writeToLogSummary(const std::list<Node> &path, int numberofsteps, int nodescreated, float length, long double Time, float maxAngle, int sections);
};

#endif
