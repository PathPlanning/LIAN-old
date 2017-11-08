#ifndef CXMLLOGGER_H
#define CXMLLOGGER_H

#include"logger.h"
#include<iostream>
#include"tinyxml.h"
#include"tinystr.h"
#include<string>


class XmlLogger:public Logger
{
private:
    std::string LogFileName;
    TiXmlDocument *doc;

public:
    XmlLogger(float loglvl);
    ~XmlLogger();

    bool getLog(const char* FileName);
    void saveLog();
    void writeToLogMap(const Map &Map,const std::list<Node> &path);
    void writeToLogOpenClose(const std::vector<std::list<Node>> &open, const std::unordered_multimap<int, Node> &close);
    void writeToLogPath(const std::list<Node> &path, const std::vector<float> &angles);
    void writeToLogHpLevel(const std::list<Node> &path);
    void writeToLogSummary(const std::list<Node> &path, int numberofsteps, int nodescreated, float length, double Time, float maxAngle, int sections);
};

#endif
