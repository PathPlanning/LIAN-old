#ifndef XMLLOGGER_H
#define XMLLOGGER_H

#include "gl_const.h"
#include "logger.h"
#include "tinyxml.h"
#include "tinystr.h"

#include <iostream>
#include <sstream>
#include <string>

class cXmlLogger : public Logger {

private:
    std::string     logFileName;
    TiXmlDocument   *doc;

public:
    cXmlLogger(float loglvl);
    ~cXmlLogger();

    bool getLog(const char* FileName);
    void saveLog();
    void writeToLogMap(const Map &map,const std::list<Node> &path);
    void writeToLogOpenClose(const OpenList &open, const std::unordered_multimap<int, Node> &close, const int size);
    void writeToLogPath(const std::list<Node> &path, const std::vector<float> &angles);
    void writeToLogHpLevel(const std::list<Node> &path);
    void writeToLogSummary(const std::list<Node> &path, int numberofsteps, int nodescreated, float length, long double time, float max_angle, int sections);
};

#endif
