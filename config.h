#ifndef CONFIG_H
#define CONFIG_H

#include "gl_const.h"
#include "tinyxml.h"
#include "tinystr.h"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>

class Config {

private:
    float *searchParams;
    int N;

public:
    Config();
    Config(int numParams, float *paramArray);
    ~Config();

    float getParamValue(int i) const;

    bool getConfig(const char* FileName);
};

#endif
