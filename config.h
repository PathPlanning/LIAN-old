#ifndef CONFIG_H
#define CONFIG_H

#include<iostream>
#include"tinyxml.h"
#include"tinystr.h"
#include<string>
#include"gl_const.h"


class Config
{
public:
    float *searchParams;
    int N;
public:
    Config();
    Config(int numParams, float *paramArray);
    ~Config();

    bool getConfig(const char* FileName);
};

#endif
