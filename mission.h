#ifndef CMISSION_H
#define CMISSION_H

#include "map.h"
#include "config.h"
#include "search.h"
#include <string>
#include "liansearch.h"
#include "xml_logger.h"
#include "searchresult.h"

class Mission
{
public:
    Mission(const char* fName);
    ~Mission();

    bool getMap();
    bool getConfig();
    bool createLog();
    void createSearch();
    void startSearch();
    void printSearchResultsToConsole();
    void saveSearchResultsToLog();

private:
    Map    m_map;
    Config m_config;

    Search *m_pSearch;
    Logger *m_pLogger;

    const char* m_fileName;

    SearchResult sr;
};

#endif

