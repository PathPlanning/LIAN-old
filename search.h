#ifndef CSEARCH_H
#define CSEARCH_H

#include "map.h"
#include "logger.h"
#include "xml_logger.h"
#include "gl_const.h"
#include "searchresult.h"

class Search
{
public:
    Search() {}
    virtual ~Search() {}
    virtual void addOpen(Node& newNode) = 0;
    virtual SearchResult startSearch(Logger *Log, const Map &Map) = 0;

    SearchResult sresult;
};

#endif
