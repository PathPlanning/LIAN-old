cmake_minimum_required(VERSION 3.2)
project(LianSearch)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -g")

set(SOURCE_FILES cConfig.h cList.h cLogger.h cMap.h cMission.h cSearch.h cXmlLogger.h gl_const.h liansearch.h RSearch.h sCell.h searchresult.h sNode.h tinystr.h tinyxml.h cConfig.cpp cList.cpp cLogger.cpp cMap.cpp cMission.cpp cXmlLogger.cpp liansearch.cpp main.cpp tinystr.cpp tinyxml.cpp tinyxmlerror.cpp tinyxmlparser.cpp Bresenham.cpp Bresenham.h)

add_executable(LianSearch ${SOURCE_FILES})
