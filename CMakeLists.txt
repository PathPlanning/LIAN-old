cmake_minimum_required(VERSION 2.8)
project(LIAN)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2")
set(SOURCE_FILES
        cConfig.cpp
        cConfig.h
        cList.cpp
        cList.h
        cLogger.cpp
        cLogger.h
        cMap.cpp
        cMap.h
        cMission.cpp
        cMission.h
        cSearch.h
        cXmlLogger.cpp
        cXmlLogger.h
        gl_const.h
        liansearch.cpp
        liansearch.h
        main.cpp
        sCell.h
        searchresult.h
        sNode.h
        tinystr.cpp
        tinystr.h
        tinyxml.cpp
        tinyxml.h
        tinyxmlerror.cpp
        tinyxmlparser.cpp
        )
add_executable(LIAN ${SOURCE_FILES})