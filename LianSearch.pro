#-------------------------------------------------
#
# Project created by QtCreator 2014-08-11T16:11:48
#
#-------------------------------------------------

TARGET = LianSearch
CONFIG   += console
CONFIG   -= app_bundle
QMAKE_CXXFLAGS += -std=c++11 -O3
TEMPLATE = app
win32 {
QMAKE_LFLAGS += -static -static-libgcc -static-libstdc++
}
SOURCES += main.cpp \
    tinyxmlparser.cpp \
    tinyxmlerror.cpp \
    tinyxml.cpp \
    tinystr.cpp \
    cXmlLogger.cpp \
    cMission.cpp \
    cMap.cpp \
    cLogger.cpp \
    cConfig.cpp \
    liansearch.cpp \
    Bresenham.cpp \
    Queues.cpp

HEADERS += \
    tinyxml.h \
    tinystr.h \
    sNode.h \
    searchresult.h \
    gl_const.h \
    cXmlLogger.h \
    cSearch.h \
    cMission.h \
    cMap.h \
    cLogger.h \
    cConfig.h \
    liansearch.h \
    Bresenham.h \
    Queues.h
