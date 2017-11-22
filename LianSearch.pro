#-------------------------------------------------
#
# Project created by QtCreator 2014-08-11T16:11:48
#
#-------------------------------------------------
#refactoring process

TARGET = LianSearch
CONFIG   += console
CONFIG   -= app_bundle
QMAKE_CXXFLAGS += -std=c++0x
TEMPLATE = app
win32 {
QMAKE_LFLAGS += -static -static-libgcc -static-libstdc++
}
SOURCES += main.cpp \
    tinyxmlparser.cpp \
    tinyxmlerror.cpp \
    tinyxml.cpp \
    tinystr.cpp \
    liansearch.cpp \
    openlist.cpp \
    config.cpp \
    map.cpp \
    mission.cpp \
    xmllogger.cpp

HEADERS += \
    tinyxml.h \
    tinystr.h \
    searchresult.h \
    gl_const.h \
    liansearch.h \
    node.h \
    openlist.h \
    config.h \
    map.h \
    logger.h \
    mission.h \
    search.h \
    xmllogger.h
