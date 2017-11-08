#-------------------------------------------------
#
# Project created by QtCreator 2014-08-11T16:11:48
#
#-------------------------------------------------

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
    xml_logger.cpp \
    mission.cpp \
    map.cpp \
    config.cpp

HEADERS += \
    tinyxml.h \
    tinystr.h \
    searchresult.h \
    gl_const.h \
    liansearch.h \
    config.h \
    logger.h \
    xml_logger.h \
    mission.h \
    search.h \
    map.h \
    node.h
