TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    config.cpp \
    environmentoptions.cpp \
    map.cpp \
    mission.cpp \
    tinyxml2.cpp \
    xmllogger.cpp \
    dlian.cpp \
    openlist.cpp

HEADERS += \
    config.h \
    gl_const.h \
    heuristics.h \
    map.h \
    mission.h \
    node.h \
    openlist.h \
    searchresult.h \
    tinyxml2.h \
    xmllogger.h \
    logger.h \
    dlian.h

