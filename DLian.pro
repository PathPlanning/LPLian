TEMPLATE = app
CONFIG   += console
CONFIG   -= app_bundle
QMAKE_CXXFLAGS += -std=c++0x
TEMPLATE = app
win32 {
QMAKE_LFLAGS += -static -static-libgcc -static-libstdc++
}

SOURCES += main.cpp \
    config.cpp \
    map.cpp \
    mission.cpp \
    tinyxml2.cpp \
    xmllogger.cpp \
    dlian.cpp \
    openlist.cpp

HEADERS += \
    config.h \
    gl_const.h \
    map.h \
    mission.h \
    node.h \
    openlist.h \
    searchresult.h \
    tinyxml2.h \
    xmllogger.h \
    logger.h \
    dlian.h \
    linefunctions.h

