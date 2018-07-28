CONFIG -= qt
CONFIG += c++14

LIBS += -lfltk -lpthread

MAIN_DIR =..
SRC_DIR  =../src
INCLUDEPATH += $${SRC_DIR}

SOURCES += $${SRC_DIR}/main.cpp

HEADERS += $${SRC_DIR}/threadpool.hpp
SOURCES += $${SRC_DIR}/threadpool.cpp

HEADERS += $${SRC_DIR}/gui/canvas.hpp
SOURCES += $${SRC_DIR}/gui/canvas.cpp

HEADERS += $${SRC_DIR}/boids.hpp
SOURCES += $${SRC_DIR}/boids.cpp

HEADERS += $${SRC_DIR}/vec2d.hpp

include( setup.pri )
include( linux.pri )
include( gui.pri )
