CONFIG -= qt
CONFIG += c++14

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
linux: include( linux.pri )
win32: CONFIG += static
win32: include( win.pri )
include( gui.pri )
include( libs.pri )
