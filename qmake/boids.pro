CONFIG -= qt
LIBS += -lfltk -lpthread

SOURCES += ../src/boids.cpp
SOURCES += ../src/threadpool.cpp
HEADERS += ../src/threadpool.hpp

CONFIG += c++14

QMAKE_CXXFLAGS_RELEASE += -msse2 -O2
