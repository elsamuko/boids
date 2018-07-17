CONFIG -= qt
LIBS += -lfltk -lpthread
SOURCES += boids.cpp

SOURCES += threadpool.cpp
SOURCES += threadpool.hpp

CONFIG += c++14

QMAKE_CXXFLAGS_RELEASE += -msse2 -O2
