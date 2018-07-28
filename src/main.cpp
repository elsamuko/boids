// boids demo
// http://en.wikipedia.org/wiki/Boids

#include "gui/canvas.hpp"
#include "gui/mainwindow.hpp"

int main( int argc, char** argv ) {
    srand( time( NULL ) );
    MainWindow mw;
    mw.show( argc, argv );
    int rv = Fl::run();
    return rv;
}
