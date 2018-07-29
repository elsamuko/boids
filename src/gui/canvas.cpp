#include "gui/canvas.hpp"

void Canvas::Timer( void* in ) {
    Canvas* canvas = ( Canvas* )in;
    canvas->redraw();
    Fl::repeat_timeout( 0.05, Timer, in );
}

Canvas::Canvas( int X, int Y, int W, int H, const char* L ) : Fl_Box( X, Y, W, H, L ) {
    boids.boids_old.resize( 200 );
    boids.boids_new = boids.boids_old;
    redraw();
    Fl::add_timeout( 0.05, Timer, ( void* )this );
}

int Canvas::handle( int event ) {

    if( event == FL_MOVE ) {
        boids.eagle = vec2D( Fl::event_x(), Fl::event_y() );
    }

    return Fl_Box::handle( event );
}

void Canvas::draw() {
    Fl_Box::draw();

    // flock
    fl_color( fl_rgb_color( 0 ) );

    for( const Boid& boid : boids.boids_old ) {
        // fl_circle( boid.pos.x, boid.pos.y, 4 );
        vec2D dir = 20 * boid.mom.norm();
        fl_loop( boid.pos.x,
                 boid.pos.y,
                 boid.pos.x - dir.x - 0.2 * dir.y,
                 boid.pos.y - dir.y + 0.2 * dir.x,
                 boid.pos.x - dir.x + 0.2 * dir.y,
                 boid.pos.y - dir.y - 0.2 * dir.x );
    }

    if( boids.debug ) {
        fl_line_style( FL_SOLID, 2 );

        // sight of first boid
        fl_color( FL_YELLOW );
        fl_circle( boids.boids_old[0].pos.x, boids.boids_old[0].pos.y, boids.sight );

        // momentum of first boid
        fl_color( FL_BLUE );
        fl_line( boids.boids_old[0].pos.x,
                 boids.boids_old[0].pos.y,
                 boids.boids_old[0].pos.x + 10.0 * boids.boids_old[0].mom.x,
                 boids.boids_old[0].pos.y + 10.0 * boids.boids_old[0].mom.y );

        // moveTo of first boid
        fl_color( FL_CYAN );
        fl_line( boids.boids_old[0].pos.x,
                 boids.boids_old[0].pos.y,
                 boids.boids_old[0].pos.x + 2.0 * boids.boids_old[0].moveTo.x,
                 boids.boids_old[0].pos.y + 2.0 * boids.boids_old[0].moveTo.y );
        // moveAway of first boid
        fl_color( FL_MAGENTA );
        fl_line( boids.boids_old[0].pos.x,
                 boids.boids_old[0].pos.y,
                 boids.boids_old[0].pos.x + 2.0 * boids.boids_old[0].moveAway.x,
                 boids.boids_old[0].pos.y + 2.0 * boids.boids_old[0].moveAway.y );

        // the eagle
        // fl_color( fl_rgb_color( 50 ) );
        // fl_circle( boids.eagle.x, boids.eagle.y, 10 );
        fl_line_style( 0 );
    }

    boids.iterate_all();

    if( ++iterations > 100 ) {
        iterations = 0;
        LOG( boids.boids_old[0] );
    }
}
