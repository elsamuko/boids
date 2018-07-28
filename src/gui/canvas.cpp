#include "gui/canvas.hpp"

void Canvas::draw() {
    Fl_Box::draw();

    // flock
    fl_color( fl_rgb_color( 0 ) );

    for( const Boid& boid : boids.boids_old ) {
        fl_circle( boid.pos.x, boid.pos.y, 4 );
    }

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
    fl_color( fl_rgb_color( 50 ) );
    fl_circle( boids.eagle.x, boids.eagle.y, 10 );

    boids.iterate_all();
    fl_line_style( 0 );

    if( ++iterations > 100 ) {
        iterations = 0;
        LOG( boids.boids_old[0] );
    }
}
