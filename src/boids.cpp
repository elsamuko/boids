#include <chrono>
#include <limits>

#include "boids.hpp"

inline double sign( double num ) {
    if( num  > 0.0 ) { return 1.0; }
    else { return -1.0; }
}

// https://martin.ankerl.com/2012/01/25/optimized-approximative-pow-in-c-and-cpp/
inline double fastPow( double a, double b ) {
    union {
        double d;
        int x[2];
    } u = { a };
    u.x[1] = ( int )( b * ( u.x[1] - 1072632447 ) + 1072632447 );
    u.x[0] = 0;
    return u.d;
}

Boids::Boids( int widthIn, int heightIn, int countIn, double c_moveWithIn, double c_moveToIn, double c_moveAwayIn, double maxSpeedIn, double maxAccelIn, double sightIn, double fearEagleIn ) :
    generator( std::chrono::system_clock::now().time_since_epoch().count() ),
    distribution( 0.0, maxAccelIn / 10.0 ),
    debug( false ),
    width( widthIn ),
    height( heightIn ),
    c_moveWith( c_moveWithIn ),
    c_moveTo( c_moveToIn ),
    c_moveAway( c_moveAwayIn ),
    maxSpeed( maxSpeedIn ),
    maxAccel( maxAccelIn ),
    sight( sightIn ),
    fearEagle( fearEagleIn ) {

    this->resize( countIn );
}

void Boids::threeRules( Boid& boid ) {
    vec2D dpos; // delta position
    vec2D ddir; // delta direction (normalized position)
    vec2D MA_DPOS;
    vec2D MT_DPOS;
    vec2D MW_MOM;
    int i = 0;
    double distance = 0.f;

    for( const Boid& other : boids_old ) {

        // self
        if( &boid == &other ) { continue; }

        dpos = other.pos - boid.pos;
        distance = dpos.abs();

        if( distance > std::numeric_limits<double>::epsilon() && distance < sight ) {

            i++; // count boids in sight
            ddir = dpos.norm();

            // separation: steer to avoid crowding local flockmates
            // closer ones repulse stronger
            MA_DPOS -= ddir * fastPow( distance / 5.0, -1.0 );

            // cohesion: steer to move toward the average position of local flockmates
            MT_DPOS += other.pos;

            // alignment: steer towards the average heading of local flockmates
            MW_MOM += other.mom;
        }
    }

    // correct boids acceleration
    if( i > 0 ) {
        boid.moveAway = 450 * MA_DPOS / i;
        boid.accel += c_moveAway * boid.moveAway;

        boid.moveTo = 0.13 * ( ( MT_DPOS / i ) - boid.pos );
        boid.accel += c_moveTo * boid.moveTo;

        boid.moveWith = 0.5 * ( MW_MOM  / i );
        boid.accel += c_moveWith * boid.moveWith;
    }
}

void Boids::fleeFromEagle( Boid& in ) {
    vec2D dpos = eagle - in.pos;
    double distance = dpos.abs();

    if( distance > std::numeric_limits<double>::epsilon() && distance < sight ) {
        in.fear = 200 * dpos.norm() * fastPow( distance / 5.0, -1.0 );
        in.accel -= fearEagle * in.fear;
    }
}

void Boids::borderConstraints( Boid& boid ) {

    double lower = 0.1;
    double upper = 0.9;

    if( boid.pos.x > upper * width ) {
        boid.accel.x = -maxAccel;
    }

    if( boid.pos.y > upper * height ) {
        boid.accel.y = -maxAccel;
    }

    if( boid.pos.x < lower * width ) {
        boid.accel.x = maxAccel;
    }

    if( boid.pos.y < lower * height ) {
        boid.accel.y = maxAccel;
    }

    // limit speed
    double speed = boid.mom.abs();

    if( speed > maxSpeed ) {
        boid.mom *= maxSpeed / speed;
    }

    // limit acceleration
    double accel = boid.accel.abs();

    if( accel > maxAccel ) {
        boid.accel *= maxAccel / accel;
    }
}

void Boids::iterate_all() {
    for( Boid& boid : boids_new ) {
        tp.add( [this, &boid] {
            boid.accel.reset();
            // follow the three boid rules
            threeRules( boid );

            if( fearEagle > std::numeric_limits<double>::epsilon() ) {
                fleeFromEagle( boid );
            }

            // some randomness
            boid.accel.x += distribution( generator );
            boid.accel.y += distribution( generator );

            // set new place
            borderConstraints( boid );
            boid.mom += boid.accel;
            boid.pos += boid.mom;
        } );
    }

    tp.waitForAllJobs();
    boids_old = boids_new;
}

void Boids::resize( int size ) {
    if( size <= 0 ) { size = 1; }

    int diff = size - boids_old.size();

    if( diff > 0 ) {
        boids_old.reserve( size );

        for( int i = 0; i < diff; ++i ) {
            boids_old.emplace_back( width, height ); // add needed
        }
    } else {
        boids_old.resize( size, Boid( 0, 0 ) ); // remove not needed
    }

    boids_new = boids_old;
}

