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

Boids::Boids( double c_moveWithIn, double c_moveToIn, double c_moveAwayIn, double maxSpeedIn, double maxAccel, double sightIn, double fearEagleIn ) :
    generator( std::chrono::system_clock::now().time_since_epoch().count() ),
    distribution( 0.0, maxSpeedIn / 25.0 ),
    c_moveWith( c_moveWithIn ),
    c_moveTo( c_moveToIn ),
    c_moveAway( c_moveAwayIn ),
    maxSpeed( maxSpeedIn ),
    maxAccel( maxAccel ),
    sight( sightIn ),
    fearEagle( fearEagleIn ) {}

void Boids::threeRules( Boid& boid ) {
    vec2D dpos; // delta position
    vec2D ddir; // delta direction (normalized position)
    vec2D dmom; // normalized momentum
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
            dmom = other.mom.norm();

            // separation: steer to avoid crowding local flockmates
            // closer ones repulse stronger
            MA_DPOS -= ddir * fastPow( distance / 10.0, -1.0 );

            // cohesion: steer to move toward the average position of local flockmates
            MT_DPOS += other.pos;

            // alignment: steer towards the average heading of local flockmates
            MW_MOM += dmom;
        }
    }

    // correct boids acceleration
    if( i > 0 ) {
        boid.moveAway = 200 * MA_DPOS / i;
        boid.accel += c_moveAway * boid.moveAway;

        boid.moveTo = 0.13 * ( ( MT_DPOS / i ) - boid.pos );
        boid.accel += c_moveTo * boid.moveTo;

        boid.accel += c_moveWith * 2.0 * MW_MOM  / i;
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
    if( boid.pos.x > WIDTH ) {
        boid.pos.x = 0;
    }

    if( boid.pos.y > HEIGHT ) {
        boid.pos.y = 0;
    }

    if( boid.pos.x < 0 ) {
        boid.pos.x = WIDTH;
    }

    if( boid.pos.y < 0 ) {
        boid.pos.y = HEIGHT;
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

