#pragma once

#include <cmath>
#include <iostream>
#include <random>

#include "threadpool.hpp"
#include "vec2d.hpp"

#define LOG( A ) do{ std::cout.width(10);  std::cout << __func__ << " : " << A << std::endl; } while( false )

// the individual
class Boid {
    public:
        vec2D pos;      // position
        vec2D mom;      // momentum
        vec2D accel;    // momentum change
        vec2D moveTo;   // center of visible boids
        vec2D moveAway; // direction to avoid crowding
        vec2D fear;     // direction to flee from eagle

        Boid( double xIn,
              double yIn,
              double vxIn = 0,
              double vyIn = 0 ) {
            pos.x = xIn;
            pos.y = yIn;
            mom.x = vxIn;
            mom.y = vyIn;
        }

        Boid( int width, int height ) {
            double max = double( RAND_MAX );
            pos.x  = rand() / max * width;
            pos.y  = rand() / max * height;
            mom.x = rand() / max * 10.0 - 5;
            mom.y = rand() / max * 10.0 - 5;
        }
};

inline std::ostream& operator<<( std::ostream& os, const Boid& boid ) {
    os << boid.pos << " " << boid.mom.abs() << " " << boid.accel.abs();
    return os;
}

// the group
class Boids {
    private:
        ThreadPool tp;
        std::default_random_engine generator;
        std::normal_distribution<double> distribution;
    public:
        std::vector<Boid> boids_old;
        std::vector<Boid> boids_new;
        vec2D eagle;
        bool   debug;
        int    width;
        int    height;
        double c_moveWith;
        double c_moveTo;
        double c_moveAway;
        double maxSpeed;
        double maxAccel;
        double sight;
        double fearEagle;

        Boids( int widthIn,
               int heightIn,
               int countIn            = 200,
               double c_moveWithIn  = 1.0,
               double c_moveToIn    = 1.0,
               double c_moveAwayIn  = 1.0,
               double maxSpeedIn    = 5,
               double maxAccel      = 0.5,
               double sightIn       = 200,
               double fearEagleIn   = 0.0 );

        // follow three rules: seperation, cohesion, alignment
        void threeRules( Boid& boid );
        void fleeFromEagle( Boid& in );
        void borderConstraints( Boid& in );
        void iterate_all();
        void resize( int size );
};
