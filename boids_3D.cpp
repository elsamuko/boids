/**************************************************************************
*   Copyright (C) 2009 by elsamuko                                        *
*   elsamuko@gmail.com                                                    *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or     *
*   (at your option) any later version.                                   *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU General Public License for more details.                          *
*                                                                         *
*   You should have received a copy of the GNU General Public License     *
*   along with this program; if not, write to the                         *
*   Free Software Foundation, Inc.,                                       *
*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
***************************************************************************/

// http://en.wikipedia.org/wiki/Boids
// boid simulation, compile with (needs libfltk-dev):
// g++ -O3 -Wall -funroll-loops -o boids_3D boids_3D.cpp -lfltk -lpthread


//THREADS
#include <pthread.h>   /* Posix 1003.1c threads */

//FLTK
#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Box.H>
#include <FL/fl_draw.H>
#include <FL/Fl_Value_Slider.H>

//BOOST
#include "boost/date_time/posix_time/posix_time.hpp"

//REST
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <time.h>
#include <vector>
#include <string>
#include <sstream>

//DEFINES
#define BG_COLOR 255
#define WIDTH 800
#define HEIGHT 600
#define DEPTH 800

using namespace std;
using namespace boost::posix_time;


ptime clock_gl;
int n_of_iterations = 0;

void *iterate_ext0( void* arg );
void *iterate_ext1( void* arg );

string toString( double val ) {
    ostringstream o;
    o << val;
    return o.str() ;
}

double sign( double num ) {
    return num / abs( num );
}

//simple 3D vector class
class Triple {
public:
    double x;
    double y;
    double z;
    Triple(): x( 0 ), y( 0 ), z( 0 ) { }
    Triple( double xIn, double yIn, double zIn ): x( xIn ), y( yIn ), z( zIn ) {  }
    Triple( double xIn ): x( xIn ), y( xIn ), z( xIn ) {  }

    Triple& operator+=( const Triple& in ) {
        x += in.x;
        y += in.y;
        z += in.z;
        return *this;
    }

    Triple& operator-=( const Triple& in ) {
        x -= in.x;
        y -= in.y;
        z -= in.z;
        return *this;
    }

    Triple& operator*=( const Triple& in ) {
        x *= in.x;
        y *= in.y;
        z *= in.z;
        return *this;
    }

    Triple& operator*=( const double& in ) {
        x *= in;
        y *= in;
        z *= in;
        return *this;
    }

    double abs( )const {
        return sqrt( x*x + y*y + z*z );
    }

    double distance( const Triple& b )const {
        double dx (x - b.x);
        double dy (y - b.y);
        double dz (z - b.z);
        return sqrt( dx*dx + dy*dy + dz*dz );
    }
};

inline Triple operator+( Triple const& a, Triple const& b ) {
    Triple result (a);
    result += b;
    return result;
}

inline Triple operator-( Triple const& a, Triple const& b ) {
    Triple result (a);
    result -= b;
    return result;
}

inline Triple operator*( Triple const& a, Triple const& b )  {
    Triple result (a);
    result *= b;
    return result;
}

inline Triple operator*( Triple const& a,  const double& b )  {
    Triple result (a);
    result *= b;
    return result;
}

inline Triple operator*( const double& a, Triple const& b )  {
    Triple result (b);
    result *= a;
    return result;
}

inline Triple operator/( Triple const& a, const double& b ) {
    return ( a * ( 1 / b ) );
};

// the individual
class Boid {
public:
    Triple pos; //position
    Triple mom; //momentum

    Boid( double xIn,
          double yIn,
          double zIn,
          double vxIn = 0,
          double vyIn = 0,
          double vzIn = 0 ) {
        pos.x = xIn;
        pos.y = yIn;
        pos.z = zIn;
        mom.x = vxIn;
        mom.y = vyIn;
        mom.z = vzIn;
    }

    Boid( ) {
        double max = double( RAND_MAX );
        pos.x  = rand() / max * WIDTH;
        pos.y  = HEIGHT; // start from the bottom
        pos.z  = rand() / max * DEPTH;
        mom.x = rand() / max * 10.0 - 5;
        mom.y = rand() / max * 10.0 - 5;
        mom.z = rand() / max * 10.0 - 5;
    }

    ~Boid() { }
};

// the group
class Boids {
public:
    vector<Boid> boids;
    vector<Boid> boids_new;
    vector< Boid >::iterator thread0_begin;
    vector< Boid >::iterator thread0_end;
    vector< Boid >::iterator thread1_begin;
    vector< Boid >::iterator thread1_end;
    Boid eagle;
    Boid victim;
    double c_moveWith;
    double c_moveTo;
    double c_moveAway;
    double maxSpeed;
    double minDistance;
    double sight;
    double fearEagle;

    Boids( double c_moveWithIn = 8000,
           double c_moveToIn = 200,
           double c_moveAwayIn = 60000,
           double maxSpeedIn = 5,
           double minDistanceIn = 20,
           double sightIn = 300,
           double fearEagleIn = 0.0019305 ) {
        c_moveWith = c_moveWithIn;
        c_moveTo = c_moveToIn;
        c_moveAway = c_moveAwayIn;
        maxSpeed = maxSpeedIn;
        minDistance = minDistanceIn;
        sight = sightIn;
        fearEagle = fearEagleIn;
    }

    ~Boids() { }

    // separation: steer to avoid crowding local flockmates
    void moveAway( Boid& in ) {
        Triple dpos;
        Triple DPOS;
        double d  = 0;
        int i = 0;
        for ( vector< Boid >::iterator a = boids.begin(); a != boids.end(); ++a ) {
            dpos = in.pos - a->pos;
            d = dpos.abs();
            if ( d < sight ) {
                //linear speed change:
                // dpos(sight) = 0;
                // dpos(0)     = maxspeed;
                DPOS -= dpos * ( -maxSpeed / sight * d + maxSpeed );
                ++i;
            }
        }
        in.mom -= DPOS / ( c_moveAway * i );
    }

    // cohesion: steer to move toward the average position of local flockmates
    void moveTo( Boid& in ) {
        Triple DPOS;
        int i = 0;
        for ( vector< Boid >::iterator a = boids.begin(); a != boids.end(); ++a ) {
            if ( in.pos.distance( a->pos ) < sight ) {
                DPOS += in.pos - a->pos;
                ++i;
            }
        }
        in.mom -= DPOS / ( c_moveTo * i );
    }

    // alignment: steer towards the average heading of local flockmates
    void moveWith( Boid& in ) {
        Triple MOM;
        for ( vector< Boid >::iterator a = boids.begin(); a != boids.end(); ++a ) {
            if ( in.pos.distance( a->pos ) < sight ) {
                MOM += a->mom;
            }
        }
        in.mom += MOM / c_moveWith;
    }

    // flee from the eagle
    void fleeFromEagle( Boid& in ) {
        Triple dpos = in.pos - eagle.pos;
        double d = dpos.abs();
        if ( d < sight ) {
            in.mom += dpos * ( -maxSpeed / sight * d + maxSpeed ) * fearEagle;
        }
    }

    // the eagle picks one out and follows him
    void followSingle( Boid& in ) {
        double c = 0.008;
        Triple dpos = in.pos - eagle.pos;;
        double d = dpos.abs();
        // eagle can see twice as far
        if ( d < 2*sight ) {
            eagle.mom += dpos * c;
        }
    }

    void borderConstraints( Boid& in, int speed ) {
        if ( in.pos.x > WIDTH ) {
            //in.mom.x = -in.mom.x;
            in.pos.x = 0;
        }
        if ( in.pos.y > HEIGHT ) {
            //in.mom.y = -in.mom.y;
            in.pos.y = 0;
        }
        if ( in.pos.z > DEPTH ) {
            //in.mom.z = -in.mom.z;
            in.pos.z = 0;
        }

        if ( in.pos.x < 0 ) {
            //in.mom.x = -in.mom.x;
            in.pos.x = WIDTH;
        }
        if ( in.pos.y < 0 ) {
            //in.mom.y = -in.mom.y;
            in.pos.y = HEIGHT;
        }
        if ( in.pos.z < 0 ) {
            //in.mom.z = -in.mom.z;
            in.pos.z = DEPTH;
        }

        if ( abs( in.mom.x ) > speed ) in.mom.x = sign( in.mom.x ) * speed;
        if ( abs( in.mom.y ) > speed ) in.mom.y = sign( in.mom.y ) * speed;
        if ( abs( in.mom.z ) > speed ) in.mom.z = sign( in.mom.z ) * speed;
    }

    void iterate_flock( vector< Boid >::iterator begin, vector< Boid >::iterator end ) {
        double max = double( RAND_MAX );
        for ( vector< Boid >::iterator a = begin; a != end; ++a ) {

            // follow the three boid rules
            moveAway( *a );
            moveTo( *a );
            moveWith( *a );
            fleeFromEagle( *a );
            borderConstraints( *a , maxSpeed );

            // some randomness
            a->mom.x += 2 * ( rand() / max - 0.5 );
            a->mom.y += 2 * ( rand() / max - 0.5 );
            a->mom.z += 2 * ( rand() / max - 0.5 );

            // set new place
            a->pos += a->mom;
        }
    }

    void iterate_all() {
        double max = double( RAND_MAX );

        //the eagle follows the first bird
        double rnd = rand()/max;
        if ( rnd < 0.05 ) {
            int choice = floor( 20 * rnd * boids.size() );
            victim = boids[choice];
        }
        followSingle( victim );
        borderConstraints( eagle , 1.5*maxSpeed );
        eagle.pos += eagle.mom;

        //iterate the flock in two threads
        thread0_begin = boids_new.begin();
        thread0_end = thread0_begin + boids_new.size() / 2;
        thread1_begin = thread0_end + 1;
        thread1_end = boids_new.end();

        pthread_t thread[2];

        //call threads
        pthread_create( &thread[0], NULL, iterate_ext0, this );
        pthread_create( &thread[1], NULL, iterate_ext1, this );

        //wait for threads to end
        pthread_join( thread[0], NULL );
        pthread_join( thread[1], NULL );

        //iterate_flock(thread0_begin, thread0_end);
        //iterate_flock(thread1_begin, thread1_end);

        boids = boids_new;
    }
};

void *iterate_ext0( void* arg ) {
    Boids* b = ( Boids* )arg;
    b->iterate_flock( b->thread0_begin, b->thread0_end );
    return( 0 );
}

void *iterate_ext1( void* arg ) {
    Boids* b = ( Boids* )arg;
    b->iterate_flock( b->thread1_begin, b->thread1_end );
    return( 0 );
}

class Canvas : public Fl_Box {
private:
    Boids boids;

    void draw() {
        Fl_Box::draw();

        //the flock
        for ( vector< Boid >::iterator a = boids.boids.begin(); a != boids.boids.end(); ++a ) {
            fl_color( fl_rgb_color( a->pos.z / DEPTH*220 ) );
            fl_pie( a->pos.x - 3, a->pos.y - 3, 6, 6, 0, 360 );
        }
        //the eagle
        fl_color( fl_rgb_color( boids.eagle.pos.z / DEPTH*255 ) );
        fl_pie( boids.eagle.pos.x - 6, boids.eagle.pos.y - 6, 12, 12, 0, 360 );

        boids.iterate_all();

        //measure time between 100 iterations
        if (( ++n_of_iterations ) > 100 ) {
            ptime tmp (microsec_clock::universal_time());
            cout << to_simple_string( tmp - clock_gl ) << endl;
            clock_gl = tmp;
            n_of_iterations = 0;
        };
    }

    static void Timer( void *in ) {
        Canvas *o = ( Canvas* )in;
        o->redraw();
        Fl::repeat_timeout( 0.040, Timer, in );
    }

    // moveWith, moveTo, moveAway, maxSpeed, minDistance, sight
    void set_val( int key, double val ) {
        switch ( key ) {
            double c;
        case 0:
            c = 1 / exp( val - 9 );
            cout << "move with = " << c << endl;
            boids.c_moveWith = c;
            break;
        case 1:
            c = 1 / exp( val - 7 );
            cout << "move to = " << c << endl;
            boids.c_moveTo = c;
            break;
        case 2:
            c = 1 / exp( val - 20 );
            cout << "move away = " << c << endl;
            boids.c_moveAway = c;
            break;
        case 3:
            c = val;
            cout << "max speed = " << c << endl;
            boids.maxSpeed = c;
            break;
        case 4:
            c = val;
            cout << "min distance = " << c << endl;
            boids.minDistance = c;
            break;
        case 5:
            c = val;
            cout << "sight = " << c << endl;
            boids.sight = c;
            break;
        case 6:
            c = val;
            cout << "number of boids = " << c << endl;
            boids.boids.resize( c );
            boids.boids_new.resize( c );
            break;
        case 7:
            c = exp(( val - 30.0 ) / 4 );
            cout << "fear the eagle = " << c << endl;
            boids.fearEagle = c;
            break;
        default:
            cout << "Huh, an error." << endl;
            break;
        }
    }

    static void sl_moveWith_CB( Fl_Widget *w, void* v ) {
        int key = 0;
        //int key_void = *(int*)((( Fl_Value_Slider* )w )->user_data());
        double val = (( Fl_Value_Slider* )w )->value();
        cout << "key: " << key << "\tval: " << val << endl;
        (( Canvas* )v )->set_val( key, val );
    }
    static void sl_moveTo_CB( Fl_Widget *w, void* v ) {
        int key = 1;
        double val = (( Fl_Value_Slider* )w )->value();
        cout << "key: " << key << "\tval: " << val << endl;
        (( Canvas* )v )->set_val( key, val );
    }
    static void sl_moveAway_CB( Fl_Widget *w, void* v ) {
        int key = 2;
        double val = (( Fl_Value_Slider* )w )->value();
        cout << "key: " << key << "\tval: " << val << endl;
        (( Canvas* )v )->set_val( key, val );
    }
    static void sl_maxSpeed_CB( Fl_Widget *w, void* v ) {
        int key = 3;
        double val = (( Fl_Value_Slider* )w )->value();
        cout << "key: " << key << "\tval: " << val << endl;
        (( Canvas* )v )->set_val( key, val );
    }
    static void sl_minDistance_CB( Fl_Widget *w, void* v ) {
        int key = 4;
        double val = (( Fl_Value_Slider* )w )->value();
        cout << "key: " << key << "\tval: " << val << endl;
        (( Canvas* )v )->set_val( key, val );
    }
    static void sl_sight_CB( Fl_Widget *w, void* v ) {
        int key = 5;
        double val = (( Fl_Value_Slider* )w )->value();
        cout << "key: " << key << "\tval: " << val << endl;
        (( Canvas* )v )->set_val( key, val );
    }
    static void sl_boids_CB( Fl_Widget *w, void* v ) {
        int key = 6;
        double val = (( Fl_Value_Slider* )w )->value();
        cout << "key: " << key << "\tval: " << val << endl;
        (( Canvas* )v )->set_val( key, val );
    }

    static void sl_fearEagle_CB( Fl_Widget *w, void* v ) {
        int key = 7;
        double val = (( Fl_Value_Slider* )w )->value();
        cout << "key: " << key << "\tval: " << val << endl;
        (( Canvas* )v )->set_val( key, val );
    }

    // static void sl_val_CB( Fl_Widget *w, void* v ) {
    //     int key = *(int*)(( Fl_Value_Slider* )w )->user_data();
    //     double val = (( Fl_Value_Slider* )w )->value();
    //     cout << "key: " << key << "\tval: " << val << endl;
    //     ((Canvas*)v)->set_val( key, val );
    // }

public:
    Canvas( int numOfBoids, int X, int Y, int W, int H, const char*L = 0 ) : Fl_Box( X, Y, W, H, L ) {
        // set up boid crowd
        boids = Boids();
        boids.eagle = Boid( WIDTH / 2, HEIGHT / 2, DEPTH / 2, 0, 0 );
        double max = double( RAND_MAX );
        for ( int i = 0; i < numOfBoids; ++i ) {
            double x  = rand() % w();
            double y  = h(); // start from the bottom
            double z  = rand() % DEPTH;
            double vx = rand() / max * 10.0 - 5;
            double vy = rand() / max * 10.0 - 5;
            double vz = rand() / max * 10.0 - 5;
            boids.boids.push_back( Boid( x, y, z, vx, vy, vz ) );
        }
        boids.boids_new = boids.boids;
        box( FL_FLAT_BOX );
        color( BG_COLOR );
        Fl::add_timeout( 0.05, Timer, ( void* )this );

        int pos = 0;
        Fl_Value_Slider* sl_moveWith = new Fl_Value_Slider( 0, HEIGHT, WIDTH / 3, 20, "move with" );
        sl_moveWith->type( 1 );
        sl_moveWith->callback(( Fl_Callback* )sl_moveWith_CB , ( void* )this );
        sl_moveWith->round( 0.1 );
        sl_moveWith->range( -20.0, 10.0 );
        sl_moveWith->value( 3.0 );

        pos++;
        Fl_Value_Slider* sl_moveTo = new Fl_Value_Slider( WIDTH / 3, HEIGHT, WIDTH / 3, 20, "move to" );
        sl_moveTo->type( 1 );
        sl_moveTo->callback(( Fl_Callback* )sl_moveTo_CB, ( void* )this );
        sl_moveTo->round( 0.1 );
        sl_moveTo->range( 0.0, 4.0 );
        sl_moveTo->value( 2.0 );

        pos++;
        Fl_Value_Slider* sl_moveAway = new Fl_Value_Slider( 2*WIDTH / 3, HEIGHT, WIDTH / 3, 20, "move away" );
        sl_moveAway->type( 1 );
        sl_moveAway->callback(( Fl_Callback* )sl_moveAway_CB, ( void* )this );
        sl_moveAway->round( 0.1 );
        sl_moveAway->range( 0.0, 20.0 );
        sl_moveAway->value( 10.0 );

        pos++;
        Fl_Value_Slider* sl_maxSpeed = new Fl_Value_Slider( 0, HEIGHT + 60, WIDTH / 3, 20, "max speed" );
        sl_maxSpeed->type( 1 );
        sl_maxSpeed->callback(( Fl_Callback* )sl_maxSpeed_CB , ( void* )this );
        sl_maxSpeed->round( 0.1 );
        sl_maxSpeed->range( 0.0, 20.0 );
        sl_maxSpeed->value( 5.0 );

        pos++;
        Fl_Value_Slider* sl_minDistance = new Fl_Value_Slider( WIDTH / 3, HEIGHT + 60, WIDTH / 3, 20, "min dist" );
        sl_minDistance->type( 1 );
        sl_minDistance->callback(( Fl_Callback* )sl_minDistance_CB, ( void* )this );
        sl_minDistance->round( 1.0 );
        sl_minDistance->range( 1.0, 200.0 );
        sl_minDistance->value( 30.0 );

        pos++;
        Fl_Value_Slider* sl_sight = new Fl_Value_Slider( 2*WIDTH / 3, HEIGHT + 60, WIDTH / 3, 20, "sight" );
        sl_sight->type( 1 );
        sl_sight->callback(( Fl_Callback* )sl_sight_CB, ( void* )this );
        sl_sight->round( 1.0 );
        sl_sight->range( 1.0, ( double )WIDTH );
        sl_sight->value( 300.0 );

        pos++;
        Fl_Value_Slider* sl_boids = new Fl_Value_Slider( 0, HEIGHT + 110, WIDTH / 2 , 20, "number of boids" );
        sl_boids->type( 1 );
        sl_boids->precision( 0 );
        sl_boids->callback(( Fl_Callback* )sl_boids_CB, ( void* )this );
        sl_boids->round( 1 );
        sl_boids->range( 1, 2000 );
        sl_boids->value( 400.0 );

        pos++;
        Fl_Value_Slider* sl_fearEagle = new Fl_Value_Slider( WIDTH / 2, HEIGHT + 110, WIDTH / 2 , 20, "fear of eagle" );
        sl_fearEagle->type( 1 );
        sl_fearEagle->callback(( Fl_Callback* )sl_fearEagle_CB, ( void* )this );
        sl_fearEagle->round( 0.1 );
        sl_fearEagle->range( 0.0, 15.0 );
        sl_fearEagle->value( 5.0 );

        clock_gl = microsec_clock::universal_time();
        redraw();
    }

    ~Canvas() {
    }

};

int main( int argc, char *argv[] ) {
    srand( time( NULL ) );
    Fl_Double_Window win( WIDTH, HEIGHT + 150, "Boids" );
    //win.size_range(400, 200);
    Canvas can( 400, 0, 0, WIDTH, HEIGHT );
    win.show();
    return( Fl::run() );
}

// EOF
