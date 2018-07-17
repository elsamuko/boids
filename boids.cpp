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
// g++ -O3 -msse2 -funroll-loops -o boids boids.cpp -lfltk -lpthread

//FLTK
#include <FL/Fl.H>
#include <FL/Fl_File_Icon.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Box.H>
#include <FL/fl_draw.H>
#include <FL/Fl_Value_Slider.H>

//REST
#include <cmath>
#include <iostream>
#include <chrono>

#include "threadpool.hpp"

//DEFINES
#define BG_COLOR 255
#define WIDTH 800
#define HEIGHT 600

using namespace std;

double sign( double num ) {
    return num / abs( num );
}

// simple 2D vector class
class Double {
    public:
        double x;
        double y;
        Double( double xIn = 0.0, double yIn = 0.0 ): x( xIn ), y( yIn ) {  }

        Double& operator+=( const Double& in ) {
            x += in.x;
            y += in.y;
            return *this;
        }

        Double& operator-=( const Double& in ) {
            x -= in.x;
            y -= in.y;
            return *this;
        }

        Double& operator*=( const Double& in ) {
            x *= in.x;
            y *= in.y;
            return *this;
        }

        Double& operator*=( const double& in ) {
            x *= in;
            y *= in;
            return *this;
        }

        double abs( )const {
            return sqrt( x * x + y * y );
        }

        double distance( const Double& b )const {
            double dx( x - b.x );
            double dy( y - b.y );
            return sqrt( dx * dx + dy * dy );
        }
};

inline Double operator+( Double const& a, Double const& b ) {
    Double result( a );
    result += b;
    return result;
}

inline Double operator-( Double const& a, Double const& b ) {
    Double result( a );
    result -= b;
    return result;
}

inline Double operator*( Double const& a, Double const& b )  {
    Double result( a );
    result *= b;
    return result;
}

inline Double operator*( Double const& a,  const double& b )  {
    Double result( a );
    result *= b;
    return result;
}

inline Double operator*( const double& a, Double const& b )  {
    Double result( b );
    result *= a;
    return result;
}

inline Double operator/( Double const& a, const double& b ) {
    return ( a * ( 1 / b ) );
}

// the individual
class Boid {
    public:
        Double pos; //position
        Double mom; //momentum

        Boid( double xIn,
              double yIn,
              double vxIn = 0,
              double vyIn = 0 ) {
            pos.x = xIn;
            pos.y = yIn;
            mom.x = vxIn;
            mom.y = vyIn;
        }

        Boid( ) {
            double max = double( RAND_MAX );
            pos.x  = rand() / max * WIDTH;
            pos.y  = HEIGHT; // start from the bottom
            mom.x = rand() / max * 10.0 - 5;
            mom.y = rand() / max * 10.0 - 5;
        }

        ~Boid() { }
};

// the group
class Boids {
    private:
        ThreadPool tp;
    public:
        vector<Boid> boids;
        vector<Boid> boids_new;
        vector< Boid >::iterator thread0_begin;
        vector< Boid >::iterator thread0_end;
        vector< Boid >::iterator thread1_begin;
        vector< Boid >::iterator thread1_end;
        Double eagle;
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

        // follow three rules: seperation, cohesion, alignment
        void threeRules( Boid& in ) {
            Double dpos;
            Double MA_DPOS;
            Double MT_DPOS;
            Double MW_MOM;
            double d  = 0;
            int i = 0;

            for( const Boid& boid : boids ) {
                dpos = in.pos - boid.pos;
                d = dpos.abs();

                if( d < sight ) {
                    // separation: steer to avoid crowding local flockmates
                    MA_DPOS -= dpos * ( -maxSpeed / sight * d + maxSpeed );
                    // cohesion: steer to move toward the average position of local flockmates
                    MT_DPOS += dpos;
                    // alignment: steer towards the average heading of local flockmates
                    MW_MOM += boid.mom;
                    ++i;
                }
            }

            in.mom -= MA_DPOS / ( c_moveAway * i );
            in.mom -= MT_DPOS / ( c_moveTo * i );
            in.mom += MW_MOM / c_moveWith;
        }

        // flee from the eagle
        void fleeFromEagle( Boid& in ) {
            Double dpos = in.pos - eagle;
            double d = dpos.abs();

            if( d < sight ) {
                in.mom += dpos * ( -maxSpeed / sight * d + maxSpeed ) * fearEagle;
            }
        }

        void borderConstraints( Boid& in, int speed ) {
            if( in.pos.x > WIDTH ) {
                in.pos.x = 0;
            }

            if( in.pos.y > HEIGHT ) {
                in.pos.y = 0;
            }

            if( in.pos.x < 0 ) {
                in.pos.x = WIDTH;
            }

            if( in.pos.y < 0 ) {
                in.pos.y = HEIGHT;
            }

            if( abs( in.mom.x ) > speed ) {
                in.mom.x = sign( in.mom.x ) * speed;
            }

            if( abs( in.mom.y ) > speed ) {
                in.mom.y = sign( in.mom.y ) * speed;
            }
        }

        void iterate_all() {
            for( Boid& boid : boids_new ) {
                tp.add( [this, &boid] {
                    double max = double( RAND_MAX );
                    // follow the three boid rules
                    threeRules( boid );
                    fleeFromEagle( boid );
                    borderConstraints( boid, maxSpeed );

                    // some randomness
                    boid.mom.x += 2 * ( rand() / max - 0.5 );
                    boid.mom.y += 2 * ( rand() / max - 0.5 );

                    // set new place
                    boid.pos += boid.mom;
                } );
            }

            tp.waitForAllJobs();
            boids = boids_new;
        }
};

class Canvas : public Fl_Box {
    private:
        Boids boids;
        std::chrono::system_clock::time_point clock;
        int n_of_iterations = 0;

        void draw() {
            Fl_Box::draw();

            // the flock
            fl_color( fl_rgb_color( 0 ) );

            for( vector< Boid >::iterator a = boids.boids.begin(); a != boids.boids.end(); ++a ) {
                fl_pie( a->pos.x - 2, a->pos.y - 2, 4, 4, 0, 360 );
            }

            // the eagle
            fl_color( fl_rgb_color( 50 ) );
            fl_pie( boids.eagle.x - 6, boids.eagle.y - 6, 12, 12, 0, 360 );

            boids.iterate_all();

            // measure time between 100 iterations
            if( ( ++n_of_iterations ) > 100 ) {
                std::chrono::system_clock::time_point tmp( std::chrono::system_clock::now() );
                cout << std::chrono::duration_cast<std::chrono::seconds>( tmp - clock ).count() << "ms" << endl;
                clock = tmp;
                n_of_iterations = 0;
            };
        }

        static void Timer( void* in ) {
            Canvas* o = ( Canvas* )in;
            o->redraw();
            Fl::repeat_timeout( 0.040, Timer, in );
        }

        // moveWith, moveTo, moveAway, maxSpeed, minDistance, sight
        void set_val( int key, double val ) {
            switch( key ) {
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
                    c = exp( ( val - 30.0 ) / 4 );
                    cout << "fear the eagle = " << c << endl;
                    boids.fearEagle = c;
                    break;

                default:
                    cout << "Huh, an error." << endl;
                    break;
            }
        }

        static void sl_moveWith_CB( Fl_Widget* w, void* v ) {
            int key = 0;
            //int key_void = *(int*)((( Fl_Value_Slider* )w )->user_data());
            double val = ( ( Fl_Value_Slider* )w )->value();
            cout << "key: " << key << "\tval: " << val << endl;
            ( ( Canvas* )v )->set_val( key, val );
        }
        static void sl_moveTo_CB( Fl_Widget* w, void* v ) {
            int key = 1;
            double val = ( ( Fl_Value_Slider* )w )->value();
            cout << "key: " << key << "\tval: " << val << endl;
            ( ( Canvas* )v )->set_val( key, val );
        }
        static void sl_moveAway_CB( Fl_Widget* w, void* v ) {
            int key = 2;
            double val = ( ( Fl_Value_Slider* )w )->value();
            cout << "key: " << key << "\tval: " << val << endl;
            ( ( Canvas* )v )->set_val( key, val );
        }
        static void sl_maxSpeed_CB( Fl_Widget* w, void* v ) {
            int key = 3;
            double val = ( ( Fl_Value_Slider* )w )->value();
            cout << "key: " << key << "\tval: " << val << endl;
            ( ( Canvas* )v )->set_val( key, val );
        }
        static void sl_minDistance_CB( Fl_Widget* w, void* v ) {
            int key = 4;
            double val = ( ( Fl_Value_Slider* )w )->value();
            cout << "key: " << key << "\tval: " << val << endl;
            ( ( Canvas* )v )->set_val( key, val );
        }
        static void sl_sight_CB( Fl_Widget* w, void* v ) {
            int key = 5;
            double val = ( ( Fl_Value_Slider* )w )->value();
            cout << "key: " << key << "\tval: " << val << endl;
            ( ( Canvas* )v )->set_val( key, val );
        }
        static void sl_boids_CB( Fl_Widget* w, void* v ) {
            int key = 6;
            double val = ( ( Fl_Value_Slider* )w )->value();
            cout << "key: " << key << "\tval: " << val << endl;
            ( ( Canvas* )v )->set_val( key, val );
        }

        static void sl_fearEagle_CB( Fl_Widget* w, void* v ) {
            int key = 7;
            double val = ( ( Fl_Value_Slider* )w )->value();
            cout << "key: " << key << "\tval: " << val << endl;
            ( ( Canvas* )v )->set_val( key, val );
        }

        // static void sl_val_CB( Fl_Widget *w, void* v ) {
        //     int key = *(int*)(( Fl_Value_Slider* )w )->user_data();
        //     double val = (( Fl_Value_Slider* )w )->value();
        //     cout << "key: " << key << "\tval: " << val << endl;
        //     ((Canvas*)v)->set_val( key, val );
        // }

    public:
        Canvas( int numOfBoids, int X, int Y, int W, int H, const char* L = 0 ) : Fl_Box( X, Y, W, H, L ) {
            double max = double( RAND_MAX );

            for( int i = 0; i < numOfBoids; ++i ) {
                double x  = rand() % w();
                double y  = h(); // start from the bottom
                double vx = rand() / max * 10.0 - 5;
                double vy = rand() / max * 10.0 - 5;
                boids.boids.push_back( Boid( x, y, vx, vy ) );
            }

            boids.boids_new = boids.boids;
            box( FL_FLAT_BOX );
            color( BG_COLOR );
            Fl::add_timeout( 0.05, Timer, ( void* )this );

            Fl_Value_Slider* sl_moveWith = new Fl_Value_Slider( 0, HEIGHT, WIDTH / 3, 20, "move with" );
            sl_moveWith->type( 1 );
            sl_moveWith->callback( ( Fl_Callback* )sl_moveWith_CB, ( void* )this );
            sl_moveWith->round( 0.1 );
            sl_moveWith->range( -20.0, 10.0 );
            sl_moveWith->value( 3.0 );

            Fl_Value_Slider* sl_moveTo = new Fl_Value_Slider( WIDTH / 3, HEIGHT, WIDTH / 3, 20, "move to" );
            sl_moveTo->type( 1 );
            sl_moveTo->callback( ( Fl_Callback* )sl_moveTo_CB, ( void* )this );
            sl_moveTo->round( 0.1 );
            sl_moveTo->range( 0.0, 4.0 );
            sl_moveTo->value( 2.0 );

            Fl_Value_Slider* sl_moveAway = new Fl_Value_Slider( 2 * WIDTH / 3, HEIGHT, WIDTH / 3, 20, "move away" );
            sl_moveAway->type( 1 );
            sl_moveAway->callback( ( Fl_Callback* )sl_moveAway_CB, ( void* )this );
            sl_moveAway->round( 0.1 );
            sl_moveAway->range( 0.0, 20.0 );
            sl_moveAway->value( 10.0 );

            Fl_Value_Slider* sl_maxSpeed = new Fl_Value_Slider( 0, HEIGHT + 60, WIDTH / 3, 20, "max speed" );
            sl_maxSpeed->type( 1 );
            sl_maxSpeed->callback( ( Fl_Callback* )sl_maxSpeed_CB, ( void* )this );
            sl_maxSpeed->round( 0.1 );
            sl_maxSpeed->range( 0.0, 20.0 );
            sl_maxSpeed->value( 5.0 );

            Fl_Value_Slider* sl_minDistance = new Fl_Value_Slider( WIDTH / 3, HEIGHT + 60, WIDTH / 3, 20, "min dist" );
            sl_minDistance->type( 1 );
            sl_minDistance->callback( ( Fl_Callback* )sl_minDistance_CB, ( void* )this );
            sl_minDistance->round( 1.0 );
            sl_minDistance->range( 1.0, 200.0 );
            sl_minDistance->value( 30.0 );

            Fl_Value_Slider* sl_sight = new Fl_Value_Slider( 2 * WIDTH / 3, HEIGHT + 60, WIDTH / 3, 20, "sight" );
            sl_sight->type( 1 );
            sl_sight->callback( ( Fl_Callback* )sl_sight_CB, ( void* )this );
            sl_sight->round( 1.0 );
            sl_sight->range( 1.0, ( double )WIDTH );
            sl_sight->value( 300.0 );

            Fl_Value_Slider* sl_boids = new Fl_Value_Slider( 0, HEIGHT + 110, WIDTH / 2, 20, "number of boids" );
            sl_boids->type( 1 );
            sl_boids->precision( 0 );
            sl_boids->callback( ( Fl_Callback* )sl_boids_CB, ( void* )this );
            sl_boids->round( 1 );
            sl_boids->range( 1, 2000 );
            sl_boids->value( 400.0 );

            Fl_Value_Slider* sl_fearEagle = new Fl_Value_Slider( WIDTH / 2, HEIGHT + 110, WIDTH / 2, 20, "fear of eagle" );
            sl_fearEagle->type( 1 );
            sl_fearEagle->callback( ( Fl_Callback* )sl_fearEagle_CB, ( void* )this );
            sl_fearEagle->round( 0.1 );
            sl_fearEagle->range( 0.0, 15.0 );
            sl_fearEagle->value( 5.0 );

            clock = std::chrono::system_clock::now();
            redraw();
        }

        int handle( int e ) {
            int ret = Fl_Box::handle( e );

            switch( e ) {
                case FL_MOVE:
                    boids.eagle = Double( Fl::event_x(), Fl::event_y() );
                    break;
            }

            return( ret );
        }
};

int main( int /*argc*/, char** /*argv*/ ) {
    srand( time( NULL ) );
    Fl_Double_Window win( WIDTH, HEIGHT + 150, "Boids" );
    Canvas can( 400, 0, 0, WIDTH, HEIGHT );
    win.show();
    int rv = Fl::run();
    return rv;
}

// EOF
