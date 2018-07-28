#pragma once

//FLTK
#include <FL/Fl.H>
#include <FL/Fl_File_Icon.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Box.H>
#include <FL/fl_draw.H>
#include <FL/Fl_Value_Slider.H>
#include <FL/Fl_Spinner.H>

#include "boids.hpp"

class Canvas : public Fl_Box {
    public:
        Canvas( int X, int Y, int W, int H, const char* L = 0 ) : Fl_Box( X, Y, W, H, L ) {
            boids.boids_old.resize( 200 );
            boids.boids_new = boids.boids_old;
            redraw();
            Fl::add_timeout( 0.05, Timer, ( void* )this );
        }

    private:
        void draw();

        static void Timer( void* in ) {
            Canvas* canvas = ( Canvas* )in;
            canvas->redraw();
            Fl::repeat_timeout( 0.05, Timer, in );
        }

        int handle( int event ) {

            if( event == FL_MOVE ) {
                boids.eagle = vec2D( Fl::event_x(), Fl::event_y() );
            }

            return Fl_Box::handle( event );
        }

    public:
        void setMoveWith( int val ) {
            boids.c_moveWith = val / 100.0;
            LOG( "Move With: " << boids.c_moveWith );
        }
        void setMoveTo( int val ) {
            boids.c_moveTo = val / 100.0;
            LOG( "Move To: " << boids.c_moveTo );
        }
        void setMoveAway( int val ) {
            boids.c_moveAway = val / 100.0;
            LOG( "Move Away: " << boids.c_moveAway );
        }
        void setMaxSpeed( int val ) {
            LOG( "Max Speed: " << val );
            boids.maxSpeed = val;
        }
        void setSight( int val ) {
            LOG( "Sight: " << val );
            boids.sight = val;
        }
        void setBoids( int val ) {
            LOG( "Boids: " << val );
            boids.boids_old.resize( val );
            boids.boids_new.resize( val );
        }
        void setFearEagle( int val ) {
            LOG( "Fear Eagle: " << val );
            boids.fearEagle = val;
        }


    private:
        Boids boids;
        int iterations = 0;
};
