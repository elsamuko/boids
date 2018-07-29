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
        Canvas( int X, int Y, int W, int H, const char* L = 0 );

    private:
        void draw();

        static void Timer( void* in );

        int handle( int event );

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
            boids.maxSpeed = val / 20.0;
            LOG( "Max Speed: " << boids.maxSpeed );
        }
        void setMaxAccel( int val ) {
            boids.maxAccel = val / 1000.0;
            LOG( "Max Accel: " << boids.maxAccel );
        }
        void setSight( int val ) {
            boids.sight = val;
            LOG( "Sight: " << boids.sight );
        }
        void setBoids( int val ) {
            boids.boids_old.resize( val );
            boids.boids_new.resize( val );
            LOG( "Boids: " << val );
        }
        void setFearEagle( int val ) {
            boids.fearEagle = val / 100.0;
            LOG( "Fear Eagle: " << boids.fearEagle );
        }
        void setDebug( bool debug ) {
            boids.debug = debug;
            LOG( "Debug: " << boids.debug );
        }


    private:
        Boids boids;
        int iterations = 0;
};
