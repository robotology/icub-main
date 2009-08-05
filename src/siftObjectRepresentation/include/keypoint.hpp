// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Authors: 2008 Dario Figueira
 *          Bruno Damas
 *      Institute for Systems and Robotics (IST)
 *      IST, TULisbon, Portugal
 */


#ifndef __KEYPOINT_HPP__B
#define __KEYPOINT_HPP__B

#include <ostream>
using namespace std;

class Keypoint {
    friend ostream& operator<<(ostream& output, const Keypoint& k);
    
    public:
        Keypoint( Keypoint const &other );
        Keypoint &operator=( Keypoint const &other );
        Keypoint();
        ~Keypoint();

    
        static void set_descriptor_size( int size );
        static int get_descriptor_size();

        void set_descriptor( float descriptor[]);
        float const *get_descriptor() const;

        float distance( Keypoint const &kp);
        float square_distance( Keypoint const &kp);

        void set_pos(int octave = 0, int scale = 0, int row = 0, int col = 0);
    //		Keypoint(int octave, int scale, int row, int col, float theta = 0);
    //		void set_theta( float theta );

        void data_dump();
        float sum_desc();

        int octave;
        int scale;
        int row;
        int col;

        float s;
        float x;
        float y;
        float th;

    /*		float contrast_value;
        float edge_ratio_value;*/

    private:
        static int descriptor_size;
        static bool initialized;

        float *descriptor;

        void copy( Keypoint const &other );
        void destroy();
};


#endif
