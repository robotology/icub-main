// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Authors: 2008 Dario Figueira
 *          Bruno Damas
 *      Institute for Systems and Robotics (IST)
 *      IST, TULisbon, Portugal
 */

#include "keypoint.hpp"
#include "common.hpp"

#include <string>
#include <iostream>
#include <cmath>
using namespace std;

/********  Static ********/

int Keypoint::descriptor_size = -1;

bool Keypoint::initialized = false;

void Keypoint::set_descriptor_size( int size ) {
    initialized = true;
    descriptor_size = size;
}

int Keypoint::get_descriptor_size() {
    //	if( !initialized )
    //	{
    //		cerr << "Argh! Descriptor size not initialized! Exiting!" << endl;
    //		system("PAUSE");
    //		exit(1);
    //	}

    return descriptor_size;
}

/********  Public ********/


void Keypoint::copy( Keypoint const &other ) {
    octave = other.octave;
    scale = other.scale;
    row = other.row;
    col = other.col;

    s = other.s;
    x = other.x;
    y = other.y;
    th = other.th;

    // 	contrast_value = other.contrast_value;
    // 	edge_ratio_value = other.edge_ratio_value;


    if( (descriptor = other.descriptor) == 0 )
        return;

    descriptor = new float[descriptor_size];
    for(int i = 0; i < descriptor_size; i++)
        descriptor[i] = other.descriptor[i];
}


void Keypoint::destroy() {
    if( descriptor != 0 )
        delete[] descriptor;
}


Keypoint::Keypoint( Keypoint const &other ) {
    copy( other );
}

Keypoint &Keypoint::operator=( Keypoint const &other ) {
    if (this != &other) {
        destroy();
        copy(other);
    }

    return *this;
}

ostream& operator<<(ostream& output, const Keypoint& k) {
    output <<  k.octave << " " << k.scale <<" "<< k.row <<" "<< k.col <<" "<< k.s <<" "<< k.x <<" "<< k.y <<" "<< k.th <<" ";
    return output;  // for multiple << operators.
}

Keypoint::Keypoint() {
    descriptor = 0;
}

Keypoint::~Keypoint() {
    destroy();
}

void Keypoint::set_pos(int o, int s, int r, int c) {
    octave = o;
    scale = s;
    row = r;
    col = c;
}

/*
Keypoint::Keypoint(int o, int s, int r, int c, float theta)
{
	octave = o;
	scale = s;
	row = r;
	col = c;
	th = theta;
 
	descriptor = 0;
}
 
inline void Keypoint::set_theta( float theta )
{
	th = theta;
}
*/

void Keypoint::set_descriptor( float desc[]) {
    //	if( !initialized )
    //	{
    //		cerr << "Argh! Descriptor size not initialized! Exiting!" << endl;
    //		system("PAUSE");
    //		exit(1);
    //	}

    if( descriptor == 0)
        descriptor = new float[descriptor_size];

    memcpy(descriptor, desc, descriptor_size * sizeof(float));
}


float const *Keypoint::get_descriptor() const {
    return descriptor;
}


float Keypoint::square_distance( Keypoint const &kp) {
    float dist = 0.0;

    for( int i = 0; i < descriptor_size; i++ )
        dist += SQUARE(descriptor[i]-kp.descriptor[i]);

    return dist;
}

float Keypoint::distance( Keypoint const &kp) {
    float dist = 0.0;

    for( int i = 0; i < descriptor_size; i++ )
        dist += SQUARE(descriptor[i]-kp.descriptor[i]);

    return sqrt(dist);
}

void Keypoint::data_dump() {
    cout << "Descriptor:" << endl;
    if( descriptor == 0)
        cerr << "NO DATA!!!!" << endl;
    for(int i = 0; i < descriptor_size; i++)
        cout << descriptor[i] << " ";
    cout << endl;
}


float Keypoint::sum_desc() {
    float sum = 0.0;

    for( int i = 0; i < descriptor_size; i++ )
        sum += descriptor[i];

    return sum;
}


