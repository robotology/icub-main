// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Author: 2008 Dario Figueira
 *      Institute for Systems and Robotics (IST)
 *      IST, TULisbon, Portugal
 */

#include <highgui.h>
#include <iostream>
#include "keypoint.hpp"
#include "siftRH.h"
#include "kdtree.h"
#include "imgfeatures.h"

#include <cmath> //M_PI

#include <string>
#include <sstream>
using namespace std;

string itos(int i){      // convert int to string
    stringstream s;
    s << i;
    return s.str();
}

void SaveImage( IplImage* image, const char* rootname ) {
    int i = 0;
    char name[1000];

    for( i = 0; i < 1000; i++ ) {
        sprintf( name, "%s%03d.bmp", rootname, i );
        FILE* f = fopen( name, "rb" );

        if( f ) {
            fclose(f);
            continue;
        }

        cvSaveImage( name, image );

        break;
    }
}

IplImage* crop(IplImage* img, int MaxWidth, int MinWidth, int MaxHeight, int MinHeight) {

    CvRect rect = cvRect( MinWidth, MinHeight, MaxWidth - MinWidth, MaxHeight - MinHeight);

//     cout <<"rect="<<MinWidth <<" "<< MinHeight <<" "<<  MaxWidth - MinWidth <<" "<<  MaxHeight - MinHeight<<endl;
    CvRect rect_orig = cvGetImageROI(img);
//     cout <<"rect_orig="<<rect_orig.x <<" "<< rect_orig.y <<" "<<  rect_orig.width <<" "<<  rect_orig.height <<endl;

    cvSetImageROI(img, rect);
    CvSize size = cvSize( MaxWidth - MinWidth, MaxHeight - MinHeight);
    IplImage *cropped = cvCreateImage(size, img->depth, img->nChannels);
    cvCopy(img, cropped);

    cvResetImageROI(img);
    return cropped;
}

Keypoint feature2keypoint(struct feature* feat) {
    Keypoint kp;
    int octave, scale, row, col;
    struct detection_data* ddata = NULL;
    ddata = feat_detection_data( feat );

    if(ddata == NULL) {
        cout <<"ddata == NULL"<<endl; //DEBUG
        exit(0);
    }

    octave = ddata->octv;
    scale = ddata->intvl; ///IS IT?????
    row = ddata->r;
    col = ddata->c;

    kp.set_pos(octave, scale, row, col);
    kp.s = (float) feat->scl;
    kp.x = (float) feat->x;
    kp.y = (float) feat->y;
    kp.th = (float) (-feat->ori+M_PI) *360./(2.*M_PI);
    kp.set_descriptor((float *) feat->descr);

    // 	MESSAGE("In feature2keypoint: kp.s = " << kp.s); //DEBUG

    return kp;
}
