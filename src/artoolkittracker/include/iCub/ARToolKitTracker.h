// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 Alex Bernardino, Vislab, IST/ISR
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iostream>
#include <ostream>

using namespace std;

#ifndef __ARTOOLKITTRACKER__
#define __ARTOOLKITTRACKER__

#define	VISUALIZATIONWINDOWNAME	"ARToolKitTracker"

// ARToolkit
#include <AR/ar.h>

// OpenCV
#include  <cv.h>
#include <highgui.h>

//object database
//#include "object.h"


#define   OBJECT_MAX       8

typedef struct {
    char    name[256];
    int     id;
    bool    visible;
    double	marker_width;
    double	trans[3][4];
	int		markerindex;
} ObjectData_T;


/**
 *
 * ARToolkit Marker Detector 
 *
 *
 */

class ARToolKitTracker {

private:
	ObjectData_T *_object;
	ARParam _cparam; // parameters after correcting for actual image resolution
	ARParam _oparam; // original parameters
	int _object_num;


	ARMarkerInfo    *_marker_info;
	CvFont* _font;

	bool _visualize;

	IplImage *_frame;
	int _thresh;

	int _sizex;
	int _sizey;

    void copytrans(double src[3][4], double dst[3][4]);
    void calibtrans(double src[3][4], double dst[3][4]);

	int createAuxImage(void);
	
	ObjectData_T  *read_objectdata( const char *name, int *objectnum );
	char *get_buff( char *buf, int n, FILE *fp );



public:

    ARToolKitTracker();
    ~ARToolKitTracker();
    bool open();
    bool close();
	bool loadCameraParameters(const char* filename, int xsize, int ysize);
	bool loadObjectData(const char* filename);

	int initobjectdata( int numberofobjects );
	int addobjectdata( const int objnumber, const char *name, const char *filename, const double size );

	// if visualize is true it will open and show the tracking results
	int openVisualization();

	bool detectMarkers( IplImage *iplimg);

	bool isObjectDetected(int objnum);

	int getMatrix(int objnum, double T[3][4]);

	int getCenter(int objnum, int* x, int* y);

	int getSize(int objnum);

	int nobjects();

	int getObjectName(int objnum, char *name);
};





#endif

