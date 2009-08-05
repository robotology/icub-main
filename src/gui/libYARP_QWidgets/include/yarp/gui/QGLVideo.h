// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */


#ifndef __ICUB_QGLVIDEO__
#define __ICUB_QGLVIDEO__

// std
#include <iostream>
#include <string>

// yarp
#include <yarp/sig/Image.h>

// opengl
#ifdef WIN32
	#define _WINSOCKAPI_
	#include <windows.h>
#endif
#include <GL/gl.h>
#include <GL/glu.h>

// Qt
#include <qgl.h>
#include <qevent.h>

namespace yarp {
    namespace gui {
        class QGLVideo;
    }
}

class yarp::gui::QGLVideo : public QGLWidget {

	Q_OBJECT      

public:

    QGLVideo(QWidget *parent = NULL, const char *name = NULL );
	virtual ~QGLVideo();

	virtual void paintImage(yarp::sig::ImageOf<yarp::sig::PixelRgb> &img);
	virtual bool getMouseClick(int &x, int &y); // returns false if no click position available; sets internal x and y mouse positions to <0

protected:
	
	yarp::sig::ImageOf<yarp::sig::PixelRgb> _yrpImgCache;
	unsigned char *_pixData;
	int _xClick; // last click position 
	int _yClick; // last click position

    void initializeGL();
    void resizeGL( int w, int h );
    void paintGL();

	void mousePressEvent ( QMouseEvent * e );
};
#endif
