// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include<yarp/gui/QGLVideo.h>

using namespace yarp::gui;

QGLVideo::QGLVideo(QWidget *parent, const char *name ): QGLWidget(parent, name),
	_pixData(NULL),
	_xClick(-1),
	_yClick(-1){
}

QGLVideo::~QGLVideo(){
	_pixData = NULL;
}

void QGLVideo::initializeGL(){
    glClearColor( 0.0, 0.0, 0.0, 0.0 );
}

void QGLVideo::resizeGL( int w, int h ){
	glViewport(0, 0, (GLint) w, (GLint) h); /* Update the viewport size. */
	glMatrixMode(GL_PROJECTION); //glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluOrtho2D(0.0,(GLdouble) 1.0,0.0,(GLdouble) 1.0); // parallel projection
}

void QGLVideo::paintGL() {
    
	glClear(GL_COLOR_BUFFER_BIT); /* Clear the screen (colour) buffer. */

	if(_pixData != NULL){
		glRasterPos2i(0, 0 );
		//glBitmap (0, 0, 0, 0, xMove, yMove, NULL);
		//int cw = _yrpImgCache.width();
		//int ch = _yrpImgCache.height();
		//if(cw <= this->width() && ch <= this->height()){
		//std::cout << "cw: " << cw << " ch: " << ch << " thisw: " << this->width() << " thish: " << this->height() << std::endl;
		glDrawPixels((GLsizei) _yrpImgCache.width(),(GLsizei)  _yrpImgCache.height(), GL_RGB, GL_UNSIGNED_BYTE, _pixData);
		//}		
	}

	glFlush(); /* Flush the screen buffer to ensure it is displayed. */
}


void QGLVideo::paintImage(yarp::sig::ImageOf<yarp::sig::PixelRgb> &img){

	int wWidth = this->width();
	int wHeight = this->height();
	
	int width = img.width();
	int height = img.height();

	if(width == 0 || height == 0){
		return;
	}

	if (!(wWidth == _yrpImgCache.width() || wHeight == _yrpImgCache.height())) { 
		double ratioWindow = (double)wWidth/(double)wHeight;
		double ratioImage = (double)width/(double)height;
		_yrpImgCache.setQuantum(1);
		_yrpImgCache.setTopIsLowIndex(false);	
		if(ratioWindow > ratioImage){
			// need to stretch height
			_yrpImgCache.resize((int)(((double)width)*((double)wHeight/(double)height)), wHeight); 
		}
		else{
			// need to stretch width
			_yrpImgCache.resize(wWidth, (int)(((double)height)*((double)wWidth/(double)width)));
		}		
		_yrpImgCache.zero();
	}

    _yrpImgCache.copy(img, _yrpImgCache.width(), _yrpImgCache.height()); // scale input image (if required)

	_pixData = (unsigned char*)_yrpImgCache.getRawImage();

	this->updateGL();
}

void QGLVideo::mousePressEvent ( QMouseEvent * e ){
	_xClick = (e->pos()).x();
	_yClick = (e->pos()).y();
	//std::cout << "Mouse click at: " << _xClick << " " << _yClick << std::endl;
}

bool QGLVideo::getMouseClick(int &x, int &y){
	if(_xClick > -1 && _yClick > -1){
		x = _xClick;
		y = _yClick;
		_xClick = -1;
		_yClick = -1;
		return true;
	}
	return false;
}

