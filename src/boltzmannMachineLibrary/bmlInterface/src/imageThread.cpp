// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iCub/imageThread.h>


/**
* initialise the thread
*/
bool imageThread::threadInit(){
	cvImage= cvCreateImage(cvSize(320,240), IPL_DEPTH_8U, 3 );
	image2=new ImageOf<PixelRgb>;
	image2->resize(320,240);
	printf("Image Thread initialising.....");
	return true;
}

/**
* code that is executed after the thread starts
* @param s is true if the thread started
*/
void imageThread::afterStart(bool s){
	if(s){
		printf("Image Thread after start.....\n");		
	}

}

/**
* running code of the thread
*/
void imageThread::run(){
	printf("Image Thread running..... \n");
	
}

/**
* code executed when the thread is released
*/
void imageThread::threadRelease(){
	printf("Image Thread releasing..... \n");	
}

ImageOf<PixelRgb>* imageThread::getYarpImage(){
	return image2;
}