/*	----- DFTPlots (2008 / 08 / 22) -------
	I decided to create the A and B panels on the screen
	This code generates a black screen with two colored panels
	One of the panels changes its size in sinusoidal way.
	The color of the panels in RGB is specified by the user:

	dftPlots.exe [R:0-255] [G:0-255] [B:0-255]

	Boris A. Duran
*/

#include <stdio.h>
#include <iostream>

#include <math.h>

#include "cv.h"
#include "highgui.h"

using namespace std;

#define PI 3.1415926535897932

const int _w = 1400;	// Size of field
const int _h = 900;		// Width of plot
const int T = 200;	// Speed for stimulus

int main(int argc, char **argv){
	int rr, gg, bb;
	int ab=0, ts = 0, abTop = 500, delay = 1;
	IplImage *plotImg = NULL;
	CvSize	wSize;
	CvPoint v1o, v1f, v2o, v2f;
	const char *wName = "Panel";

	rr = atoi(argv[1]); gg = atoi(argv[2]); bb = atoi(argv[3]);
	wSize.width = _w;
	wSize.height = _h;
	plotImg = cvCreateImage( wSize, 8, 3 );
	cvNamedWindow( wName, CV_WINDOW_AUTOSIZE ); // create a window
	cvMoveWindow( wName, 100, 1);

	v1o.x = 100;		v1o.y = _h/2 + 100;	
	v1f.x = _w/2 - 400;	v1f.y = _h - 100;
	v2o.x = _w/2 + 400;	v2o.y = _h/2 + 100;
	v2f.x = _w - 100;	v2f.y = _h - 100;
	for(;;){
		if( ts == T/4 ){
			if( ab == 'a' )
				v1o.y = 100 + _h/2 - abTop;
			else if( ab == 'd' )
				v2o.y = 100 + _h/2 - abTop;
			delay = 5000;
			cout << "Cue delay: " << delay/1000 << " secs." << endl;
		}else if( ts == T/2 ){
			delay = 7000;
			cout << "Rest delay: " << delay/1000 << " secs." << endl;
		}else if( ts == 3*T/4 ){
			v1o.y = 100 + _h/2 - abTop;
			v2o.y = 100 + _h/2 - abTop;
			delay = 5000;
			cout << "Task delay: " << delay/1000 << " secs." << endl;
		}else{
			if( ab == 'a' )
				v1o.y = 100 + _h/2 - (int)floor(abTop*fabs( sin(2*PI*ts/T) ));
			else if( ab == 'd' )
				v2o.y = 100 + _h/2 - (int)floor(abTop*fabs( sin(2*PI*ts/T) ));
			if( ts > T/2 ){
				v1o.y = 100 + _h/2 - (int)floor(abTop*fabs( sin(2*PI*ts/T) ));
				v2o.y = 100 + _h/2 - (int)floor(abTop*fabs( sin(2*PI*ts/T) ));
			}
			if( ts == T ) ts = 0;
			delay = 1;
		}
		cvSetZero( plotImg );
		cvRectangle( plotImg, v1o, v1f, CV_RGB(rr,gg,bb), CV_FILLED, 8, 0 );
		cvRectangle( plotImg, v2o, v2f, CV_RGB(rr,gg,bb), CV_FILLED, 8, 0 );
		cvShowImage( wName, plotImg );
		cvWaitKey(delay);

		if(ts == 0 )
			ab = cvWaitKey(0);
		ts = ts + 1;

		int key = cvWaitKey(2);
		if( key == 27 ) // 'ESC'
			break;
	}

	cout << "Done... byebye!!!" << endl ;
	cvReleaseImage(&plotImg);

	return 0; 
}
