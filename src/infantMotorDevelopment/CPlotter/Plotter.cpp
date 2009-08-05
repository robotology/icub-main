#include "Plotter.h"

CPlotter::CPlotter(void){
}

CPlotter::~CPlotter(void){
	plotImg->imageData = NULL;		// memory release with OpenCV
	cvReleaseImage( &plotImg );
}
void CPlotter::init( int i, int n, int _w, int _h, double yUpperLimit, double yLowerLimit ){
	wSize.width = _w;
	wSize.height = _h;
	yMax = yUpperLimit;
	yMin = yLowerLimit;
	yFactor = wSize.height/(yMax - yMin);
	plotImg = cvCreateImage( wSize, 8, 3 );
	cvNamedWindow( wName, CV_WINDOW_AUTOSIZE ); // create a window
	cvMoveWindow( wName, 1, 1+(i-1)*(_h+40));

	yAxis_ini.x = wSize.width/2;	yAxis_ini.y = 1;
	yAxis_fin.x = wSize.width/2;	yAxis_fin.y = wSize.height;
	xAxis_ini.x = 1;				xAxis_ini.y = yMax*yFactor;
	xAxis_fin.x = wSize.width;		xAxis_fin.y = yMax*yFactor;

	return;
}
void CPlotter::show( double *field ){
	CvPoint pIni, pFin;
	cvSetZero( plotImg );
	int yZero = yMax*wSize.height/(yMax - yMin);

	cvLine( plotImg, xAxis_ini, xAxis_fin, CV_RGB(250,250,250), 1, CV_AA, 0 );
	cvLine( plotImg, yAxis_ini, yAxis_fin, CV_RGB(250,250,250), 1, CV_AA, 0 );
	for(int i = 0; i<wSize.width-1; i++){
		pIni.x = i;
		pIni.y = floor((yMax - field[i])*yFactor);
		pFin.x = i+1;
		pFin.y = floor((yMax - field[i+1])*yFactor);
		cvLine( plotImg, pIni, pFin, CV_RGB(255,0,0), 1, CV_AA, 0 );
	}
	cvShowImage( wName, plotImg );
	cvWaitKey(2);

	return;
}

void CPlotter::show( double *field1, double *field2 ){
	CvPoint pIni1, pFin1, pIni2, pFin2;
	cvSetZero( plotImg );

	cvLine( plotImg, xAxis_ini, xAxis_fin, CV_RGB(250,250,250), 1, CV_AA, 0 );
	cvLine( plotImg, yAxis_ini, yAxis_fin, CV_RGB(250,250,250), 1, CV_AA, 0 );
	for(int i = 0; i<wSize.width-1; i++){
		pIni1.x = i;
		pIni1.y = (int)floor((yMax - field1[i])*yFactor);
		pFin1.x = i+1;
		pFin1.y = (int)floor((yMax - field1[i+1])*yFactor);
		cvLine( plotImg, pIni1, pFin1, CV_RGB(255,0,0), 1, CV_AA, 0 );
		pIni2.x = i;
		pIni2.y = (int)floor((yMax - field2[i])*yFactor);
		pFin2.x = i+1;
		pFin2.y = (int)floor((yMax - field2[i+1])*yFactor);
		cvLine( plotImg, pIni2, pFin2, CV_RGB(0,255,0), 1, CV_AA, 0 );
	}
	cvShowImage( wName, plotImg );
	cvWaitKey(2);

	return;
}
