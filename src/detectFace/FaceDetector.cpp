#include "FaceDetector.h"

//Class constructor
FaceDetector::FaceDetector(int width, int height)
{

	bool ret = false;

	facedetector = new CFaceDetect();
	facedetector->init(width, height);      


	// A CvSize structure to hold the dimensions
	imgSize.width = width; 
	imgSize.height = height;

	img2=cvCreateImage(imgSize,  IPL_DEPTH_8U, 3);
	
}


FaceDetector::~FaceDetector()//Class Destructor
{

	bool ret = false;
	cvReleaseImage( &img2 );

}



int FaceDetector::try_detect_features( IplImage *img )
{
	int facedetected=0;
	bool ret = false;


	cvNamedWindow("eye",0);

	cvCopy(img, img2,NULL);


	facedetected=facedetector->detect( img2 );
	facedetector->getBigFace();
	facedetector->draw( img2);


	if(facedetected)
	{
		cvShowImage( "eye", facedetector->in_copy );
		cvWaitKey( 10 );
		feature[0] = facedetector->auxRect.x + facedetector->auxRect.width/2;  //correcçao pelo facto de ele estar a trackar só com um olho
		feature[1] = facedetector->auxRect.y + facedetector->auxRect.height/2;
	}
	else
	{
		cvCopy(img, img2, NULL);
		cvShowImage("eye", img2);
		cvWaitKey( 10 );

		feature[0] = 0.0;
		feature[1] = 0.0;
	}

	return 0;
}