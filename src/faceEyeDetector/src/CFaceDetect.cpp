/*!
$Author: vislab $
Changed in 14/03/07 by Júlio - email: jgomes@isr.ist.utl.pt
Changes: nearest neighbourhood
*/

#include <iCub/CFaceDetect.h>
#include <stdio.h>
#include <math.h>

//#include "highgui.h"


// Classifier database
// Database must be in same directory as executable file
// Databases are usually provided by OpenCV demos in dir
// <OpenCV path>\data\haarcascades
//const char* CFaceDetect::cascade_name = "haarcascade_frontalface_alt.xml";
//const char* CFaceDetect::cascade_name = "G:/iCub/src/faceeyedetector/haarcascade_frontalface_alt.xml";
//"haarcascade_upperbody.xml";
//"haarcascade_profileface.xml";
//"haarcascade_frontalface_alt.xml";


/***************************************************************************
* Class Constructor:
*
* Creates an instance of class CFaceDetect.
* Don't forget to call init() to initialize all variables!
*/
CFaceDetect::CFaceDetect()
{
	storage=0;
	cascade=0;

	mask = 0;
	in_copy = 0;

	cascade = 0;
	storage = 0;

	first=true;
	lostLag=0;
	last_auxRect=cvRect(0,0,0,0);
	faces = NULL;
}


/***************************************************************************
* Instance initialization function:
*
* <cols> and <lines> are the dimensions of the largest images to be used
* by this instance.
*/
void CFaceDetect::init(int cols, int lines, const char *fileName)
{
	//printf("Init entered\n");
	//printf("%s\n",fileName);
	CvSize size = cvSize(cols,lines);
	//<<<<<<< .mine

	//	mask = cvCreateImage( size, 8, 1 );
	//=======
	//printf("facedetect c\n"); 
	mask = cvCreateImage( size, 8, 1 );
	  //printf("facedetect d\n"); 
	//>>>>>>> .r115
	in_copy = cvCreateImage( size, 8, 3 );
	//<<<<<<< .mine
	//cascade = (CvHaarClassifierCascade*)cvLoad( cascade_name, 0, 0, 0 );
	//=======
	//printf("facedetect e\n"); 
	//printf("Before loading cascade\n");
	//cascade = (CvHaarClassifierCascade*)cvLoad( cascade_name, 0, 0, 0 );
	cascade = (CvHaarClassifierCascade*)cvLoad( fileName, 0, 0, 0 );
	//>>>>>>> .r115
	//printf("After loading cascade\n");
	storage = cvCreateMemStorage(0);
}


/***************************************************************************
* Deallocation function:
*
* Deallocates images and storage spaces.
*/
void CFaceDetect::close()
{
	if (mask)
		cvReleaseImage(&mask);
	if (in_copy)
		cvReleaseImage(&in_copy);

	if (storage)
		cvReleaseMemStorage(&storage);
}


/***************************************************************************
* Face detection function:
*
* Detects all faces in input image <in>.
* If <in> ROI is set, a fake header is created and <in> is processed only
* within that ROI.
* Returns the number of detected faces.
* NOTE: Input image <in> can be RGB (tested) or Grayscale (not tested yet).
*/
int CFaceDetect::detect(IplImage *in)
{
	cvClearMemStorage( storage );

	if( cascade )
	{
		rect=cvGetImageROI(in);

		// CREATE A FAKE IMAGE HEADER:
		// DATA POINTS TO THE ORIGINAL DATA,
		// BUT CV "THINKS" fake_in HAS DIFFERENT SIZE.
		// __NEVER__ DEALLOCATE fake_in !!!
		fake_in=cvCreateImageHeader( cvSize(rect.width, rect.height),
			in->depth, in->nChannels);
		fake_in->widthStep=in->widthStep;
		fake_in->imageData=in->imageData+in->widthStep*rect.y+rect.x*in->nChannels;

		// DEBUG
		//cvNamedWindow( "Face Detect fake_in", 1 );
		//cvShowImage( "Face Detect fake_in", fake_in );

		// normal settings
		//faces = cvHaarDetectObjects( fake_in, cascade, storage,
		//                                1.1, 3, 0,
		//                                cvSize(40, 40) );

		// real-time settings
		//faces = cvHaarDetectObjects( fake_in, cascade, storage,
		//                                1.2, 2, CV_HAAR_DO_CANNY_PRUNING,
		//                                cvSize(40, 40) );

		// custom settings
		//faces = cvHaarDetectObjects( fake_in, cascade, storage,
		//                                1.2, 2, CV_HAAR_DO_CANNY_PRUNING,
		//                                cvSize(10, 10) );

		// custom settings 2
		if (faces != NULL)
			cvClearSeq( faces );
		faces = cvHaarDetectObjects( fake_in, cascade, storage,
			1.4, 3, CV_HAAR_DO_CANNY_PRUNING,
			cvSize(10, 10) );  //este ultimo parametro é o tamanho da BondingBox min 
		//para considerar a cara detectada
		cvReleaseImageHeader(&fake_in);
		return faces->total;
		//delete rect1;
	}
	else
	{
		printf("db: NOT CASCADE\n");
		return -1;
	}
}


/***************************************************************************
* Detected faces sorting function:
*
* Searches internal CvSeq <faces> for the face with greatest area.
* Stores bounding box in internal CvRect <auxRect> and returns it too.
* IMPORTANT: CALL AFTER detect(), NOT BEFORE!
*/
CvRect CFaceDetect::getBigFace()
{
	int scale = 1;
	CvPoint pt1, pt2;
	CvPoint pt1_aux, pt2_aux;
	int i;
	double dist,neigh,last_dist;
	dist=neigh=last_dist=0;

	last_dist=100;
	int centroid_nowX, centroid_nowY, centroid_beforeX, centroid_beforeY;
	lostLag++;
	AreaGreater=0;
	auxRect=cvRect(0,0,0,0);
	if (first) {
		//printf("\nta no first\n");

		for( i = 0; i < (faces ? faces->total : 0); i++ )
		{
			CvRect* r = (CvRect*)cvGetSeqElem( faces, i );
			pt1.x = r->x*scale+rect.x;
			pt2.x = (r->x+r->width)*scale+rect.x;
			pt1.y = r->y*scale+rect.y;
			pt2.y = (r->y+r->height)*scale+rect.y;

			area=(pt2.x-pt1.x)*(pt2.y-pt1.y);

			if (area>AreaGreater)
			{
				first=false;
				// CREATE RECT WITH GREATEST FACE BB			
				AreaGreater=area;
				auxRect=cvRect(pt1.x,pt1.y,(pt2.x-pt1.x),(pt2.y-pt1.y));		
				pt1_aux=pt1;
				pt2_aux=pt2;
			}
		}
		if ((faces ? faces->total : 0)==0)
			auxRect=cvRect(0,0,0,0);
		else
			last_auxRect=cvRect(pt1_aux.x,pt1_aux.y,(pt2_aux.x-pt1_aux.x),(pt2_aux.y-pt1_aux.y));

		/*
		if (i>0)
		last_auxRect=cvRect(pt1_aux.x,pt1_aux.y,(pt2_aux.x-pt1_aux.x),(pt2_aux.y-pt1_aux.y));
		else
		auxRect=cvRect(0,0,0,0);
		*/
	}
	else {

		//if (lostLag>5)
		//first=true;
		auxRect=cvRect(last_auxRect.x,last_auxRect.y,last_auxRect.width,last_auxRect.height);	
		//printf("\nestá aqui\n");
		centroid_beforeX=last_auxRect.x+last_auxRect.width/2;
		centroid_beforeY=last_auxRect.y+last_auxRect.height/2;

		for( i = 0; i < (faces ? faces->total : 0); i++ )
		{
			CvRect* r = (CvRect*)cvGetSeqElem( faces, i );
			pt1.x = r->x*scale+rect.x;
			pt2.x = (r->x+r->width)*scale+rect.x;
			pt1.y = r->y*scale+rect.y;
			pt2.y = (r->y+r->height)*scale+rect.y;
			centroid_nowX=pt1.x+abs((pt2.x-pt1.x))/2;
			centroid_nowY=pt1.y+abs((pt2.y-pt1.y))/2;		
			dist=sqrt((double)(centroid_beforeX-centroid_nowX)*(centroid_beforeX-centroid_nowX)+(double)(centroid_beforeY-centroid_nowY)*(centroid_beforeY-centroid_nowY));			
			neigh=(double)(last_auxRect.width/3);
			if (dist<=neigh && dist<last_dist)
				//if (dist<last_dist)
			{
				auxRect=cvRect(pt1.x,pt1.y,(pt2.x-pt1.x),(pt2.y-pt1.y));
				pt1_aux=pt1;
				pt2_aux=pt2;				
				lostLag=0;
				first=false;
				//printf("\nx-%d y-%d w-%d h-%d dist-%2.2f neigh-%2.2f last-%d\n", auxRect.x, auxRect.y, auxRect.width, auxRect.height,dist,neigh,last_auxRect.width);
				//printf("\ndist-%2.2f, centroidnowX-%d centroidnowY-%d\n", dist, centroid_beforeX, centroid_beforeY);
				last_dist=dist;
			}	
		}
		if ((faces ? faces->total : 0)==0) {
			first=true;
			auxRect=cvRect(0,0,0,0);
		}
		//else
		//last_auxRect=cvRect(pt1_aux.x,pt1_aux.y,(pt2_aux.x-pt1_aux.x),(pt2_aux.y-pt1_aux.y));
	}
	dist=sqrt((double)(auxRect.x-last_auxRect.x)*(auxRect.x-last_auxRect.x)+(double)(auxRect.y-last_auxRect.y)*(auxRect.y-last_auxRect.y));
	if (dist>60) {
		printf("\nvariacao %2.2f\n", dist);
		auxRect=cvRect(0,0,0,0);
	}
	//	if (auxRect.
	return auxRect;
}

/***************************************************************************
* Detected faces sorting function:
*
* Searches internal CvSeq <faces> for the face with greatest area.
* Stores bounding box in internal CvRect <auxRect> and returns it too.
* IMPORTANT: CALL AFTER detect(), NOT BEFORE!
*/
CvRect CFaceDetect::getLargestFace()
{
	int scale = 1;
	CvPoint pt1, pt2;
	CvPoint pt1_aux, pt2_aux;
	int i;
	double dist,neigh,last_dist;
	dist=neigh=last_dist=0;

	last_dist=100;
	//int centroid_nowX, centroid_nowY, centroid_beforeX, centroid_beforeY;
	lostLag++;
	AreaGreater=0;
	auxRect=cvRect(0,0,0,0);
	for( i = 0; i < (faces ? faces->total : 0); i++ )
	{
		CvRect* r = (CvRect*)cvGetSeqElem( faces, i );
		pt1.x = r->x*scale+rect.x;
		pt2.x = (r->x+r->width)*scale+rect.x;
		pt1.y = r->y*scale+rect.y;
		pt2.y = (r->y+r->height)*scale+rect.y;

		area=(pt2.x-pt1.x)*(pt2.y-pt1.y);

		if (area>AreaGreater)
		{
			// CREATE RECT WITH LARGEST FACE BB			
			AreaGreater=area;
			auxRect=cvRect(pt1.x,pt1.y,(pt2.x-pt1.x),(pt2.y-pt1.y));		
			pt1_aux=pt1;
			pt2_aux=pt2;
		}
	}
	return auxRect;
}

/***************************************************************************
* Detected faces bounding boxes drawing function:
*
* Draws bounding boxes of detected faces on image <in_copy> (an internal
* copy of <in> and marks those regions on binary image <mask>.
* If <in> ROI is set, a fake header is created and <in_copy> and <mask> are
* only processed within that ROI.
* IMPORTANT: CALL AFTER detect(), NOT BEFORE!
* IMPORTANT: input image <in> _MUST_ be the same as in detect()!
* NOTE: may be called before or after getBigFace(). 
* NOTE: input image can be RGB (tested) or Grayscale (not tested yet).
*/
void CFaceDetect::draw(IplImage *in)
{
	int scale = 1;
	CvPoint pt1, pt2;
	int i;

	for( i = 0; i < (faces ? faces->total : 0); i++ )
	{
		CvRect* r = (CvRect*)cvGetSeqElem( faces, i );
		pt1.x = r->x*scale+rect.x;
		pt2.x = (r->x+r->width)*scale+rect.x;
		pt1.y = r->y*scale+rect.y;
		pt2.y = (r->y+r->height)*scale+rect.y;

		// draws red boxes on respective image
		CvRect in_ROI=cvGetImageROI(in);
		cvResetImageROI(in);
		cvCopy(in, in_copy);
		//cvCopy(in, out);
		cvSetImageROI(in, in_ROI);
		if (in_copy->nChannels==1)
			cvRectangle( in_copy, pt1, pt2, cvScalar(255), 3, 8, 0 );
		else if (in_copy->nChannels==3)
			cvRectangle( in_copy, pt1, pt2, CV_RGB(255,0,0), 1, 8, 0 );
		cvCopy(in_copy, in);
		// draws filled white rectangles on respective mask
		//cvRectangle( mask, pt1, pt2, cvScalar(255), CV_FILLED, 8, 0 );
	}
}
