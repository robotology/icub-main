
#include <iCub/vis/facedet.h>


/***************************************************/
faceDetector::faceDetector()
{
	pCascade=NULL;
	pStorage=NULL;
}


/***************************************************/
bool faceDetector::init(const char *haarCascadePath)
{
	pStorage=cvCreateMemStorage(0);

	if (pCascade=(CvHaarClassifierCascade*)cvLoad(haarCascadePath,0,0,0))
		return true;
	else
		return false;
}


/***************************************************/
faceDetector::~faceDetector()
{
	if (pCascade)
		cvReleaseHaarClassifierCascade(&pCascade);

	if (pStorage)
		cvReleaseMemStorage(&pStorage);
}


/***************************************************/
CvSeq *faceDetector::detect(IplImage *pImg)
{
    int minFaceSize=pImg->width/12;
    cvClearMemStorage(pStorage);

    return cvHaarDetectObjects(pImg,pCascade,pStorage,
                               1.1,                       // increase search scale by 10% each pass
                               3,                         // require three neighbors
                               CV_HAAR_DO_CANNY_PRUNING,  // skip regions unlikely to contain a face
                               cvSize(minFaceSize,minFaceSize));
}



