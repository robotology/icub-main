/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

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



