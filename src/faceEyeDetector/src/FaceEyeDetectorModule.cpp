// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Plinio Moreno
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#include <iCub/FaceEyeDetectorModule.h>

FaceEyeDetectorModule::FaceEyeDetectorModule(){

}

FaceEyeDetectorModule::~FaceEyeDetectorModule(){
	// data is released trough close/interrupt methods
}

bool FaceEyeDetectorModule::open(Searchable& config){

	openResult = fEDetector.open(config);
	// open image ports
	_prtImgRgb.open(getName("rgb"));
	_prtImgDetectedFacesRgb.open(getName("out"));
	_detectorOutput.open(getName("detectOut"));
	return true;
}

bool FaceEyeDetectorModule::close(){
	_prtImgRgb.close();
	_prtImgDetectedFacesRgb.close();
	_detectorOutput.close();
	//_prtImg.close();
	if (openResult)
		fEDetector.close();
		//fEDetector.~FaceEyeDetector();
	//printf("after destructor called\n");
	/*if (_ocvImgIn != NULL)
	cvReleaseImage(&_ocvImgIn);
	_ocvImgIn = NULL;
	if (_ocvImgTmp1 != NULL)
	cvReleaseImage(&_ocvImgTmp1);
	_ocvImgTmp1 = NULL;
	if (_ocvImgTmp2 != NULL)
	cvReleaseImage(&_ocvImgTmp2);
	_ocvImgTmp2 = NULL;
	if (_ocvImgOut != NULL)
	cvReleaseImage(&_ocvImgOut);
	_ocvImgOut = NULL;*/


	return true;
}

bool FaceEyeDetectorModule::interruptModule(){

	//_prtImg.interrupt();
	_prtImgRgb.interrupt();
	_prtImgDetectedFacesRgb.interrupt();
	_detectorOutput.interrupt();
	return true;
}

bool FaceEyeDetectorModule::updateModule(){

	//yarp::sig::ImageOf<PixelRgb> *yrpImgIn;
	//printf("Entered update module\n");
	yarp::sig::ImageOf<PixelRgb> *yrpImgRgbIn = _prtImgRgb.read(false);
	//printf("update 1\n");
	//yrpImgIn->getIplImage)=
	//yrpImgIn = _prtImg.read();
	//if (yrpImgIn == NULL)   // this is the case if module is requested to quit while waiting for image
	//    return true;
	if (yrpImgRgbIn != NULL){
		fEDetector.detection(*yrpImgRgbIn);
	//}
	//else return true;
	//printf("update 2\n");
	yarp::sig::ImageOf<PixelRgb> yrpImgOut(*yrpImgRgbIn);
	// prepare output images
	//printf("update 3\n");
	_currentImage = (IplImage*)yrpImgOut.getIplImage();
	//printf("update 4\n");
	ImageOf<PixelRgb>& yrpImgRgbOut = _prtImgDetectedFacesRgb.prepare();
	drawDetectedFaces(_currentImage,1,CV_RGB(255,0,255));
	yrpImgRgbOut = yrpImgOut;
	_prtImgDetectedFacesRgb.write();
	DetectorOutput& outputPrepare = _detectorOutput.prepare();
	//outputPrepare.detectImage->copy(*yrpImgRgbIn);
	outputPrepare.detectImage = yrpImgRgbIn;//new ImageOf<PixelRgb>(*yrpImgRgbIn);//yrpImgRgbIn;//
	if (fEDetector.eyeOption){
		outputPrepare.nFaces = fEDetector.numberOfFacesFound;
		outputPrepare.eyeFlag = true;
		outputPrepare.facesEyes = list<FaceObject *>(fEDetector.facesEyesList);
	}
	else{
		//printf ("list size: %d\n",fEDetector.facesList.size());
		outputPrepare.nFaces = fEDetector.numberOfFacesFound;//fEDetector.facesList.size();
		outputPrepare.eyeFlag = false;
		outputPrepare.facesOnly = FaceBoxList(fEDetector.facesList);
	}
	_detectorOutput.write();
	//printf("writing\n");
	}
	else return true;
	//outputPrepare.~DetectorOutput();
	/*if (fEDetector.eyeOption)
		outputPrepare.facesEyes.~VisualObject();
	else
		outputPrepare.facesOnly.~ObjectList();*/
	//outputPrepare.~DetectorOutput();
	//	track
	//cout << "module running " << endl;

	return true;
	//return false;   // closing module
}

void FaceEyeDetectorModule::drawDetectedFaces(IplImage *currentImage, int lineThickness, CvScalar color){
	/*void drawBoundingBox(IplImage *currentImage, IplImage *imageCopy, 
	CvRect boundingBox, int lineThickness, CvScalar color)
	{*/
	CvPoint pt1, pt2;
	if ( fEDetector.eyeOption ){
		float eyewidth, shift;
		list<FaceObject *>::iterator face = fEDetector.facesEyesList.begin();
		list<FaceObject *>::iterator last_face = fEDetector.facesEyesList.end();
		for(int i = 0; face != last_face; ++face, ++i){
			// Current face plot
			FaceObject *f = static_cast<FaceObject*>(*face);
			pt1.x = (int)f->x;
			pt2.x = (int)(f->x+f->xSize);
			pt1.y = (int)f->y;
			pt2.y = (int)(f->y+f->ySize);
			//CvRect in_ROI=cvGetImageROI(buffer);
			//cvResetImageROI(buffer);
			//cvCopy(buffer, in_copy);
			//cvSetImageROI(buffer, in_ROI);
			//f->eyes.xLeft;
			if (currentImage->nChannels==1)
				cvRectangle( currentImage, pt1, pt2, cvScalar(255), lineThickness, 8, 0 );
			else if (currentImage->nChannels==3)
				cvRectangle( currentImage, pt1, pt2, color, lineThickness, 8, 0 );

			eyewidth = (float)((f->xSize)*0.04);
			shift = eyewidth/2;
			//printf("shift: %f\n",shift);
			pt2.x = (int)(f->eyes.xLeft+eyewidth);
			pt1.x = (int)(f->eyes.xLeft-shift);
			pt2.y = (int)(f->eyes.yLeft+eyewidth);
			pt1.y = (int)(f->eyes.yLeft-shift);
			if (currentImage->nChannels==1)
				cvRectangle( currentImage, pt1, pt2, cvScalar(255), lineThickness, 8, 0 );
			else if (currentImage->nChannels==3)
				cvRectangle( currentImage, pt1, pt2, color, lineThickness, 8, 0 );
			pt2.x = (int)(f->eyes.xRight+eyewidth);
			pt1.x = (int)(f->eyes.xRight-shift);
			pt2.y = (int)(f->eyes.yRight+eyewidth);
			pt1.y = (int)(f->eyes.yRight-shift);
			if (currentImage->nChannels==1)
				cvRectangle( currentImage, pt1, pt2, cvScalar(255), lineThickness, 8, 0 );
			else if (currentImage->nChannels==3)
				cvRectangle( currentImage, pt1, pt2, color, lineThickness, 8, 0 );
		}
	}
	else{
		int nBoxes = fEDetector.facesList.size();
		FaceBoxList tempFacesList = FaceBoxList(fEDetector.facesList);
		if(nBoxes != 0) {
			while(!tempFacesList.empty( ))
			{
				Square face = tempFacesList.front();  
				tempFacesList.pop_front();
				pt1.x = face.x;
				pt2.x = face.x+face.size;
				pt1.y = face.y;
				pt2.y = face.y+face.size;
				//CvRect in_ROI=cvGetImageROI(buffer);
				//cvResetImageROI(buffer);
				//cvCopy(buffer, in_copy);
				//cvSetImageROI(buffer, in_ROI);
				if (currentImage->nChannels==1)
					cvRectangle( currentImage, pt1, pt2, cvScalar(255), lineThickness, 8, 0 );
				else if (currentImage->nChannels==3)
					cvRectangle( currentImage, pt1, pt2, color, lineThickness, 8, 0 );
				//cvCopy(in_copy, buffer);

			}
			tempFacesList.clear();
		}
	}
	/*CvPoint pt1, pt2;
	CvRect in_ROI=cvGetImageROI(currentImage);
	cvResetImageROI(currentImage);
	cvCopy(currentImage, imageCopy);
	cvSetImageROI(currentImage, in_ROI);
	pt1.x = boundingBox.x;
	pt2.x = boundingBox.x+boundingBox.width;//  (r->x+r->width)*scale+rect.x;
	pt1.y = boundingBox.y;
	pt2.y = boundingBox.y+boundingBox.height;
	if (imageCopy->nChannels==1)
	cvRectangle( imageCopy, pt1, pt2, cvScalar(255), 1, 8, 0 );
	else if (imageCopy->nChannels==3)
	cvRectangle( imageCopy, pt1, pt2, color, lineThickness, 8, 0 );
	cvCopy(imageCopy, currentImage);
	}*/
}
//
