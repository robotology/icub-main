// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Plinio Moreno
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#include <iCub/FaceEyeDetector.h>

FaceEyeDetector::FaceEyeDetector(){
	numberOfFacesFound = -1;
	detectorType = -1;
	_oldInSize.width = -1;
    _oldInSize.height = -1;
	currentColorImage = 0;
	currentGrayFloatImage = 0;
	themode = wt_avg;
	_needInit=true;
	eyeOption=false;
	openCvDetectObject = new CFaceDetect();
	//printf("Entered FaceEyeDetector constructor\n");
}

FaceEyeDetector::~FaceEyeDetector(){
	close();
}

bool FaceEyeDetector::open(Searchable &config){
	if (config.check("help","if present, display usage message")) {
        printf("Call with --name /module_prefix --file configFile.ini\n");
        return false;
    }
	eyeOption = (config.check("eyes",
                                    Value(0),
									"Flag to include eye detection").asInt()!=0);
	faceDetectorType = config.check("type",Value(""),"Face detection algorithm (string [mpt|opencv])").asString();
	if (faceDetectorType=="") {
        printf("*** Please specify a face detection algorithm, e.g. --type mpt\n");
        printf("*** Filters available:\n");
        printf("mpt opencv\n");
        return false;
    }
	else if ( faceDetectorType == "mpt"){
		learningAlgorithm = config.check("classificationAlgorithm",
                                    Value(0),
									"Set the learning algorithm: 0 gentleboost, 1 adaboost.").asInt();
		windowShifting = (float)config.check("windowShifting",Value(1.25),
		"Window shifiting to search for the best output in a local neighborhood, in image scale units, e.g. 1-no shift").asDouble();
		
		faceSimplification = (float)config.check("faceSimplification",
									Value(0.24),
		"intersection/union area ratio to reduce bounding boxes with some overlapping").asDouble();

		maxNumberFaces = config.check("maxNumberFaces",
									Value(5),
									"maximum number of faces found").asInt();
		if (eyeOption)
			mptFaceEyeDetectObject = new MPEyeFinderBinary(learningAlgorithm);
		else
			mptFaceDetectObject = new MPISearchBinary(learningAlgorithm);
		detectorType = MPTFACEDETECTORTYPE;

	}
	else if ( faceDetectorType == "opencv"){
		xmlFileLocation = config.check("cascade",Value(""),"XML file location (global path)").asString();
		minimumBoundingBoxSize = config.check("minBoundingBoxSize",
									Value(10),
									"minimum bounding box size").asInt();
		faceSimplification = (float)config.check("faceSimplification",
									Value(0.24),
		"intersection/union area ratio to reduce bounding boxes with some overlapping").asDouble();
		maxNumberFaces = config.check("maxNumberFaces",
									Value(5),
									"maximum number of faces found").asInt();
		detectorType = OPENCVFACEDETECTORTYPE;
		if (eyeOption){
			mptFaceEyeDetectObject = new MPEyeFinderBinary(learningAlgorithm);
			//facesEyesList = VisualObject();
		}
	}
	_needInit = true;
	return true;
}


bool FaceEyeDetector::detection(const Image &in){
	if ( in.width()  != _oldInSize.width || 
         in.height() != _oldInSize.height || 
        _needInit)
        init(cvSize(in.width(), in.height()));
	
	if ( detectorType == MPTFACEDETECTORTYPE ){
		//float *myFloatPointImage;
		cvCvtColor((IplImage*)in.getIplImage(),imgGray, CV_RGB2GRAY); //imgGray now contains gray scale image
		cvConvert( imgGray, currentGrayFloatImage );// Conversion from uchar to float
		// Conversion from opencv format to raw format
		grayImgStep=imgGray->width*sizeof(float);

        CvSize tmpSize=cvSize(in.width(), in.height());
		cvGetRawData(currentGrayFloatImage,(uchar**)&myFloatPointImage,&grayImgStep,&tmpSize);
		grayImgStep /= sizeof(myFloatPointImage[0]);
		// Image copy from raw format to array field in RImage object pixels
		mptImage.copyFromRawImage(myFloatPointImage, grayImgStep);
		if ( eyeOption ){
			//facesEyesList.clear();
			mptFaceEyeDetectObject->findEyes(mptImage, facesEyesList, windowShifting, faceSimplification, themode);
			//mptFaceEyeDetectObject->releaseStream();
			facesEyesList.sort();
			for ( int tempCont=0 ; tempCont < (int)facesEyesList.size()-maxNumberFaces; tempCont++ )
				facesEyesList.pop_front();
			numberOfFacesFound = facesEyesList.size();
		}
		else{
			facesList.clear();
			numberOfFacesFound = mptFaceDetectObject->search(mptImage,facesList);
			if ( faceSimplification != 0 )
				facesList.simplify(faceSimplification);
			facesList.sort();
			for ( int tempCont=0 ; tempCont < facesList.size()-maxNumberFaces; tempCont++ )
				facesList.pop_front();
			numberOfFacesFound = facesList.size();
		}
	}
	else if ( detectorType == OPENCVFACEDETECTORTYPE ){
		cvCvtColor( (IplImage*)in.getIplImage(), currentColorImage, CV_RGB2BGR);  //buffer now contains the original rgb image
		numberOfFacesFound = openCvDetectObject->detect( currentColorImage );
		if ( eyeOption && numberOfFacesFound > 0){
			cvCvtColor((IplImage*)in.getIplImage(),imgGray, CV_RGB2GRAY); //imgGray now contains gray scale image
			cvConvert( imgGray, currentGrayFloatImage );// Conversion from uchar to float
			// Conversion from opencv format to raw format
			grayImgStep=imgGray->width*sizeof(float);
			//myFloatPointImage=(float *)malloc(sizeof(float)*in.width()*in.height());
			
            CvSize tmpSize=cvSize(in.width(), in.height());
            cvGetRawData(currentGrayFloatImage,(uchar**)&myFloatPointImage,&grayImgStep,&tmpSize);
			grayImgStep /= sizeof(myFloatPointImage[0]);
			// Image copy from raw format to array field in RImage object pixels
			mptImage.copyFromRawImage(myFloatPointImage, grayImgStep);
			//cvReleaseImage(&currentGrayFloatImage);
			//free(myFloatPointImage);
			/*list<VisualObject *>::iterator currentTracker = facesEyesList.begin();
		    list<VisualObject *>::iterator lastTracker = facesEyesList.end();
            for(; currentTracker != lastTracker; ++currentTracker){
			    //printf("i: %d\n",i);
			    VisualObject *hTracker = *currentTracker;
                delete hTracker;
            }*/
			//printf("After cleaning\n");
			//facesEyesList.clear();
			//mptFaceEyeDetectObject->initStream(in.width(), in.height());
			mptFaceEyeDetectObject->findEyesUsingOCVFaceDetect(mptImage, facesEyesList, openCvDetectObject->faces, windowShifting, faceSimplification, themode);
			facesEyesList.sort();
			//mptFaceEyeDetectObject->releaseStream();
			/*int endListCrit;
			if (facesList.size()<=maxNumberFaces)
				endListCrit=facesList.size();
			else
				endListCrit=facesList.size()-maxNumberFaces;*/
			for ( int tempCont=0 ; tempCont < facesList.size()-maxNumberFaces; tempCont++ )
				facesEyesList.pop_front();
			numberOfFacesFound = facesEyesList.size();
		}
		else if ( eyeOption && numberOfFacesFound <= 0 ){
			list<FaceObject *>::iterator currentFace = facesEyesList.begin();
			list<FaceObject *>::iterator lastFace = facesEyesList.end();
			for(; currentFace != lastFace; ++currentFace){
				//printf("i: %d\n",i);
				FaceObject *hTracker = *currentFace;
				delete hTracker;
			}
			facesEyesList.clear();
		}
		else if ( !eyeOption && numberOfFacesFound > 0 ){
			facesList.clear();
			int faceSize=0;
			int i;
			//printf("number of faces: %d\n",openCvDetectObject->faces->total);
			for( i = 0; i < (openCvDetectObject->faces ? openCvDetectObject->faces->total : 0); i++ ){
					CvRect* r = (CvRect*)cvGetSeqElem( openCvDetectObject->faces, i );
					if (r->width > r->height)
						faceSize = r->width;
					else
						faceSize = r->height;
					facesList.push_front(Square (faceSize, r->x,r->y,0));
					delete r;
					//printf("After deleting r\n");
			}
			if ( faceSimplification != 0 )
				facesList.simplify(faceSimplification);
			facesList.sort();
			//printf("Face list size: %d\n",facesList.size());
			//printf ("Faces list size: %d\n",);
			/*int endListCrit;
			if (facesList.size()<=maxNumberFaces)
				endListCrit=facesList.size();
			else
				endListCrit=facesList.size()-maxNumberFaces;*/
			//printf ("end list criterion: %d\n",endListCrit);
			for ( int tempCont=0 ; tempCont < facesList.size()-maxNumberFaces; tempCont++ )
				facesList.pop_front();
			numberOfFacesFound = facesList.size();
			//printf("Face list size: %d\n",facesList.size());
		}
		else if ( !eyeOption && numberOfFacesFound <= 0 )
			facesList.clear();
	}
	_oldInSize.width  = in.width();
    _oldInSize.height = in.height();
	return true;
}

bool FaceEyeDetector::init(CvSize currImgSize){
	if ( detectorType == MPTFACEDETECTORTYPE ){
		if (eyeOption)
			mptFaceEyeDetectObject->initStream(currImgSize.width, currImgSize.height);
		mptImage.setSize(currImgSize.width, currImgSize.height);
		imgGray=cvCreateImage(currImgSize,  IPL_DEPTH_8U, 1);
		currentGrayFloatImage=cvCreateImage(currImgSize,  IPL_DEPTH_32F, 1);
	}
	else if ( detectorType == OPENCVFACEDETECTORTYPE ){
		if (eyeOption){
			mptFaceEyeDetectObject->initStream(currImgSize.width, currImgSize.height);
			mptImage.setSize(currImgSize.width, currImgSize.height);
			imgGray=cvCreateImage(currImgSize,  IPL_DEPTH_8U, 1);
			currentGrayFloatImage=cvCreateImage(currImgSize,  IPL_DEPTH_32F, 1);
			//myFloatPointImage=(float *)malloc(sizeof(float)*currImgSize.width*currImgSize.height);
		}
		openCvDetectObject->init(currImgSize.width, currImgSize.height, xmlFileLocation);
	}
	currentColorImage = cvCreateImage( currImgSize, 8, 3 );
	_needInit = false;
	return true;
}

bool FaceEyeDetector::close(){
	if ( faceDetectorType == "mpt"){
		if (eyeOption){
			delete mptFaceEyeDetectObject;
			if (!_needInit){
				cvReleaseImage(&imgGray);
				cvReleaseImage(&currentColorImage); 
				cvReleaseImage(&currentGrayFloatImage);
			}
		}
		else
			delete mptFaceDetectObject;
	}
	else if ( faceDetectorType == "opencv"){
		delete openCvDetectObject;
		if (eyeOption){
			delete mptFaceEyeDetectObject;
			//free(myFloatPointImage);
			if (!_needInit){
				cvReleaseImage(&imgGray);
				cvReleaseImage(&currentColorImage); 
				cvReleaseImage(&currentGrayFloatImage);
			}
			//facesEyesList.~VisualObject();
		}
	}
	return true;
}
//
