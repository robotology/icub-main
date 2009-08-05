// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Plinio Moreno
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUB_FACEEYEDETECTOR_INC
#define ICUB_FACEEYEDETECTOR_INC

//yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iCub/eyefinderBinary.h>
#include <iCub/mpisearchBinary.h>
#include <iCub/CFaceDetect.h>

namespace iCub {
    namespace contrib {
        class FaceEyeDetector;
    }
}

using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::contrib;

#define MPTFACEDETECTORTYPE 0
#define OPENCVFACEDETECTORTYPE 1

/**
 * Face detection module.
 */
class iCub::contrib::FaceEyeDetector : public IConfig {
private:
	IplImage *currentColorImage;
	IplImage *currentGrayFloatImage;
	IplImage *imgGray;
	float *myFloatPointImage;
	int grayImgStep;
	MPISearchBinary *mptFaceDetectObject;
	MPEyeFinderBinary *mptFaceEyeDetectObject;
	RImage<float> mptImage;
	CFaceDetect *openCvDetectObject;
	float windowShifting;
	float faceSimplification;
	int maxNumberFaces;
	int minimumBoundingBoxSize;
	int learningAlgorithm;
	bool _needInit;
	int detectorType;
	combine_mode themode;
	CvSize          _oldInSize;
	ConstString faceDetectorType;
	ConstString xmlFileLocation;
public:
	//////////////////// These variables must be private!!!
	list<FaceObject *> facesEyesList;
	FaceBoxList facesList;
	bool eyeOption;
	int numberOfFacesFound;
	////////////////////
	FaceEyeDetector();
	virtual ~FaceEyeDetector();
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();
	bool detection(const Image &in);
	bool init(CvSize currImgSize);

};

#endif
