// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Authors: 2008 Dario Figueira
 * 
 *      Institute for Systems and Robotics (IST)
 *      IST, TULisbon, Portugal
 */


#ifndef SIFTOBJECTREPRESENTATION_H
#define SIFTOBJECTREPRESENTATION_H

#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <highgui.h>

#include <stdio.h>
#include <iostream>

#include "matching.h"
#include "SIFT_disparity.h"

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include "common_functions.h"
#include "ThreadSift.h"

using namespace std;

using namespace yarp;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;

class siftObjectRepresentationModule : public Module {
private:
// the yarp connection ports
    //In left image port and Out port for recognized objects
    BufferedPort<ImageOf<PixelBgr> > _prtImgLeft;
    //In right image port and Out port for saved objects
    BufferedPort<ImageOf<PixelBgr> > _prtImgRight;
    //Position of obect recognized in "left"
    yarp::os::BufferedPort<yarp::os::Bottle> _prtObjectPositions;
    // sending [azimuth, elevation] to gaze control
    BufferedPort<VectorOf<double> > _prtVctPosOut;  
    int _saccadeIndex;
    // reference to the Sift algorithm
    Matching _recognizer;
    SIFT_disparity _dispar;

    // controlboard
    PolyDriver  _dd;
    IEncoders   *_ienc;
    double      *_encoders;
    int         _numAxes;
    //focal lenghts, used in the calculations of disparity
    double      _left_fx;
    double      _left_fy;
    bool        _simulation; //if we should expect a real controlboard to be existant or not
    int         _storeNewObjects;  //if we should expect images from both eyes and look for new objects to store
    int         _orderSaccades; //if we can order the robot around, to look at the mean of the objects detected (or at a prefered object)
    int         _preferedObject; //number of a prefered object to look upon when detected
    int _saveOriginalLeftImages; //Whether to save or not the images seen by the left eye
    int _saveOriginalRightImages; //Whether to save or not the images seen by the right eye
    int _saveDrawnUponImages; //Whether to save or not the images  with the drawn detections on them
    int _writeDetections; //write the bottles to files

    //where to save the experiment images and bottles
    string _imagePath;
    string _filePath;    
    void SaveImage( IplImage* image, std::string filename );

public:
    // constructor & destructor
    siftObjectRepresentationModule();
    ~siftObjectRepresentationModule();

    // standard functions for YARP modules
    virtual bool open(Searchable &config);
    virtual bool close();
    virtual bool interruptModule();

    // the actual run function
    virtual bool updateModule();
    
    void orderSaccade(Bottle ObjectPositions);
};



#endif //SIFTOBJECTREPRESENTATION_H
