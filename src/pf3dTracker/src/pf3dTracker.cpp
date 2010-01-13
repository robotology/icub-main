
/**
* \defgroup icub_pf3dtracker_lib pf3dTracker Library
* @ingroup icub_libraries
* @ingroup icub_pf3dtracker
*
* Library of the 3d position tracker implementing the particle filter.
* See \ref icub_pf3dtracker \endref
*
* Copyright (C) 2009 RobotCub Consortium
*
* Author: Matteo Taiana
*
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

//
//
//
// NOTES:
// distances are trated as in millimeters inside the program, but they are converted to meters when communicating to the outside.
// likelihood is normalized only when communicated to the outside.

// TODO
//
//  0. make things compile and build.
//
//  1. test that the function that computes the histogram from the RGB image works.
//
//  1. check that things behave properly
//
//  2. make sure randomization is really random.
//
//  3. remove stuff that is not used
//
//  4. make the code check the return values (ROBUSTNESS/DEBUGGABILITY).
//
//  5. try to optimize the code, make it faster.
//

//  X. fix the memory leak. DONE

//          il RESAMPLING HA DEI PROBLEMI???: tipicamente la matrice delle particelle e' piena di n-a-n dopo il resampling, se uso poche particelle (non ho provato con molte a vedere come e' l'output).
//          con 5000 particelle sembra che funzioni tutto...

//things to be done:

//1. when the ball is moving towards the camera, the tracker lags behind it... is this because particles that are nearer than the true value to the camera have zero likelihood while the ones that are farther have higher likelihood values? how could i test this? plotting the positions of the particles? how could i solve this?

//2. check the resampling function: sometimes when the image goes blank the tracker programs quits suddenly. it's probably that function trying to write a particle value in an area where it should not (array overflow).

// measure which parts of the tracking loop are using up processing time.

//WHISHLIST:
//turn all static arrays into pointers to pointers, so that their size can be chosen at run time (through the configuration file)
//remove unnecessary included headers and namespaces.
//comment the whole project in doxygen style.
//create a pair of support files (.hpp and .cpp) specifically for this project.
//create a variable to contain the parameters for the motion model
//the size of the image is not a parameter.
//there are some functions which have among their parameters "inside_outside_...". this should be fixed, as this value is now defined at compile time.

//DONE:
//fixed: memory leak. when is the memory allocated by OpenCV freed? never? take a look at the small functions I call repeatedly: they might be the cause for the memory leak. use: cvReleaseImage.
//the images are visualized through opencv: change this to the output video port.
//the output data port is not used so far: it should stream the numeric values of the estimates.
//fixed the seg-fault problem. s-f happen when stdDev is high (or at the beginning of a tracking)... seems like a problem in the resampling algorithm.
//fixed. it was a prolem in the randomization that was not really random. 1. check the resampling algorithm: when the acceleration error is low, the tracker starts with particles in 0,0,0 and stays there.
//maybe this is due to the fact that the first images the tracker receives are meaningless, the likelihood is quite low, so the particles are spread a lot and one happens to be near the origin. all the particles are concentrated there and if you don't have a high acceleration noise you get stuck there.
//done. write particles status/likelihood on the particleOutputPort, for visualization purposes.
//   write a client that reads this data and plots particles.

#include <iCub/pf3dTracker.hpp>

#include <highgui.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <time.h>

#include <gsl/gsl_math.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/os/ResourceFinder.h>



void printMat(CvMat* A);



//constructor
PF3DTracker::PF3DTracker()
{
    ;
}



//destructor
PF3DTracker::~PF3DTracker()
{
    cout<<"oh my god! they killed kenny!    you bastards!\n";
}



//member function that set the object up.
bool PF3DTracker::open(Searchable& config)
{
    _doneInitializing=false;

    //cout<<"\033[35;1m***** CHECKPOINT 00 ***** beginning\033[0m\n";
    //fflush(stdout);
    
    bool failure;
    bool quit;
    string trackedObjectColorTemplate;
    string dataFileName;
    string trackedObjectShapeTemplate;
    string motionModelMatrix;
    string temp;
    int row, column;
    double widthRatio, heightRatio;

    quit=false;
    _saveImagesWithOpencv=false;

    _lut = new Lut[256*256*256];
    //create the look up table lut
    //this shouldn't be done every time
    //TOBEDONE: write the lut on a file and read it back.
    fillLut(_lut);

    srand(time(0)); //make sure random numbers are really random.
    rngState = cvRNG(rand());

    //allocate some memory and initialize some data structures for colour histograms.
    int dimensions;
    dimensions=3;
    int sizes[3]={YBins,UBins,VBins};
    _modelHistogramMat=cvCreateMatND(dimensions, sizes, CV_32FC1);
    if(_modelHistogramMat==0)
    {
        cout<<"PF3DTracker::open - I wasn\'t able to allocate memory for _modelHistogramMat.\n";
        fflush(stdout);
        quit =true;
    }
    _innerHistogramMat=cvCreateMatND(dimensions, sizes, CV_32FC1);
    if(_innerHistogramMat==0)
    {
        cout<<"PF3DTracker::open - I wasn\'t able to allocate memory for _innerHistogramMat.\n";
        fflush(stdout);
        quit =true;
    }
    _outerHistogramMat=cvCreateMatND(dimensions, sizes, CV_32FC1);
    if(_outerHistogramMat==0)
    {
        cout<<"PF3DTracker::open - I wasn\'t able to allocate memory for _outerHistogramMat.\n";
        fflush(stdout);
        quit =true;
    }

    _model3dPointsMat=cvCreateMat(3, 2*nPixels, CV_32FC1);
    if(_model3dPointsMat==0)
    {
        cout<<"PF3DTracker::open - I wasn\'t able to allocate memory for _model3dPointsMat.\n";
        fflush(stdout);
        quit =true;
    }
    _points2Mat=cvCreateMat(3, 2*nPixels, CV_32FC1);
    if(_points2Mat==0)
    {
        cout<<"PF3DTracker::open - I wasn\'t able to allocate memory for _points2Mat.\n";
        fflush(stdout);
        quit =true;
    }
    _tempMat=cvCreateMat(3, 2*nPixels, CV_32FC1);
    if(_tempMat==0)
    {
        cout<<"PF3DTracker::open - I wasn\'t able to allocate memory for _tempMat.\n";
        fflush(stdout);
        quit =true;
    }



   //***********************************
    //Read options from the command line.
    //***********************************
    Bottle initialBottle(config.toString().c_str());

    ConstString initializationFile = initialBottle.check("from",
                                       Value("pf3dTracker.ini"),
                                      "Initialization file (string)").asString();
    
    ConstString context = initialBottle.check("context",
                                       Value("pf3dTracker"),
                                      "Context (string)").asString();

    //create and initialize the resource finder
    ResourceFinder rf;
    rf.setDefaultContext(context.c_str());
    rf.setDefaultConfigFile(initializationFile.c_str());
    rf.configure("ICUB_ROOT", 0, NULL);

    // pass configuration over to bottle
    Bottle botConfig(rf.toString().c_str());
    //botConfig.setMonitor(config.getMonitor()); //is this needed?



    _inputVideoPortName = botConfig.check("inputVideoPort",
                                      Value("/PF3DTracker/videoIn"),
                                      "Input video port (string)").asString();
    _inputVideoPort.open(_inputVideoPortName);

    _outputVideoPortName = botConfig.check("outputVideoPort",
                                       Value("/PF3DTracker/videoOut"),
                                       "Output video port (string)").asString();
    _outputVideoPort.open(_outputVideoPortName);

    _outputDataPortName = botConfig.check("outputDataPort",
                                      Value("/PF3DTracker/dataOut"),
                                      "Output data port (string)").asString();
    _outputDataPort.open(_outputDataPortName);

    _outputParticlePortName = botConfig.check("outputParticlePort",
                                       Value("/PF3DTracker/particleOut"),
                                       "Output particle port (string)").asString();
    _outputParticlePort.open(_outputParticlePortName);

    _outputAttentionPortName = botConfig.check("outputAttentionPort",
                                       Value("/PF3DTracker/attentionOut"),
                                       "Output attention port (string)").asString();
    _outputAttentionPort.open(_outputAttentionPortName);

    _likelihoodThreshold = botConfig.check("likelihoodThreshold",
                                        Value(1),
                                        "Likelihood threshold value (double)").asDouble();

    _attentionOutputMax = botConfig.check("attentionOutputMax",
                                        Value(257),
                                        "attentionOutputMax (double)").asDouble();
    _attentionOutputDecrease = botConfig.check("attentionOutputDecrease",
                                        Value(0.99),
                                        "attentionOutputDecrease (double)").asDouble();




    _nParticles = botConfig.check("nParticles",
                                    Value("1000"),
                                    "Number of particles used in the tracker (int)").asInt();


    _colorTransfPolicy = botConfig.check("colorTransfPolicy",
                                    Value("1"),
                                    "Color transformation policy (int)").asInt();
    if(_colorTransfPolicy!=0 && _colorTransfPolicy!=1)
    {
        cout<<"Color trasformation policy "<<_colorTransfPolicy<<" is not yet implemented."<<endl;
        quit=true; //stop the execution, after checking all the parameters.
    }

    _inside_outside_difference_weight = botConfig.check("insideOutsideDiffWeight",
                                    Value("1.5"),
                                    "Inside-outside difference weight in the likelihood function (double)").asDouble();

    _projectionModel = botConfig.check("projectionModel",
                                       Value("perspective"),
                                       "Projection model (string)").asString();

    if(_projectionModel=="perspective")
    {

        _calibrationImageWidth = botConfig.check("w",
                                        Value(320),
                                        "Image width  (int)").asInt();
        _calibrationImageHeight = botConfig.check("h",
                                        Value(240),
                                        "Image height (int)").asInt();

        _perspectiveFx = botConfig.check("perspectiveFx",
                                        Value(1),
                                        "Focal distance * kx  (double)").asDouble();

        _perspectiveFy = botConfig.check("perspectiveFy",
                                        Value(1),
                                        "Focal distance * ky  (double)").asDouble();

        _perspectiveCx = botConfig.check("perspectiveCx",
                                        Value(1),
                                        "X position of the projection center, in pixels (double)").asDouble();

        _perspectiveCy = botConfig.check("perspectiveCy",
                                        Value(1),
                                        "Y position of the projection center, in pixels (double)").asDouble();
    }
    else
    {
        if(_projectionModel=="equidistance" || _projectionModel=="unified")
        {
            cout<<"Projection model "<<_projectionModel<<" is not yet implemented."<<endl;
            quit=true; //stop the execution, after checking all the parameters.
        }
        else
        {
            cout<<"The projection model you specified ("<<_projectionModel<<") is not supported."<<endl;
            quit=true; //stop the execution, after checking all the parameters.
        }
    }    



    _initializationMethod = botConfig.check("initializationMethod",
                                       Value("search"),
                                       "Initialization method (string)").asString();

    if(_initializationMethod=="3dEstimate")
    {
        _initialX = botConfig.check("initialX",
                                    Value("0"),
                                    "Estimated initial X position [m] (double)").asDouble()*1000; //meters to millimeters
        _initialY = botConfig.check("initialY",
                                    Value("0"),
                                    "Estimated initial Y position [m] (double)").asDouble()*1000; //meters to millimeters
        _initialZ = botConfig.check("initialZ",
                                    Value("1000"),
                                    "Estimated initial Z position [m] (double)").asDouble()*1000; //meters to millimeters
    }
    else
    {
        if(_initializationMethod=="2dEstimate" || _initializationMethod=="search")
        {
            cout<<"Initialization method "<<_initializationMethod<<" is not yet implemented."<<endl;
            quit=true; //stop the execution, after checking all the parameters.
        }
        else
        {
            cout<<"The initialization method you specified ("<<_initializationMethod<<") is not supported."<<endl;
            quit=true; //stop the execution, after checking all the parameters.
        }
    }    



    _trackedObjectType = botConfig.check("trackedObjectType",
                                       Value("sphere"),
                                       "Tracked object type (string)").asString();
    if(_trackedObjectType=="sphere")
    {
        ;//good, this one is implemented.
	//0 means inner and outer circle, 1 means just one circle of the correct size.
	_circleVisualizationMode = botConfig.check("circleVisualizationMode",
                                    Value("0"),
                                    "Visualization mode for the sphere (int)").asInt();
    }
    else
    {
        if(_trackedObjectType=="parallelogram")
        {
            cout<<"Tracked object type "<<_trackedObjectType<<" is not yet implemented."<<endl;
            quit=true; //stop the execution, after checking all the parameters.
        }
        else
        {
            cout<<"The tracked object type you specified ("<<_trackedObjectType<<") is not supported."<<endl;
            quit=true; //stop the execution, after checking all the parameters.
        }
    }



    //*****************************************************
    //Build and read the color model for the tracked object
    //*****************************************************
    //
    trackedObjectColorTemplate = rf.findFile("trackedObjectColorTemplate");
    dataFileName = rf.findFile("trackedObjectTemp");
    //TESTcout<<"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"<<trackedObjectColorTemplate<<endl;
  
    failure=computeTemplateHistogram(trackedObjectColorTemplate,dataFileName);
    if(failure)
    {
        cout<<"I had troubles computing the template histogram."<<endl;
        quit=true;
    }
    

    failure=readModelHistogram(_modelHistogramMat,dataFileName.c_str());
    if(failure)
    {
        cout<<"I had troubles reading the template histogram."<<endl;
        quit=true;
    }





    //*******************************************
    //Read the shape model for the tracked object
    //*******************************************
    trackedObjectShapeTemplate = rf.findFile("trackedObjectShapeTemplate");
    failure=readInitialmodel3dPoints(_model3dPointsMat,trackedObjectShapeTemplate);
    if(failure)
    {
        cout<<"I had troubles reading the model 3D points."<<endl;
        quit=true;
    }

    if((_trackedObjectType=="sphere") && (_circleVisualizationMode==1))
      {
            //create _visualization3dPointsMat and fill it with the average between outer and inner 3D points.
            _visualization3dPointsMat=cvCreateMat( 3, 2*nPixels, CV_32FC1 );
            //only the first half of this matrix is used. the second part can be full of rubbish (not zeros, I guess).
            cvSet(_visualization3dPointsMat,(cvScalar(1)));
            for(row=0;row<3;row++)
            {
                for(column=0;column<nPixels;column++)
                {
                    
                   ((float*)(_visualization3dPointsMat->data.ptr + _visualization3dPointsMat->step*row))[column]=(((float*)(_model3dPointsMat->data.ptr + _model3dPointsMat->step*row))[column]+((float*)(_model3dPointsMat->data.ptr + _model3dPointsMat->step*row))[column+nPixels])/2;
                }
            }
//Used to be done like this:
//         ippsSet_32f(1,&_visualization3dPoints[0][0], 3*2*nPixels);
// 	//COMPUTE THE MEAN OF INNER AND OUTER POINTS
//         ippsAdd_32f(&_model3dPoints[0][0], &_model3dPoints[0][nPixels], &_visualization3dPoints[0][0], nPixels);  //sum the first half line of model3dPoints to the second half line. put the result into the first line of _visualization3dPoints.
//         ippsAdd_32f(&_model3dPoints[1][0], &_model3dPoints[1][nPixels], &_visualization3dPoints[1][0], nPixels);  //same for the second line.
//         ippsAdd_32f(&_model3dPoints[2][0], &_model3dPoints[2][nPixels], &_visualization3dPoints[2][0], nPixels);  //same for the third line.
// 	ippsDivC_32f_I(2, &_visualization3dPoints[0][0], nPixels*2*3); //divide everything by 2. 

      }


    //***************************************************
    //Read the motion model matrix for the tracked object
    //***************************************************

    _accelStDev = botConfig.check("accelStDev",
                                        Value(150),
                                        "StDev of acceleration noise (double)").asDouble();

    motionModelMatrix = rf.findFile("motionModelMatrix");

    //allocate space for the _A matrix. 32bit floats, one channel.
    _A=cvCreateMat(7,7,CV_32FC1);
    failure=readMotionModelMatrix(_A, motionModelMatrix);
    if(failure)
    {
        cout<<"I had troubles reading the motion model matrix."<<endl;
        quit=true;
    }



    //allocate memory for the particles;
    _particles=cvCreateMat(7,_nParticles,CV_32FC1);
    //define ways of accessing the particles:
    _particles1 = cvCreateMatHeader( 1,_nParticles, CV_32FC1);
    cvInitMatHeader( _particles1, 1, _nParticles, CV_32FC1, _particles->data.ptr, _particles->step );
    _particles2 = cvCreateMatHeader( 1,_nParticles, CV_32FC1);
    cvInitMatHeader( _particles2, 1, _nParticles, CV_32FC1, _particles->data.ptr + _particles->step*1, _particles->step );
    _particles3 = cvCreateMatHeader( 1,_nParticles, CV_32FC1);
    cvInitMatHeader( _particles3, 1, _nParticles, CV_32FC1, _particles->data.ptr + _particles->step*2, _particles->step );
    _particles4 = cvCreateMatHeader( 1,_nParticles, CV_32FC1);
    cvInitMatHeader( _particles4, 1, _nParticles, CV_32FC1, _particles->data.ptr + _particles->step*3, _particles->step );
    _particles5 = cvCreateMatHeader( 1,_nParticles, CV_32FC1);
    cvInitMatHeader( _particles5, 1, _nParticles, CV_32FC1, _particles->data.ptr + _particles->step*4, _particles->step );
    _particles6 = cvCreateMatHeader( 1,_nParticles, CV_32FC1);
    cvInitMatHeader( _particles6, 1, _nParticles, CV_32FC1, _particles->data.ptr + _particles->step*5, _particles->step );
    _particles7 = cvCreateMatHeader( 1,_nParticles, CV_32FC1);
    cvInitMatHeader( _particles7, 1, _nParticles, CV_32FC1, _particles->data.ptr + _particles->step*6, _particles->step );
    _particles1to6 = cvCreateMatHeader( 6,_nParticles, CV_32FC1);
    cvInitMatHeader( _particles1to6, 6, _nParticles, CV_32FC1, _particles->data.ptr, _particles->step );


    //allocate memory for the "new" particles;
    _newParticles=cvCreateMat(7,_nParticles,CV_32FC1);
    _newParticles1to6 = cvCreateMatHeader( 6,_nParticles, CV_32FC1);
    cvInitMatHeader( _newParticles1to6, 6, _nParticles, CV_32FC1, _newParticles->data.ptr, _newParticles->step );

    //allocate memory for "noise"
    _noise=cvCreateMat(6,_nParticles,CV_32FC1);
    _noise1 = cvCreateMatHeader( 3,_nParticles, CV_32FC1);
    cvInitMatHeader( _noise1, 3, _nParticles, CV_32FC1, _noise->data.ptr, _noise->step );
    _noise2 = cvCreateMatHeader( 3,_nParticles, CV_32FC1);
    cvInitMatHeader( _noise2, 3, _nParticles, CV_32FC1, _noise->data.ptr + _noise->step*3, _noise->step );

    //resampling-related stuff.
    _nChildren = cvCreateMat(1,_nParticles,CV_32FC1);
    _label     = cvCreateMat(1,_nParticles,CV_32FC1);
    _ramp      = cvCreateMat(1,_nParticles,CV_32FC1);
    _u         = cvCreateMat(1,_nParticles,CV_32FC1);

    _cumWeight =cvCreateMat(1,_nParticles+1,CV_32FC1);

    int count;
    for(count=0;count<_nParticles;count++)
    {
        ((float*)(_ramp->data.ptr))[count]=count+1;
    }


    temp = botConfig.check("saveImagesWithOpencv",
                                      Value("false"),
                                      "Save elaborated images with OpenCV? (string)").asString();
    if(temp=="true")
    {
        _saveImagesWithOpencv=true;
    }

    if(_saveImagesWithOpencv)
    {
        _saveImagesWithOpencvDir = botConfig.check("saveImagesWithOpencvDir",
                                      Value(""),
                                      "Directory where to save the elaborated images (string)").asString();
    }

    if(_initializationMethod=="3dEstimate")
    {
        //*************************************************************************
        //generate a set of random particles near the estimated initial 3D position
        //*************************************************************************
                    
        float mean,velocityStDev;
        velocityStDev=0; //warning ??? !!! I'm setting parameters for the dynamic model here.

        //initialize X
        mean=_initialX;
        cvRandArr( &rngState, _particles1, CV_RAND_NORMAL, cvScalar(mean), cvScalar(_accelStDev));
        //initialize Y
        mean=_initialY;
        cvRandArr( &rngState, _particles2, CV_RAND_NORMAL, cvScalar(mean), cvScalar(_accelStDev));
        //initialize Z
        mean=_initialZ;
        cvRandArr( &rngState, _particles3, CV_RAND_NORMAL, cvScalar(mean), cvScalar(_accelStDev));
        //initialize VX
        mean=0;
        cvRandArr( &rngState, _particles4, CV_RAND_NORMAL, cvScalar(mean), cvScalar(velocityStDev));
        //initialize VY
        cvRandArr( &rngState, _particles5, CV_RAND_NORMAL, cvScalar(mean), cvScalar(velocityStDev));
        //initialize VZ
        cvRandArr( &rngState, _particles6, CV_RAND_NORMAL, cvScalar(mean), cvScalar(velocityStDev));
    }

    downsampler=0; //this thing is used to send less data to the plotter


    //Matrices-related stuff.
    //connect headers to data, allocate space...
    _rzMat = cvCreateMat(3, 3, CV_32FC1);
    _ryMat = cvCreateMat(3, 3, CV_32FC1);
    _uv = cvCreateMat(2,2*nPixels, CV_32FC1);

    _tempMat1 = cvCreateMatHeader( 1,2*nPixels, CV_32FC1);
    cvInitMatHeader( _tempMat1, 1, 2*nPixels, CV_32FC1,_tempMat->data.ptr, _tempMat->step ); 

    _tempMat2 = cvCreateMatHeader( 1,2*nPixels, CV_32FC1);
    cvInitMatHeader( _tempMat2, 1, 2*nPixels, CV_32FC1, _tempMat->data.ptr+_tempMat->step*1, _tempMat->step ); //FUNZIONA? ??? !!!

    _tempMat3 = cvCreateMatHeader( 1,2*nPixels, CV_32FC1);
    cvInitMatHeader( _tempMat3, 1, 2*nPixels, CV_32FC1, _tempMat->data.ptr+_tempMat->step*2, _tempMat->step ); //FUNZIONA? ??? !!!

    _p2Mat1 = cvCreateMatHeader( 1,2*nPixels, CV_32FC1);
    cvInitMatHeader( _p2Mat1, 1, 2*nPixels, CV_32FC1, _points2Mat->data.ptr ); //FUNZIONA? ??? !!!
    _p2Mat3 = cvCreateMatHeader( 1,2*nPixels, CV_32FC1);
    cvInitMatHeader( _p2Mat3, 1, 2*nPixels, CV_32FC1, _points2Mat->data.ptr+_points2Mat->step*2, _points2Mat->step ); //FUNZIONA? ??? !!!

    _drawingMat=cvCreateMat(3, 2*nPixels, CV_32FC1);
    _projectionMat=cvCreateMat(2, 3, CV_32FC1);

    _xyzMat1 = cvCreateMatHeader(1,2*nPixels,CV_32FC1);
    _xyzMat2 = cvCreateMatHeader(1,2*nPixels,CV_32FC1);
    _xyzMat3 = cvCreateMatHeader(1,2*nPixels,CV_32FC1);


    //testOpenCv(); //Used to test stuff.

    //**********************************
    //Write the header line for the data
    //**********************************
    //cout<<"\033[37;1m"<<"  frame#";
    //cout<<"\033[32;1m"<<"   meanX";
    //cout<<"     meanY";
    //cout<<"     meanZ";
    //cout<<"\033[37;1m   Likelihood";
    //cout<<"  Seing";
    //cout<<"\033[33;1m  meanU";
    //cout<<" meanV";
    //cout<<"\033[37;1m  fps"<<"\033[0m"<<endl;

    cout<<"  frame#";
    cout<<"   meanX";
    cout<<"   meanY";
    cout<<"   meanZ";
    cout<<"   Likelihood";
    cout<<"   Seing";
    cout<<"   meanU";
    cout<<"   meanV";
    cout<<"   fps"<<endl;

    //*********************************************************************
    //Read one image from the stream.
    //*********************************************************************
    _yarpImage = _inputVideoPort.read();

    widthRatio=(double)_yarpImage->width()/(double)_calibrationImageWidth;
    heightRatio=(double)_yarpImage->height()/(double)_calibrationImageHeight;
    _perspectiveFx=_perspectiveFx*widthRatio;
    _perspectiveFy=_perspectiveFy*heightRatio;
    _perspectiveCx=_perspectiveCx*widthRatio;
    _perspectiveCy=_perspectiveCy*heightRatio;

    //cout<<"perspectiveFx= "<<_perspectiveFx<<endl;
    //cout<<"perspectiveCx= "<<_perspectiveCx<<endl;
    //cout<<"perspectiveFy= "<<_perspectiveFy<<endl;
    //cout<<"perspectiveCy= "<<_perspectiveCy<<endl;

    _rawImage = cvCreateImage(cvSize(_yarpImage->width(),_yarpImage->height()),IPL_DEPTH_8U, 3); //This allocates space for the image.
    _transformedImage = cvCreateImage(cvSize(_yarpImage->width(),_yarpImage->height()),IPL_DEPTH_8U, 3); //This allocates space for the image.
    cvCvtColor((IplImage*)_yarpImage->getIplImage(), _rawImage, CV_RGB2BGR);
    
    rgbToYuvBinImageLut(_rawImage,_transformedImage,_lut);

    //allocate space for the transformed image.
    _transformedImage = cvCreateImage(cvSize(_yarpImage->width(),_yarpImage->height()),IPL_DEPTH_8U,3);

    



    _framesNotTracking=0;
    _frameCounter=1;
    _attentionOutput=0;

    _lastU=_yarpImage->width()/2;
    _lastV=_yarpImage->height()/2;

    _initialTime=0;
    _finalTime=0;
    _firstFrame=true;

   if(quit==true)
   {
        printf("There were problems initializing the object: the execution was interrupted.\n");
        fflush(stdout);
        return false; //there were problems initializing the objet: stop the execution.
   }
   else
   {
        _doneInitializing=true;
        return true;  //the object was set up successfully.
   }

    

}



//member that closes the object.
bool PF3DTracker::close()
{

    _inputVideoPort.close();
    _outputVideoPort.close();
    _outputDataPort.close();
    _outputParticlePort.close();
    _outputAttentionPort.close();

    cvReleaseMat(&_A);
    cvReleaseMat(&_particles);
    cvReleaseMat(&_newParticles);

    return true;
}

//member that closes the object.
bool PF3DTracker::interruptModule()
{

    _inputVideoPort.interrupt();
    _outputVideoPort.interrupt();
    _outputDataPort.interrupt();
    _outputParticlePort.interrupt();
    _outputAttentionPort.interrupt();

    return true;
}


//member that is repeatedly called by YARP, to give this object the chance to do something.
//should this function return "false", the object would be terminated.
//I already have one image, when I get here (I either acquire it in the initialization method or in the end of this same method).
bool PF3DTracker::updateModule()
{

    if(_doneInitializing)
    {
        int count;
        unsigned int seed;
        float likelihood, mean, maxX, maxY, maxZ;
        float weightedMeanX, weightedMeanY, weightedMeanZ;
        bool failure;
        float meanU;
        float meanV;
        float wholeCycle;
        string outputFileName;
        stringstream out;
        
        seed=rand();    
    
    
        _finalTime=yarp::os::Time::now();
        wholeCycle=_finalTime-_initialTime;
        _initialTime=yarp::os::Time::now();
    
    
        //*****************************************
        //calculate the likelihood of each particle
        //*****************************************
        float sumLikelihood=0.0;
        float maxLikelihood=0.0;
        int   maxIndex=-1;
        for(count=0;count< _nParticles;count++)
        {
            if(_colorTransfPolicy==0)
            {
                  evaluateHypothesisPerspective(_model3dPointsMat,cvmGet(_particles,0,count),cvmGet(_particles,1,count),cvmGet(_particles,2,count),_modelHistogramMat,_transformedImage,_perspectiveFx,_perspectiveFy, _perspectiveCx,_perspectiveCy,_inside_outside_difference_weight,likelihood);
            }
            else
            {
                if(_colorTransfPolicy==1)
                {
                  evaluateHypothesisPerspectiveFromRgbImage(_model3dPointsMat,cvmGet(_particles,0,count),cvmGet(_particles,1,count),cvmGet(_particles,2,count),_modelHistogramMat,_rawImage,_perspectiveFx,_perspectiveFy, _perspectiveCx,_perspectiveCy,_inside_outside_difference_weight,likelihood);
                }
                else
                {
                    cout<<"Wrong ID for color transformation policy:"<<_colorTransfPolicy<<". Quitting.\n";
                    exit(1);
                }
            }
    
            cvmSet(_particles,6,count,likelihood);
            sumLikelihood+=likelihood;
            if(likelihood>maxLikelihood)
            {
                maxLikelihood=likelihood;
                maxIndex=count;
            }
        }
    
    
        if(maxIndex!=-1)
        {
            maxX=cvmGet(_particles,0,maxIndex);
            maxY=cvmGet(_particles,1,maxIndex);
            maxZ=cvmGet(_particles,2,maxIndex);
        }
        else
        {
            maxX=1;
            maxY=1;
            maxZ=1000;
        }
    
        if(maxLikelihood/exp((float)20.0)>_likelihoodThreshold) //normalizing likelihood
        {
          _seeingObject=1;
          _framesNotTracking=0;
          _attentionOutput=_attentionOutputMax;
        }
        else
        {
          _attentionOutput=_attentionOutput*_attentionOutputDecrease;    
          _seeingObject=0;
          _framesNotTracking+=1;
        }
    
    //******************************************************
    //     //send data to the plotter. COMMENTED FOR EFFICENCY
    //******************************************************
    //     if(downsampler % 3 ==0)
    //     {
    //         //these values might be a mix of the ones before and after the resampling took place. ??? WARNING
    //         Bottle& particleOutput=_outputParticlePort.prepare();
    //         particleOutput.clear();
    //         for(count=0;count<_nParticles;count++)
    //         {
    //             particleOutput.addDouble((double)(_particles[0][count]));
    //             particleOutput.addDouble((double)(_particles[1][count]));
    //             particleOutput.addDouble((double)(_particles[2][count]));
    //             particleOutput.addDouble((double)(_particles[3][count]));
    //             particleOutput.addDouble((double)(_particles[4][count]));
    //             particleOutput.addDouble((double)(_particles[5][count]));
    //             particleOutput.addDouble((double)(_particles[6][count]));
    //         }
    //         _outputParticlePort.write();
    //     }
    //     downsampler+=1;
    
        //If the likelihood has been under the threshold for 5 frames, reinitialize the tracker.
        //This just works for the sphere.
        if(_framesNotTracking==5 || sumLikelihood==0.0)
        {
            cout<<"**********************************************************************Reset\n";
            float mean,velocityStDev;
            velocityStDev=0; //warning ??? !!! I'm setting parameters for the dynamic model here.
            unsigned int seed;
    
            mean=_initialX;
            cvRandArr( &rngState, _particles1, CV_RAND_NORMAL, cvScalar(mean), cvScalar(_accelStDev));
            //initialize Y
            mean=_initialY;
            cvRandArr( &rngState, _particles2, CV_RAND_NORMAL, cvScalar(mean), cvScalar(_accelStDev));
            //initialize Z
            mean=_initialZ;
            cvRandArr( &rngState, _particles3, CV_RAND_NORMAL, cvScalar(mean), cvScalar(_accelStDev));
            //initialize VX
            mean=0;
            cvRandArr( &rngState, _particles4, CV_RAND_NORMAL, cvScalar(mean), cvScalar(velocityStDev));
            //initialize VY
            cvRandArr( &rngState, _particles5, CV_RAND_NORMAL, cvScalar(mean), cvScalar(velocityStDev));
            //initialize VZ
            cvRandArr( &rngState, _particles6, CV_RAND_NORMAL, cvScalar(mean), cvScalar(velocityStDev));
    
              _framesNotTracking=0;
    
          weightedMeanX=weightedMeanY=weightedMeanZ=0.0;    // UGO: they should be zeroed before accumulation
          for(count=0;count<_nParticles;count++)
          {
              weightedMeanX+=cvmGet(_particles,0,count);
              weightedMeanY+=cvmGet(_particles,1,count);
              weightedMeanZ+=cvmGet(_particles,2,count);
          }
          weightedMeanX/=_nParticles;
          weightedMeanY/=_nParticles;
          weightedMeanZ/=_nParticles;
          //this mean is not weighted as there is no weight to use: the particles have just been generated.
    
    
          //*****************************************
          //WRITE ESTIMATES TO THE SCREEN, FIRST PART
          //*****************************************
          //these are not really estimates, but... 
          cout<<setw(8)<<_frameCounter;
          cout<<setiosflags(ios::fixed)<<setprecision(3)<<setw(8)<<weightedMeanX/1000; //millimeters to meters
          cout<<"  "<<setw(8)<<weightedMeanY/1000; //millimeters to meters
          cout<<"  "<<setiosflags(ios::fixed)<<setprecision(3)<<setw(8)<<weightedMeanZ/1000; //millimeters to meters
          cout<<"  "<<setiosflags(ios::fixed)<<setprecision(5)<<setw(8)<<maxLikelihood/exp((float)20.0); //normalizing likelihood
          cout<<"  "<<setw(5)<<_seeingObject;
        }
        else
        {      
          //*********************************************
          //Compute the mean and normalize the likelihood
          //*********************************************
          weightedMeanX=0.0;
          weightedMeanY=0.0;
          weightedMeanZ=0.0;
          for(count=0;count<_nParticles;count++)
          {
            cvmSet(_particles,6,count,(cvmGet(_particles,6,count)/sumLikelihood));
            weightedMeanX+=cvmGet(_particles,0,count)*cvmGet(_particles,6,count);
            weightedMeanY+=cvmGet(_particles,1,count)*cvmGet(_particles,6,count);
            weightedMeanZ+=cvmGet(_particles,2,count)*cvmGet(_particles,6,count);
          }
    
    
    
          //*****************************************
          //WRITE ESTIMATES TO THE SCREEN, FIRST PART
          //*****************************************
          //cout<<"\033[37;1m"<<setw(8)<<_frameCounter;
          //cout<<" \033[32;1m"<<setiosflags(ios::fixed)<<setprecision(3)<<setw(8)<<weightedMeanX/1000; //millimeters to meters
          //cout<<"  "<<setw(8)<<weightedMeanY/1000; //millimeters to meters
          //cout<<"  "<<setiosflags(ios::fixed)<<setprecision(3)<<setw(8)<<weightedMeanZ/1000; //millimeters to meters
          //cout<<"  \033[37;1m"<<setiosflags(ios::fixed)<<setprecision(5)<<setw(8)<<maxLikelihood/exp((float)20.0); //normalizing likelihood
          //cout<<"  "<<setw(5)<<_seeingObject;
    
          cout<<setw(8)<<_frameCounter;
          cout<<setiosflags(ios::fixed)<<setprecision(3)<<setw(8)<<weightedMeanX/1000; //millimeters to meters
          cout<<"  "<<setw(8)<<weightedMeanY/1000; //millimeters to meters
          cout<<"  "<<setiosflags(ios::fixed)<<setprecision(3)<<setw(8)<<weightedMeanZ/1000; //millimeters to meters
          cout<<"  "<<setiosflags(ios::fixed)<<setprecision(5)<<setw(8)<<maxLikelihood/exp((float)20.0); //normalizing likelihood
          cout<<"  "<<setw(5)<<_seeingObject;
    
    
          //**********************
          //RESAMPLE THE PARTICLES
          //**********************
          //cout<<"T1\n";
          //fflush(stdout);
          int minimum_likelihood=10; //do not resample if maximum likelihood is lower than this.
                                      //this is intended to prevent that the particles collapse on the origin when you start the tracker.
          if(maxLikelihood>minimum_likelihood)
          {
            //cout<<"T2\n";
            //fflush(stdout);
    
            //TODO non funziona ancora, credo: nelle particelle resamplate ci sono dei not-a-number.
            //systematicR(_particles1to6,_particles7,_newParticles);   //SOMETHING'S WRONG HERE: sometimes the new particles look like being messed up ??? !!!
              systematic_resampling(_particles1to6,_particles7,_newParticles,_cumWeight);
            //cout<<"T4\n";
    
          }
          else //I can't apply a resampling with all weights equal to 0!
          {
            //cout<<"T3\n";
            //fflush(stdout);
    
            //TODO:CHECK that copying the whole thing creates no problems.
            //I think I used to copy only 6 lines to make it faster.
            //ippsCopy_32f(&_particles[0][0], &_newParticles[0][0], 6*_nParticles);
            cvCopy(_particles,_newParticles);
          }
    
            //cout<<"after resampling\n";
            //printMat(_newParticles);
    
    
          //the "good" particles now are in _newParticles
          //******************************************
          //APPLY THE MOTION MODEL: 1.APPLY THE MATRIX
          //******************************************
          cvMatMul(_A,_newParticles,_particles);
    
    
          //the "good" particles now are in _particles
          //********************************************************
          //APPLY THE MOTION MODEL: 2.ADD THE EFFECT OF ACCELERATION
          //********************************************************
    
            cvRandArr( &rngState, _noise1, CV_RAND_NORMAL, cvScalar(mean), cvScalar(_accelStDev));
            cvCopy(_noise1,_noise2);
            cvConvertScale( _noise1, _noise1, 0.5, 0 );//influence on the position is half that on speed.
            cvAdd(_particles1to6,_noise,_particles1to6);//sum the influence of the noise to the previous status
            //used to be like this:
                //       IPP32F NOISE[6*_NPARTICLES];
                //       MEAN=0;
                //       IPPSRANDGAUSS_DIRECT_32F(NOISE, _NPARTICLES*3, MEAN, STDEV, &SEED);
                //       IPPSCOPY_32F(NOISE, NOISE+_NPARTICLES*3,_NPARTICLES*3);  //COPY NOISE FIRST THREE COLUMNS ON THE LAST THREE.
                //       IPPSMULC_32F_I(0.5, NOISE, _NPARTICLES*3);  //INFLUENCE ON THE POSITION IS HALF THAT ON SPEED.
                //       IPPSADD_32F(NOISE, &_NEWPARTICLES[0][0], &_PARTICLES[0][0], _NPARTICLES*3);  //SUM NOISE'S FIRST THREE LINES TO _NEWPARTICLES' AND PUT THE RESULT IN THE FIRST THREE LINES OF PARTICLES.
                //       ippsAdd_32f(noise+_nParticles*3, &_newParticles[3][0], &_particles[3][0], _nParticles*3);  //sum the second batch.
    
    
    
        }
    
    
        //************************************
        //DRAW THE SAMPLED POINTS ON THE IMAGE
        //************************************
      
    
    
        if(_circleVisualizationMode==0)
        {
          drawSampledLinesPerspectiveYARP(_model3dPointsMat, weightedMeanX,weightedMeanY,weightedMeanZ, _yarpImage,_perspectiveFx, _perspectiveFy, _perspectiveCx, _perspectiveCy, 255, 255, 255, meanU, meanV);
        }
        if(_circleVisualizationMode==1)
        {
          if(_seeingObject)
            drawContourPerspectiveYARP(_visualization3dPointsMat, weightedMeanX,weightedMeanY,weightedMeanZ, _yarpImage,_perspectiveFx, _perspectiveFy, _perspectiveCx, _perspectiveCy, 0, 255, 0, meanU, meanV);
          else
            drawContourPerspectiveYARP(_visualization3dPointsMat, weightedMeanX,weightedMeanY,weightedMeanZ, _yarpImage,_perspectiveFx, _perspectiveFy, _perspectiveCx, _perspectiveCy, 255,255, 0, meanU, meanV);
        }
    
    
    
        //******************************************
        //WRITE ESTIMATES TO THE SCREEN, SECOND PART
        //******************************************
        //cout<<"\033[33;1m"<<setw(8)<<(int)meanU;
        //cout<<setw(5)<<(int)meanV<<"\033[0m";
        //if(_firstFrame==false)
        //{
        //    cout<<setw(5)<<"\033[37;1m"<<setw(8)<<setiosflags(ios::fixed)<<setprecision(3)<<wholeCycle<<"\033[0m"<<endl;      
        //}
        //else
        //{
        //    cout<<"\033[37;1m   -----\033[0m"<<endl;
        //    _firstFrame=false;
        //}
    
        cout<<setw(8)<<(int)meanU;
        cout<<setw(5)<<(int)meanV;
        if(_firstFrame==false)
        {
            cout<<setw(5)<<setw(8)<<setiosflags(ios::fixed)<<setprecision(3)<<wholeCycle<<endl;      
        }
        else
        {
            cout<<"   -----"<<endl;
            _firstFrame=false;
        }
    
    
    
        Bottle& output=_outputDataPort.prepare();
        output.clear();
        output.addDouble(weightedMeanX/1000);//millimeters to meters
        output.addDouble(weightedMeanY/1000);//millimeters to meters
        output.addDouble(weightedMeanZ/1000);//millimeters to meters
        output.addDouble(maxLikelihood/exp((float)20.0));//normalizing likelihood
        output.addDouble(meanU);
        output.addDouble(meanV);
        output.addDouble(_seeingObject);
        _outputDataPort.write();
    
        VectorOf<double>& tempVector=_outputAttentionPort.prepare();
        tempVector.resize(5);
        if(maxLikelihood>_likelihoodThreshold)
        {
          tempVector(0) = (meanU-_rawImage->width/2)/(_rawImage->width/2)/1.5; //X=(U-width/2) / width
          tempVector(1) = (meanV-_rawImage->height/2)/(_rawImage->height/2)/1.5;//Y= -(V-height/2) / height
          //_lastU=meanU; I just keep it to the center of the image, for now.
          //_lastV=meanV;I just keep it to the center of the image, for now.
        }
        else //keep sending the old value, when likelihood is under threshold
        {
          tempVector(0) = ((_lastU-_rawImage->width/2)/(_rawImage->width/2)); //X=(U-width/2) / width
          tempVector(1) = ((_lastV-_rawImage->height/2)/(_rawImage->height/2));//Y= -(V-height/2) / height
        }
    
        tempVector(2) = 0;
        tempVector(3) = 0;
        tempVector(4) = _attentionOutput;
        _outputAttentionPort.write();
    
    
    
    
        //************************************
        //Write the elaborated image to a file
        //************************************
        //I should use the output video port, instead.
        //write image to file, openCV.
        if(_saveImagesWithOpencv)
        {
            if(_frameCounter<1000) out << 0;
            if(_frameCounter<100) out << 0;
            if(_frameCounter<10) out << 0;
            out << _frameCounter;
            outputFileName=_saveImagesWithOpencvDir+out.str()+".jpeg";
            cvCvtColor((IplImage*)_yarpImage->getIplImage(), _rawImage, CV_RGB2BGR); //convert the annotated image.
            cvSaveImage(outputFileName.c_str(), _rawImage);
        }
    
        
        //write the elaborated image on the output port.
        _outputVideoPort.prepare() =  *_yarpImage;
        _outputVideoPort.write();
    
        _frameCounter++;
        //_secondCpuClocks=ippGetCpuClocks();
        //_thirdCpuClocks=ippGetCpuClocks();
    
    
        //*******************
        //acquire a new image
        //*******************
        _yarpImage = _inputVideoPort.read(); //read one image from the buffer.
    
        //_fourthCpuClocks=ippGetCpuClocks();
        cvCvtColor((IplImage*)_yarpImage->getIplImage(), _rawImage, CV_RGB2BGR);
        //_fifthCpuClocks=ippGetCpuClocks();
    
        //*************************************
        //transform the image in the YUV format
        //*************************************
        if(_colorTransfPolicy==0)
        {
            rgbToYuvBinImageLut(_rawImage,_transformedImage,_lut);
        }
        else
        {
            //do nothing!
        }

    }
    else //if initialization has not finished, do nothing.
    {

    }

    return true; //continue. //in this case it means everything is fine.
}







void PF3DTracker::drawSampledLinesPerspectiveYARP(CvMat* model3dPointsMat, float x, float y, float z, ImageOf<PixelRgb> *image,float _perspectiveFx,float  _perspectiveFy ,float _perspectiveCx,float  _perspectiveCy ,int R, int G, int B, float &meanU, float &meanV)
{


    bool failure;
    //CvMat* uv=cvCreateMat(2,2*nPixels,CV_32FC1);

    //create a copy of the 3D original points.
    cvCopy(model3dPointsMat,_drawingMat);

    //used to be:
    //     Ipp32f model3dPointsDuplicate[3][2*nPixels];
    //     //printMatrix(&model3dPoints[0][0],2*nPixels,3);
    //     failure = ippsCopy_32f(&model3dPoints[0][0], &model3dPointsDuplicate[0][0],3*2*nPixels);
    //     //printMatrix(&model3dPointsDuplicate[0][0],2*nPixels,3);



    //****************************
    //ROTOTRANSLATE THE 3D POINTS.
    //****************************
        failure=place3dPointsPerspective(_drawingMat,x,y,z);
        //cout<<"rototraslated points:\n";
        //printMatrix(&model3dPointsDuplicate[0][0],2*nPixels,3);





    //***********************
    //PROJECT 3D POINTS TO 2D
    //***********************
        failure= perspective_projection(_drawingMat, _perspectiveFx, _perspectiveFy, _perspectiveCx, _perspectiveCy, _uv);
        if(failure)
        {
            cout<<"I had troubles projecting the points.\n";
        }
        //used to be:
        //         //(Ipp32f *xyz, int xyzColumns, int xyzRows, int xyzStride1, int xyzStride2, float fx, float fy, Ipp32f u0, Ipp32f v0, Ipp32f *uv, int uvStride1, int uvStride2
        //         failure=perspective_projection(&model3dPointsDuplicate[0][0],2*nPixels,3,sizeof(Ipp32f)*2*nPixels,sizeof(Ipp32f),_perspectiveFx,  _perspectiveFy , _perspectiveCx,  _perspectiveCy,&uv[0][0],sizeof(Ipp32f)*2*nPixels,sizeof(Ipp32f));
        //
        //         if(failure)
        //         {
        //             cout<<"I had troubles projecting the points.\n";
        //         }




    //DRAW    
    int conta,cippa,lippa,uPosition,vPosition;
    meanU=0;
    meanV=0;
    for(conta=0;conta<nPixels;conta++)
    {
        meanU=meanU+((float*)(_uv->data.ptr + _uv->step*0))[conta];
        meanV=meanV+((float*)(_uv->data.ptr + _uv->step*1))[conta];


                vPosition= (int)((float*)(_uv->data.ptr + _uv->step*1))[conta];
                uPosition= (int)((float*)(_uv->data.ptr + _uv->step*0))[conta];
                if((uPosition<_rawImage->width)&&(uPosition>=0)&&(vPosition<_rawImage->height)&&(vPosition>=0))
                {
                    image->pixel(uPosition,vPosition)= PixelRgb(R,G,B);
                }
                vPosition= (int)((float*)(_uv->data.ptr + _uv->step*1))[conta+nPixels];
                uPosition= (int)((float*)(_uv->data.ptr + _uv->step*0))[conta+nPixels];
                if((uPosition<_rawImage->width)&&(uPosition>=0)&&(vPosition<_rawImage->height)&&(vPosition>=0))
                {
                    image->pixel(uPosition,vPosition)= PixelRgb(R,G,B);
                }

    }

    meanU=floor(meanU/nPixels);
    meanV=floor(meanV/nPixels);
    if((meanU<_rawImage->width)&&(meanU>=0)&&(meanV<_rawImage->height)&&(meanV>=0))
    {
        image->pixel((int)meanU,(int)meanV)= PixelRgb(R,G,B);
    }


}



void PF3DTracker::drawContourPerspectiveYARP(CvMat* model3dPointsMat,float x, float y, float z, ImageOf<PixelRgb> *image,float _perspectiveFx,float  _perspectiveFy ,float _perspectiveCx,float  _perspectiveCy ,int R, int G, int B, float &meanU, float &meanV)
{


    bool failure;
    //CvMat* uv=cvCreateMat(2,2*nPixels,CV_32FC1);

    //create a copy of the 3D original points.
    cvCopy(model3dPointsMat,_drawingMat);

    //used to be:
    //     Ipp32f model3dPointsDuplicate[3][2*nPixels];
    //     //printMatrix(&model3dPoints[0][0],2*nPixels,3);
    //     failure = ippsCopy_32f(&model3dPoints[0][0], &model3dPointsDuplicate[0][0],3*2*nPixels);
    //     //printMatrix(&model3dPointsDuplicate[0][0],2*nPixels,3);



    //****************************
    //ROTOTRANSLATE THE 3D POINTS.
    //****************************
        failure=place3dPointsPerspective(_drawingMat,x,y,z);
        //cout<<"rototraslated points:\n";
        //printMatrix(&model3dPointsDuplicate[0][0],2*nPixels,3);





    //***********************
    //PROJECT 3D POINTS TO 2D
    //***********************
        failure= perspective_projection(_drawingMat, _perspectiveFx, _perspectiveFy, _perspectiveCx, _perspectiveCy, _uv);
        if(failure)
        {
            cout<<"I had troubles projecting the points.\n";
        }
        //used to be:
        //         //(Ipp32f *xyz, int xyzColumns, int xyzRows, int xyzStride1, int xyzStride2, float fx, float fy, Ipp32f u0, Ipp32f v0, Ipp32f *uv, int uvStride1, int uvStride2
        //         failure=perspective_projection(&model3dPointsDuplicate[0][0],2*nPixels,3,sizeof(Ipp32f)*2*nPixels,sizeof(Ipp32f),_perspectiveFx,  _perspectiveFy , _perspectiveCx,  _perspectiveCy,&uv[0][0],sizeof(Ipp32f)*2*nPixels,sizeof(Ipp32f));
        //
        //         if(failure)
        //         {
        //             cout<<"I had troubles projecting the points.\n";
        //         }



    //****
    //Draw
    //****
    int conta,cippa,lippa,uPosition,vPosition;
    meanU=0;
    meanV=0;
    for(conta=0;conta<nPixels;conta++)
    {
        meanV=meanV+((float*)(_uv->data.ptr + _uv->step*1))[conta];
        meanU=meanU+((float*)(_uv->data.ptr + _uv->step*0))[conta];

        for(lippa=-2;lippa<3;lippa++)
            for(cippa=-2;cippa<3;cippa++)
            {
                vPosition= (int)(((float*)(_uv->data.ptr + _uv->step*1))[conta])+lippa-1;
                uPosition= (int)(((float*)(_uv->data.ptr + _uv->step*0))[conta])+cippa-1;

                if((uPosition<_rawImage->width)&&(uPosition>=0)&&(vPosition<_rawImage->height)&&(vPosition>=0))
                {
                    image->pixel(uPosition,vPosition)= PixelRgb(R,G,B);
                }

            }
    }

    meanU=floor(meanU/nPixels);
    meanV=floor(meanV/nPixels);
    if((meanU<_rawImage->width)&&(meanU>=0)&&(meanV<_rawImage->height)&&(meanV>=0))
    {
        image->pixel((int)meanU,(int)meanV)= PixelRgb(R,G,B);
    }

}





bool PF3DTracker::computeTemplateHistogram(string imageFileName,string dataFileName)
{
    int u,v,a,b,c;
    float usedPoints=0;
    //float histogram[YBins][UBins][VBins];
    //float* histogram;
    //histogram = new float[YBins*UBins*VBins]
    int dimensions;
    dimensions=3;
    int sizes[3]={YBins,UBins,VBins};
    //create histogram and allocate memory for it.
    CvMatND* histogram=cvCreateMatND(dimensions, sizes, CV_32FC1);
    if(histogram==0)
    {
        cout<<"computeTemplateHistogram: I wasn\'t able to allocate memory for histogram.\n";
        fflush(stdout);
        return -1; //if I can't do it, I just quit the program.
    }
    //set content of the matrix to zero.
    cvSetZero(histogram);//used to be: ipps Zero_32f(&histogram[0][0][0], YBins*UBins*VBins);
    IplImage *rawImage;
    IplImage* transformedImage;
    
    //load the image
    if( (rawImage = cvLoadImage( imageFileName.c_str(), 1)) == 0 ) //load the image from file.
    {
        cout<<"I wasn't able to open the image file!\n";
        fflush(stdout);
        return -1; //if I can't do it, I just quit the program.
    }

    //allocate space for the transformed image
    transformedImage = cvCreateImage(cvSize(rawImage->width,rawImage->height),IPL_DEPTH_8U,3);

    //transform the image in the YUV format
    rgbToYuvBinImageLut(rawImage,transformedImage,_lut);
    
    //count the frequencies of colour bins, build the histogram.
    for(v=0;v<rawImage->height;v++)
        for(u=0;u<rawImage->width;u++)
        {
            //discard white pixels [255,255,255].
            if(!(
                    (((uchar*)(rawImage->imageData + rawImage->widthStep*v))[u*3+0])==255 && (((uchar*)(rawImage->imageData + rawImage->widthStep*v))[u*3+1])==255 && (((uchar*)(rawImage->imageData + rawImage->widthStep*v))[u*3+2])==255) 

                )
            {


                a=(((uchar*)(transformedImage->imageData + transformedImage->widthStep*v))[u*3+0]);//Y bin
                b=(((uchar*)(transformedImage->imageData + transformedImage->widthStep*v))[u*3+1]);//U bin
                c=(((uchar*)(transformedImage->imageData + transformedImage->widthStep*v))[u*3+2]);//V bin

                //TEST printf("histogram->size[0].step,%d\n",histogram->dim[0].step);  256
                //TEST printf("histogram->size[1].step,%d\n",histogram->dim[1].step);   32
                //TEST printf("histogram->size[2].step,%d\n",histogram->dim[2].step);    4
                *((float*)(histogram->data.ptr + a*histogram->dim[0].step + b*histogram->dim[1].step + c*histogram->dim[2].step)) +=1;

                //initial pointer + Y*UBINS*VBINS*histogram->step + U*VBINS*histogram->step + V*histogram->step. RIGHT?
                //histogram[(yuvBinsImage[u][v][0])*UBins*VBins + (yuvBinsImage[u][v][1])*VBins +  (yuvBinsImage[u][v][2]) ]+=1; //increment the correct bin counter.
                usedPoints+=1;
            }
            
        }

    //normalize
    if(usedPoints>0)  
    {
        //histogram=histogram/usedPoints
        cvConvertScale( histogram, histogram, 1/usedPoints, 0 );
       //used to be: ipps DivC_32f_I(usedPoints, &histogram[0][0][0], YBins*UBins*VBins);
    }

    //write the computed histogram to a file.
    ofstream fout(dataFileName.c_str());//open file
    if(!fout)                           //confirm file opened
    {
        cout << "computeTemplateHistogram: unable to open the csv file to store the histogram.\n" << endl;
        fflush(stdout);
        return true;
    }
    else
    {
        for(a=0;a<YBins;a++)
        {
            for(b=0;b<UBins;b++)
            {
                for(c=0;c<VBins;c++)
                 {
                    fout<<*((float*)(histogram->data.ptr + a*histogram->dim[0].step + b*histogram->dim[1].step + c*histogram->dim[2].step))<<endl;
                    //used to be: fout<<(float)(histogram[a*UBins*VBins + b*VBins + c])<<endl;
                 }
            }
        }
        fout.close();
    }

    //clean memory up
    cvReleaseMatND(&histogram);
    cvReleaseImage(&rawImage);
    return false;

}

bool PF3DTracker::readModelHistogram(CvMatND* histogram,const char fileName[])
{
    int c1,c2,c3;
    char line[15];
            
    ifstream fin(fileName);//open file
    if(!fin)                           //confirm file opened
    {
        cout << "unable to open the csv histogram file.\n" << endl;
        fflush(stdout);
        return true;
    }
    else
    {
        for(c1=0;c1<YBins;c1++) 
            for(c2=0;c2<UBins;c2++) 
                for(c3=0;c3<VBins;c3++) 
                {
                    fin.getline(line, 14);
                    *((float*)(histogram->data.ptr + c1*histogram->dim[0].step + c2*histogram->dim[1].step + c3*histogram->dim[2].step))=atof(line);
                    //TEST cout << c1 <<" "<<c2<<" "<<c3 << endl;
                    //TEST fflush(stdout);
                }   
        return false;
    }
}



bool PF3DTracker::readInitialmodel3dPoints(CvMat* points, string fileName)
{
    int c1,c2;
    char line[15];
            
    ifstream fin(fileName.c_str());//open file
    if(!fin)                           //confirm file opened
    {
        cout << "unable to open the the 3D model file.\n" << endl;
        fflush(stdout);
        return true;
    }
    else
    {
        for(c1=0;c1<3;c1++) 
            for(c2=0;c2<2*nPixels;c2++) 
        {
            fin.getline(line, 14);
            ((float*)(points->data.ptr + points->step*c1))[c2]=atof(line);
        }   
        return false;
    }
}



bool PF3DTracker::readMotionModelMatrix(CvMat* points, string fileName)
{
    int c1,c2;
    char line[15];
            
    ifstream fin(fileName.c_str());//open file
    if(!fin)                           //confirm file opened
    {
        cout << "unable to open the motion model file.\n" << endl;
        fflush(stdout);
        return true;
    }
    else
    {
        for(c1=0;c1<7;c1++)
            for(c2=0;c2<7;c2++)
        {
            fin.getline(line, 14);
            cvmSet(points,c1,c2,atof(line));
        }   
        return false;
    }
}


bool PF3DTracker::evaluateHypothesisPerspective(CvMat* model3dPointsMat,float x, float y, float z, CvMatND* modelHistogramMat, IplImage* transformedImage,  float fx, float fy, float u0, float v0, float inside_outside, float &likelihood)
{


    bool failure;
    float usedOuterPoints, usedInnerPoints;

    //create a copy of the 3D original points.
    cvCopy(model3dPointsMat,_drawingMat);

    //****************************
    //ROTOTRANSLATE THE 3D POINTS.
    //****************************
    failure=place3dPointsPerspective(_drawingMat,x,y,z);



    //***********************
    //PROJECT 3D POINTS TO 2D
    //***********************
    failure= perspective_projection(_drawingMat, _perspectiveFx, _perspectiveFy, _perspectiveCx, _perspectiveCy, _uv);
    if(failure)
    {
        cout<<"I had troubles projecting the points.\n";
    }
    

    computeHistogram(_uv, transformedImage,  _innerHistogramMat, usedInnerPoints, _outerHistogramMat, usedOuterPoints);
    //if((usedInnerPoints<nPixels)||(usedOuterPoints<nPixels))
    //    likelihood=0;
    //else
    failure=calculateLikelihood(_modelHistogramMat, _innerHistogramMat,_outerHistogramMat, inside_outside,likelihood);


    likelihood=exp(20*likelihood); //no need to divide: I'm normalizing later.

    //make hypotheses with pixels outside the image less likely.
    likelihood=likelihood*((float)usedInnerPoints/nPixels)*((float)usedInnerPoints/nPixels)*((float)usedOuterPoints/nPixels)*((float)usedOuterPoints/nPixels);

    return false;
}





bool PF3DTracker::evaluateHypothesisPerspectiveFromRgbImage(CvMat* model3dPointsMat,float x, float y, float z, CvMatND* modelHistogramMat, IplImage *image,  float fx, float fy, float u0, float v0, float inside_outside, float &likelihood)
{
//TODO

    bool failure;
    float usedOuterPoints, usedInnerPoints;

    //create a copy of the 3D original points.
    cvCopy(model3dPointsMat,_drawingMat);

    //****************************
    //ROTOTRANSLATE THE 3D POINTS.
    //****************************
    failure=place3dPointsPerspective(_drawingMat,x,y,z);

    //***********************
    //PROJECT 3D POINTS TO 2D
    //***********************
    failure= perspective_projection(_drawingMat, _perspectiveFx, _perspectiveFy, _perspectiveCx, _perspectiveCy, _uv);
    if(failure)
    {
        cout<<"I had troubles projecting the points.\n";
    }
    


    computeHistogramFromRgbImage(_uv, image,  _innerHistogramMat, usedInnerPoints, _outerHistogramMat, usedOuterPoints);
    //if((usedInnerPoints<nPixels)||(usedOuterPoints<nPixels))
    //    likelihood=0;
    //else
    failure=calculateLikelihood(_modelHistogramMat, _innerHistogramMat, _outerHistogramMat, inside_outside,likelihood);
    
    likelihood=exp(20*likelihood); //no need to divide: I'm normalizing later.
    
    //make hypotheses with pixels outside the image less likely.
    likelihood=likelihood*((float)usedInnerPoints/nPixels)*((float)usedInnerPoints/nPixels)*((float)usedOuterPoints/nPixels)*((float)usedOuterPoints/nPixels);
    
    return false;
}



bool PF3DTracker::systematic_resampling(CvMat* oldParticlesState, CvMat* oldParticlesWeights, CvMat* newParticlesState, CvMat* cumWeight)
{
    //function [newParticlesState] = systematic_resampling(oldParticlesWeight, oldParticlesState)

    double u; //random number [0,1)
    double sum;
    int c1;
    int rIndex;  //index of the randomized array
    int cIndex;  //index of the cumulative weight array. cIndex -1 indicates which particle we think of resampling.
    int npIndex; //%new particle index, tells me how many particles have been created so far.

    //%N is the number of particles.
    //[lines, N] = size(oldParticlesWeight);
    //in CPP, _nParticles is the number of particles.


    //%NORMALIZE THE WEIGHTS, so that sum(oldParticles)=1.
    //oldParticlesWeight = oldParticlesWeight / sum(oldParticlesWeight);
    sum=0;
    for(c1=0;c1<_nParticles;c1++)
    {
        sum+=((float*)(oldParticlesWeights->data.ptr + oldParticlesWeights->step*0))[c1];
    }
    for(c1=0;c1<_nParticles;c1++)
    {
        ((float*)(oldParticlesWeights->data.ptr + oldParticlesWeights->step*0))[c1] = (((float*)(oldParticlesWeights->data.ptr + oldParticlesWeights->step*0))[c1])/sum;
    }


    //%GENERATE N RANDOM VALUES
    //u = rand(1)/N; %random value [0,1/N)
    u=1/(double)_nParticles*((double)rand()/(double)RAND_MAX);

    //%the randomized values are going to be u, u+1/N, u+2/N, etc.
    //%instread of accessing this vector, the elements are computed on the fly:
    //%randomVector(a)= (a-1)/N+u.

    //%COMPUTE THE ARRAY OF CUMULATIVE WEIGHTS
    //cumWeight=zeros(1,N+1);
    //cumWeight[0]=0;
    ((float*)(cumWeight->data.ptr))[0]=0;
    for(c1=0;c1<_nParticles;c1++)
    {
        //cumWeight[c1+1]=cumWeight[c1]+oldParticlesWeight[c1];

        ((float*)(cumWeight->data.ptr))[c1+1]=((float*)(cumWeight->data.ptr))[c1]+((float*)(oldParticlesWeights->data.ptr + oldParticlesWeights->step*0))[c1];
        //cout<<"cumulative at position "<<c1+1<<" = "<<((float*)(cumWeight->data.ptr))[c1+1]<<endl;

    }
    //CHECK IF THERE IS SOME ROUNDING ERROR IN THE END OF THE ARRAY.
    //if(cumWeight[_nParticles]!=1)
    if(((float*)(cumWeight->data.ptr))[_nParticles]!=1)
    {
        //fprintf('rounding error?\n');
        //printf("cumWeight[_nParticles]==%15.10e\n",((float*)(cumWeight->data.ptr))[_nParticles]);
        ((float*)(cumWeight->data.ptr))[_nParticles]=1;
        if( ((float*)(cumWeight->data.ptr))[_nParticles]!=1)
        {
            //printf("still different\n");
        }
        else
        {
            //printf("now it-s ok\n");
        }
    }

    //cout<<"cumulative at position "<<_nParticles-1<<" = "<<((float*)(cumWeight->data.ptr))[_nParticles-1]<<endl;
    //cout<<"cumulative at position "<<_nParticles<<" = "<<((float*)(cumWeight->data.ptr))[_nParticles]<<endl;

    //%PERFORM THE ACTUAL RESAMPLING
    rIndex=0; //index of the randomized array
    cIndex=1; //index of the cumulative weight array. cIndex -1 indicates which particle we think of resampling.
    npIndex=0; //new particle index, tells me how many particles have been created so far.

    while(npIndex<_nParticles)
    {
        //siamo sicuri che deve essere >=? ??? !!! WARNING
        if(((float*)(cumWeight->data.ptr))[cIndex]>=(double)rIndex/(double)_nParticles+u)
        {
            //%particle cIndex-1 should be copied.
            //printf("replicating particle %d\n",cIndex-1);
            //newParticlesState(npIndex)=oldParticlesState(cIndex-1);
            ((float*)(newParticlesState->data.ptr + newParticlesState->step*0))[npIndex]=((float*)(oldParticlesState->data.ptr + oldParticlesState->step*0))[cIndex-1];
            ((float*)(newParticlesState->data.ptr + newParticlesState->step*1))[npIndex]=((float*)(oldParticlesState->data.ptr + oldParticlesState->step*1))[cIndex-1];
            ((float*)(newParticlesState->data.ptr + newParticlesState->step*2))[npIndex]=((float*)(oldParticlesState->data.ptr + oldParticlesState->step*2))[cIndex-1];
            ((float*)(newParticlesState->data.ptr + newParticlesState->step*3))[npIndex]=((float*)(oldParticlesState->data.ptr + oldParticlesState->step*3))[cIndex-1];
            ((float*)(newParticlesState->data.ptr + newParticlesState->step*4))[npIndex]=((float*)(oldParticlesState->data.ptr + oldParticlesState->step*4))[cIndex-1];
            ((float*)(newParticlesState->data.ptr + newParticlesState->step*5))[npIndex]=((float*)(oldParticlesState->data.ptr + oldParticlesState->step*5))[cIndex-1];
            rIndex=rIndex+1;
            npIndex=npIndex+1;
        }
        else
        {
            //printf("not replicating particle %d\n",cIndex-1);
            cIndex=cIndex+1;
        }
    }
    
    return false;        
}

bool PF3DTracker::systematicR(CvMat* inState, CvMat* weights, CvMat* outState)
{

    //IF THIS THING WORKS IT'S A MIRACLE.

    float N=_nParticles;
    float s=1/N;

    cvZero(_nChildren);

    //used to be: ippsVectorSlope_32u(label, _nParticles, 1,1); //should be initialized as: 1,2,3,4,5,6... fine.
    cvCopy(_ramp,_label); //label should be like: 1,2,3,4,5,6...
    //printMat(_label); this one works.

    float auxw=0;
    int li=0; //label of the current point

    //initialization
    float T;
    T=s*((float)rand()/(float)RAND_MAX); //random number between 0 and s.
    
    //cout<<"T:"<<T<<"\n";
    int j=1;
    float Q=0;
    int i=0;


    //Ipp32f u[_nParticles]; done at startup time.

    //used to be done like this: ippsRandUniform_Direct_32f(&(u[0]), _nParticles, 0, 1, &seed);
    cvRandArr( &rngState, _u, CV_RAND_UNI, cvScalar(0), cvScalar(1));
    //cout<<"u:\n";
    //printMat(_u);

    //printf("S1\n");
    //fflush(stdout);

    while((T<1)  && (i<_nParticles)) //the second part of the condition is a hack:
                                     // the loop sometimes doesn't stop without it.
    {
        if((Q>T) && (i<_nParticles))//the second part of the condition is a hack:
                                     // the loop sometimes doesn't stop without it.
        {
            T=T+s;
/*            cout<<"BRANCH 1. T= "<<T<<endl;
            fflush(stdout);*/
            //nChildren[li-1]+=1;

            ((float*)(_nChildren->data.ptr))[li-1]+=1;
        }
        else //the first time it passes by here.
        {
/*            cout<<"BRANCH 2. i: "<<i<<" j: "<<j<<" Q: "<<Q<<" T: "<<T<<endl;
            fflush(stdout);*/
            //used to be: i=(int)floor((N-j+1)*u[j-1])+j; //De Freitas uses "fix" in matlab... I guess floor should do, in this case. MAYBE NOT?
            i=(int)(floor((N-j+1)*(((float*)(_u->data.ptr))[j-1]))+j); //De Freitas uses "fix" in matlab... I guess floor should do, in this case. MAYBE NOT?

            auxw=((float*)(weights->data.ptr))[i-1];
            //used to be: auxw=weights[i-1];
            //cout<<"auxw: "<<auxw<<endl;

            //cout<<"_label:\n";
            //printMat(_label);
            //cout<<"i-1:"<<i-1<<"\n";
            li=(int)((float*)(_label->data.ptr))[i-1]; //C'E' QUALCOSA CHE NON VA QUI.
            //cout<<"li="<<li<<"\n";
            //used to be: li=label[i-1];
            Q=Q+auxw;

            ((float*)(weights->data.ptr))[i-1]=((float*)(weights->data.ptr))[j-1];
            //used to be: weights[i-1]=weights[j-1];
            ((float*)(_label->data.ptr))[i-1] = ((float*)(_label->data.ptr))[j-1];
            //used to be:label[i-1]=label[j-1];
            j=j+1;
        }
    
    }
    


    //COPY STUFF. I SHOULD COPY THE STATE OF THE PARTICLES, HERE.
    int index=1;
    for(i=0;i<N;i++)
    {
        //used to be: if(nChildren[i]>0)
        if(((float*)(_nChildren->data.ptr))[i]>0)
        {
            for(j=index;(j<index+((float*)(_nChildren->data.ptr))[i])&&(j<_nParticles+1);j++)  //WARNING: ??? !!! I MIGHT WELL HAVE MESSED SOMETHING UP HERE.
            {   
                //outIndex[j-1]=inIndex[i];
                ((float*)(outState->data.ptr + outState->step*0))[j-1]=((float*)(inState->data.ptr + inState->step*0))[i];
                ((float*)(outState->data.ptr + outState->step*1))[j-1]=((float*)(inState->data.ptr + inState->step*1))[i];
                ((float*)(outState->data.ptr + outState->step*2))[j-1]=((float*)(inState->data.ptr + inState->step*2))[i];
                ((float*)(outState->data.ptr + outState->step*3))[j-1]=((float*)(inState->data.ptr + inState->step*3))[i];
                ((float*)(outState->data.ptr + outState->step*4))[j-1]=((float*)(inState->data.ptr + inState->step*4))[i];
                ((float*)(outState->data.ptr + outState->step*5))[j-1]=((float*)(inState->data.ptr + inState->step*5))[i];

                //used to be:
                //outState[2][j-1]=inState[2][i];
                //outState[3][j-1]=inState[3][i];
                //outState[4][j-1]=inState[4][i];
                //outState[5][j-1]=inState[5][i];
            }    
            
            //cout<<"I passed by the condition\n";
        }
           
        index=index+(int)((float*)(_nChildren->data.ptr))[i];
        //cout<<"I passed by the cycle\n";
        
    }
    
    return false;        
}




bool PF3DTracker::place3dPointsPerspective(CvMat* points, float x, float y, float z)
{

    
    //*********************
    // 0. Prepare some data
    //*********************
    bool returnValue;

    float floorDistance=sqrt(x*x+y*y);      //horizontal distance from the optical center to the ball
    float distance=sqrt(x*x+y*y+z*z);       //distance from the optical center to the ball
    
    float cosAlpha=floorDistance/distance;  //cosine of an angle needed for a rotation
    float sinAlpha=-z/distance;             //sine of an angle needed for a rotation
    float cosBeta=x/floorDistance;          //cosine of an angle needed for a rotation
    float sinBeta=y/floorDistance;          //sine of an angle needed for a rotation

    
    //Rotation matrix Rz: [3 x 3]
    ((float*)(_rzMat->data.ptr + _rzMat->step*0))[0]=  cosBeta;
    ((float*)(_rzMat->data.ptr + _rzMat->step*0))[1]= -sinBeta;
    ((float*)(_rzMat->data.ptr + _rzMat->step*0))[2]=        0;
    ((float*)(_rzMat->data.ptr + _rzMat->step*1))[0]=  sinBeta;
    ((float*)(_rzMat->data.ptr + _rzMat->step*1))[1]=  cosBeta;
    ((float*)(_rzMat->data.ptr + _rzMat->step*1))[2]=        0;
    ((float*)(_rzMat->data.ptr + _rzMat->step*2))[0]=        0;
    ((float*)(_rzMat->data.ptr + _rzMat->step*2))[1]=        0;
    ((float*)(_rzMat->data.ptr + _rzMat->step*2))[2]=        1;


    //Rotation matrix Ry: [3 x 3]
    ((float*)(_ryMat->data.ptr + _ryMat->step*0))[0]=  cosAlpha;
    ((float*)(_ryMat->data.ptr + _ryMat->step*0))[1]=         0;
    ((float*)(_ryMat->data.ptr + _ryMat->step*0))[2]=  sinAlpha;
    ((float*)(_ryMat->data.ptr + _ryMat->step*1))[0]=         0;
    ((float*)(_ryMat->data.ptr + _ryMat->step*1))[1]=         1;
    ((float*)(_ryMat->data.ptr + _ryMat->step*1))[2]=         0;
    ((float*)(_ryMat->data.ptr + _ryMat->step*2))[0]= -sinAlpha;
    ((float*)(_ryMat->data.ptr + _ryMat->step*2))[1]=         0;
    ((float*)(_ryMat->data.ptr + _ryMat->step*2))[2]=  cosAlpha;
    
    


    //***********************************
    // 1. Rotate points around the Y axis
    //***********************************
    //Multiply Ry by points
    //_points2Mat=_ryMat*points     [3 x 2*nPixels]
    cvMatMul(_ryMat,points,_points2Mat);
    

    //used to be:
    /*
        returnValue=ippmMul_mm_32f(ry,      rzStride1,     Stride2,  rzColumns,      rzRows,
                                points,  pointsStride1, Stride2,  pointsColumns,  pointsRows, 
                                points2, pointsStride1, Stride2);
        if(returnValue!=0)
        {
            cout<<"something's wrong the multiplication of Ry by Points in 'place3dPoints'.\n";
            fflush(stdout);
            return true;
        }
    */
    


    //*****************************************
    // 2. Apply a vertical and horizontal shift
    //*****************************************
    //sum floorDistance to all the elements in the first row of "points2".
    cvSet(_tempMat1,cvScalar(floorDistance)); //set all elements of _tempMat1 to the value of "floorDistance"
    cvAdd(_p2Mat1,_tempMat1,_p2Mat1);         //_p2Mat1=_p2Mat1+_tempMat1.

    //used to be:
    //     ippsAddC_32f_I(floorDistance,points2,pointsColumns);
    //     if(returnValue!=0)
    //     {
    //         cout<<"something's wrong the shift in 'place3dPoints'.\n";
    //         fflush(stdout);
    //         return true;
    //     }


    //sum z to the third row of "points2".
    cvSet(_tempMat3,cvScalar(z)); //set all elements of _tempMat3 to the value of "z"
    cvAdd(_p2Mat3,_tempMat3,_p2Mat3);         //_p2Mat3=_p2Mat3+_tempMat3.

    //used to be:
    /*
    ippsAddC_32f_I(z,points2+2*pointsColumns,pointsColumns);
    if(returnValue!=0)
    {
        cout<<"something's wrong the shift in 'place3dPoints'.\n";
        fflush(stdout);
        return true;
    }
    */
    
    
    
    //**********************************
    //3. Rotate points around the Z axis
    //**********************************
    //Multiply RZ by "points2", put the result in "points"
    //points=_rzMat*_points2Mat     [3 x 2*nPixels]
    cvMatMul(_rzMat,_points2Mat,points);


    //used to be:
    /*
    returnValue=ippmMul_mm_32f(rz,      rzStride1,     Stride2,  rzColumns,      rzRows, 
                               points2,  pointsStride1, Stride2,  pointsColumns,  pointsRows, 
                               points, pointsStride1, Stride2);
    if(returnValue!=0)
    {
        cout<<"something's wrong the multiplication of Rz by Points2 in 'place3dPoints'.\n";
        fflush(stdout);
        return true;
    }
    */
    
    return false;
}


int PF3DTracker::perspective_projection(CvMat* xyz, float fx, float fy, float cx, float cy, CvMat* uv)
{
    
    int returnValue;

    //fill the projection matrix with the current values.
    ((float*)(_projectionMat->data.ptr + _projectionMat->step*0))[0]= fx;
    ((float*)(_projectionMat->data.ptr + _projectionMat->step*0))[1]=  0;
    ((float*)(_projectionMat->data.ptr + _projectionMat->step*0))[2]= cx;
    ((float*)(_projectionMat->data.ptr + _projectionMat->step*1))[0]=  0;
    ((float*)(_projectionMat->data.ptr + _projectionMat->step*1))[1]= fy;
    ((float*)(_projectionMat->data.ptr + _projectionMat->step*1))[2]= cy;

//     int a,b;
//     for(a=0;a<2;a++)
//     {
//         cout<<"LINE ";
//         for(b=0;b<3;b++)
//         {
//             cout<<((float*)(_projectionMat->data.ptr + _projectionMat->step*a))[b]<<",";
//         }
//         cout<<"\n";
//     }
//     cout<<"\n";

//     for(a=0;a<3;a++)
//     {
//         cout<<"LINE ";
//         for(b=0;b<2*nPixels;b++)
//         {
//             cout<<((float*)(xyz->data.ptr + xyz->step*a))[b]<<",";
//         }
//         cout<<"\n";
//     }
//     cout<<"\n";

    //#####################################################
    //For every column of XYZ, divide each element by Z(i).
    //#####################################################
    //setup

    cvInitMatHeader( _xyzMat1, 1, 2*nPixels, CV_32FC1, xyz->data.ptr );
    cvInitMatHeader( _xyzMat2, 1, 2*nPixels, CV_32FC1, xyz->data.ptr + xyz->step*1);
    cvInitMatHeader( _xyzMat3, 1, 2*nPixels, CV_32FC1, xyz->data.ptr + xyz->step*2);

    //divide X (the first line of xyz) by Z (the third line of xyz).
    cvDiv( _xyzMat1, _xyzMat3, _xyzMat1, 1 );

//     for(a=0;a<3;a++)
//     {
//         cout<<"LINE ";
//         for(b=0;b<2*nPixels;b++)
//         {
//             cout<<((float*)(xyz->data.ptr + xyz->step*a))[b]<<",";
//         }
//         cout<<"\n";
//     }
//     cout<<"\n";

    //divide Y (the second line of xyz) by Z (the third line of xyz).
    cvDiv( _xyzMat2, _xyzMat3, _xyzMat2, 1 );

//     for(a=0;a<3;a++)
//     {
//         cout<<"LINE ";
//         for(b=0;b<2*nPixels;b++)
//         {
//             cout<<((float*)(xyz->data.ptr + xyz->step*a))[b]<<",";
//         }
//         cout<<"\n";
//     }
//     cout<<"\n";

    //set all elements of Z to 1.
    cvSet(_xyzMat3,(cvScalar(1)));

//     for(a=0;a<3;a++)
//     {
//         cout<<"LINE ";
//         for(b=0;b<2*nPixels;b++)
//         {
//             cout<<((float*)(xyz->data.ptr + xyz->step*a))[b]<<",";
//         }
//         cout<<"\n";
//     }
//     cout<<"\n";

    //#########################
    //UV=projectionMat*(XYZ/Z).
    //#########################
    cvMatMul(_projectionMat,xyz,uv);

/*    for(a=0;a<2;a++)
    {
        cout<<"LINE ";
        for(b=0;b<2*nPixels;b++)
        {
            cout<<((float*)(_uv->data.ptr + _uv->step*a))[b]<<",";
        }
        cout<<"\n";
    }
    cout<<"\n";*/
    return 0;
    
}        






bool PF3DTracker::computeHistogram(CvMat* uv, IplImage* transformedImage,  CvMatND* innerHistogramMat, float &usedInnerPoints, CvMatND* outerHistogramMat, float &usedOuterPoints)
{
    int count;
    int u, v, a, b, c;

    usedInnerPoints=0;
    cvSetZero(innerHistogramMat);
    
    for(count=0;count<nPixels;count++)
    {
        u=(int)((float*)(uv->data.ptr + uv->step*0))[count]; //truncating ??? !!! warning
        v=(int)((float*)(uv->data.ptr + uv->step*1))[count]; //truncating ??? !!! warning
        if((v<transformedImage->height)&&(v>=0)&&(u<transformedImage->width)&&(u>=0))
        {
            a=(((uchar*)(transformedImage->imageData + transformedImage->widthStep*v))[u*3+0]);//Y bin
            b=(((uchar*)(transformedImage->imageData + transformedImage->widthStep*v))[u*3+1]);//U bin
            c=(((uchar*)(transformedImage->imageData + transformedImage->widthStep*v))[u*3+2]);//V bin
            *((float*)(innerHistogramMat->data.ptr + a*innerHistogramMat->dim[0].step + b*innerHistogramMat->dim[1].step + c*innerHistogramMat->dim[2].step)) +=1;
            usedInnerPoints+=1;
        }
    }

    if(usedInnerPoints>0)
    {
        cvConvertScale(innerHistogramMat, innerHistogramMat, 1/usedInnerPoints, 0 );
    }

    usedOuterPoints=0;
    cvSetZero(outerHistogramMat);
    for(count=nPixels;count<2*nPixels;count++)
    {
        u=(int)((float*)(uv->data.ptr + uv->step*0))[count]; //truncating ??? !!! warning
        v=(int)((float*)(uv->data.ptr + uv->step*1))[count]; //truncating ??? !!! warning
        if((v<transformedImage->height)&&(v>=0)&&(u<transformedImage->width)&&(u>=0))
        {
            a=(((uchar*)(transformedImage->imageData + transformedImage->widthStep*v))[u*3+0]);//Y bin
            b=(((uchar*)(transformedImage->imageData + transformedImage->widthStep*v))[u*3+1]);//U bin
            c=(((uchar*)(transformedImage->imageData + transformedImage->widthStep*v))[u*3+2]);//V bin
            *((float*)(outerHistogramMat->data.ptr + a*outerHistogramMat->dim[0].step + b*outerHistogramMat->dim[1].step + c*outerHistogramMat->dim[2].step)) +=1;
            usedOuterPoints+=1;
        }
    }
    if(usedOuterPoints>0)
    {
        cvConvertScale(outerHistogramMat, outerHistogramMat, 1/usedOuterPoints, 0 );
    }

    return false;
}



bool PF3DTracker::computeHistogramFromRgbImage(CvMat* uv, IplImage *image,  CvMatND* innerHistogramMat, float &usedInnerPoints, CvMatND* outerHistogramMat, float &usedOuterPoints)
{
    int count;
    int u,v;
    int r,g,b;
    int h,s,i;
    int index;

    ////////
    //INNER/
    ////////
    usedInnerPoints=0;
    cvZero(innerHistogramMat);
    //used to be: ippsZero_32f(&innerHistogram[0][0][0], YBins*UBins*VBins);
    for(count=0;count<nPixels;count++)
    {
        u=(int)((float*)(uv->data.ptr + uv->step*0))[count]; //truncating ??? !!! warning
        v=(int)((float*)(uv->data.ptr + uv->step*1))[count]; //truncating ??? !!! warning
        if((v<image->height)&&(v>=0)&&(u<image->width)&&(u>=0))
        {
            //transform the color from RGB to HSI bin.
            r=(((uchar*)(image->imageData + image->widthStep*v))[u*3+0]);
            g=(((uchar*)(image->imageData + image->widthStep*v))[u*3+1]);
            b=(((uchar*)(image->imageData + image->widthStep*v))[u*3+2]);
            index=r*65536+g*256+b;
            //increase the bin counter
            *((float*)(innerHistogramMat->data.ptr + _lut[index].y*innerHistogramMat->dim[0].step + _lut[index].u*innerHistogramMat->dim[1].step + _lut[index].v*innerHistogramMat->dim[2].step)) +=1;
            //used to be: innerHistogram[_lut[index].y][_lut[index].u][_lut[index].v]+=1; //increment the correct bin counter.
            usedInnerPoints+=1;
        }
    }

    //cout<<"inner points="<<usedInnerPoints<<endl;
    if(usedInnerPoints>0)
    {
        cvConvertScale( innerHistogramMat, innerHistogramMat, 1/usedInnerPoints, 0 );
        //used to be
        //ippsDivC_32f_I(usedInnerPoints, &innerHistogram[0][0][0], YBins*UBins*VBins);
    }

    ////////
    //OUTER/
    ////////
    usedOuterPoints=0;
    cvZero(outerHistogramMat);
    //used to be ippsZero_32f(&outerHistogram[0][0][0], YBins*UBins*VBins);
    for(count=nPixels;count<2*nPixels;count++)
    {
        u=(int)((float*)(uv->data.ptr + uv->step*0))[count]; //truncating ??? !!! warning
        v=(int)((float*)(uv->data.ptr + uv->step*1))[count]; //truncating ??? !!! warning
        if((v<image->height)&&(v>=0)&&(u<image->width)&&(u>=0))
        {
            //transform the color from RGB to HSI bin.
            r=(((uchar*)(image->imageData + image->widthStep*v))[u*3+0]);
            g=(((uchar*)(image->imageData + image->widthStep*v))[u*3+1]);
            b=(((uchar*)(image->imageData + image->widthStep*v))[u*3+2]);
            index=r*65536+g*256+b;
            //increase the bin counter
            *((float*)(outerHistogramMat->data.ptr + _lut[index].y*outerHistogramMat->dim[0].step + _lut[index].u*outerHistogramMat->dim[1].step + _lut[index].v*outerHistogramMat->dim[2].step)) +=1;
            //used to be: outerHistogram[_lut[index].y][_lut[index].u][_lut[index].v]+=1; //increment the correct bin counter.
            usedOuterPoints+=1;
        }
    }
    if(usedOuterPoints>0)
    {
        cvConvertScale( outerHistogramMat, outerHistogramMat, 1/usedOuterPoints, 0 );
        //used to be: ippsDivC_32f_I(usedOuterPoints, &outerHistogram[0][0][0], YBins*UBins*VBins);
    }

    return false;
}


bool PF3DTracker::calculateLikelihood(CvMatND* templateHistogramMat, CvMatND* innerHistogramMat, CvMatND* outerHistogramMat, float inside_outside, float &likelihood)
{

    likelihood=0;
    int a,b,c;
    for(a=0;a<YBins;a++)
        for(b=0;b<UBins;b++)
            for(c=0;c<VBins;c++)
            {
                likelihood=likelihood + sqrt( 
                *((float*)(innerHistogramMat->data.ptr + a*innerHistogramMat->dim[0].step + b*innerHistogramMat->dim[1].step + c*innerHistogramMat->dim[2].step)) *
                *((float*)(templateHistogramMat->data.ptr + a*templateHistogramMat->dim[0].step + b*templateHistogramMat->dim[1].step + c*templateHistogramMat->dim[2].step)))
                - _inside_outside_difference_weight*sqrt(
                *((float*)(outerHistogramMat->data.ptr + a*outerHistogramMat->dim[0].step + b*outerHistogramMat->dim[1].step + c*outerHistogramMat->dim[2].step)) *
                *((float*)(innerHistogramMat->data.ptr + a*innerHistogramMat->dim[0].step + b*innerHistogramMat->dim[1].step + c*innerHistogramMat->dim[2].step)));

/*              used to be
                likelihood=likelihood + sqrt(
                innerHistogram[a][b][c]*
                templateHistogram[a][b][c])
                - _inside_outside_difference_weight*sqrt(
                outerHistogram[a][b][c]
                *innerHistogram[a][b][c]);*/
             }
    likelihood=(likelihood+_inside_outside_difference_weight)/(1+_inside_outside_difference_weight);
    if(likelihood<0)
        cout<<"LIKELIHOOD<0!!!\n";
    return false;


//test Alex's model.
//    likelihood=0;
//    int a,b,c;
//    for(a=0;a<YBins;a++)
//        for(b=0;b<UBins;b++)
//            for(c=0;c<VBins;c++)
//                likelihood=likelihood + sqrt(innerHistogram[a][b][c]*templateHistogram[a][b][c]) - 
//_inside_outside_difference_weight*sqrt(outerHistogram[a][b][c]*innerHistogram[a][b][c]) -
//sqrt(outerHistogram[a][b][c]*templateHistogram[a][b][c]);
//    likelihood=(likelihood+_inside_outside_difference_weight+1)/(1+1+_inside_outside_difference_weight);
//    if(likelihood<0)
//        cout<<"LIKELIHOOD<0!!!\n";
//    return false;

}



bool PF3DTracker::testOpenCv()
{
    int type;
    bool failure;
    type=CV_32FC1; //32 bits, signed, one channel.
    CvMat* points;
    
    points = cvCreateMat( 3, 2*nPixels, type );
    readInitialmodel3dPoints(points, "models/initial_ball_points_46mm_30percent.csv");

    failure=place3dPointsPerspective(points,100,200,1000); //Funziona...

    
/*    int a,b;
    for(a=0;a<3;a++)
    {
        for(b=0;b<2*nPixels;b++)
        {
            cout<<((float*)(points->data.ptr + points->step*a))[b]<<",";
        }
        cout<<"\n";
    }*/
    

//     int a,b;
//     for(a=0;a<2;a++)
//     {
//         for(b=0;b<2*nPixels;b++)
//         {
//             cout<<((float*)(_uv->data.ptr + _uv->step*a))[b]<<",";
//         }
//         cout<<"\n";
//     }


    failure= perspective_projection(points, _perspectiveFx, _perspectiveFy, _perspectiveCx, _perspectiveCy, _uv);

    return true;

}


void printMat(CvMat* A)
{
    int a,b;
    for(a=0;a<A->rows;a++)
    {
        for(b=0;b<A->cols;b++)
        {
            cout<<((float*)(A->data.ptr + A->step*a))[b]<<",";
        }
        cout<<"\n";
    }
    
}
