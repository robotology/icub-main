// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
*
@ingroup icub_module
\defgroup icub_camExtrinsicsCalibModule camExtrinsicsCalibModule

Computes the camera pose with respect to a pre-defined chessboard pattern.

\section intro_sec Description

Uses OpenCV camera calibration functions (cvFindChessboardCorners, cvFindExtrinsicCameraParams2) 
to compute the extrinsic paramenters of the camera with respect to a calibration pattern.

The intrinsics camera parameters and description of the chessboard pattern dimensions must be given.

It outputs the translation vector and rotation angles (roll, pitch, yaw) of the camera expressed in the 
coordinate frame attached to the chessboard pattern, as the following image illustrates:

\image html OpenCV_orientation.jpg

\section lib_sec Libraries
Requires YARP and OpenCV.

\section parameters_sec Parameters

--name </portprefix>                        : sets the module name
--file <configfile.ini>                     : configuration file to use
--groupIntrinsics <intrinsicsgroupname>     : name of the section in the configuration file containing the camera's intrinsic paramenters
--groupPattern <patterngroupname>           : name of the section in the configuration file containing the dimensions of the chessboard patters

\section portsa_sec Ports Accessed

The module does not assume the existence of specific ports in the system. 
A normal usage of the module will read images from a grabber device.

\section portsc_sec Ports Created

Output ports:
- /extrinsics/img:o : streams a yarp::sig::ImageOf<PixelRGB> containing the current image with overlayed chessboard corners (when detected).

Input ports:
- /extrinsics/img:i : reads a yarp::sig::ImageOf<PixelRGB> containing the current image.


Computed extrinsic parameters are sent to the screen; not written to a port.

\section in_files_sec Input Data Files

No Input Data Files are required.

\section out_data_sec Output Data Files

No Output Data Files are created.
 
\section conf_file_sec Configuration Files

Input paramenters describing the camera and the chessboard pattern are usually passed through a txt file whose name is specified by:
--file <configfile.ini>

The file contains two groups of parameters: one for the camera intrinsics [INTRINSICS_GROUP] and for the pattern properties [PATTERN_GROUP].
The names INTRINSICS_GROUP and PATTERN_GROUP can be arbitrarily specifieb by:
--groupIntrinsics MY_INTRINSICS
--groupPattern MY_PATTERN

An example of such a configuration file is:

\code
[MY_INTRINSICS]
w  640
h  480
fx 819.634
fy 820.241
cx 350.967
cy 269.419
k1 -0.114457
k2 0.128458
p1 0.00680658
p2 0.00258502
[MY_PATTEN]
numPatternInnerCornersX     8
numPatternInnerCornersY     6
patternSquareSideLength     28
\endcode

 
The above paramenters have the following meaning.

INTRINSICS
\e h:      Input image lines. Defaults to 480.
\e w:      Input image columns. Defaults to 640.
\e fx:     Focal distance (on horizontal pixel size units). Defaults to 320.
\e fy:     Focal distance (on vertical pixel size units). Defaults to 240.
\e cx:     Image center (on horizontal pixel size units). Defaults to 320.
\e cy:     Image center (on vertical pixel size units). Defaults to 240.
\e kx:     Radial Distortion (horizontal). Defaults to 0 .
\e ky:     Radial Distortion (vertical). Defaults to 0.
\e px:     Tangential Distortion (horizontal). Defaults to 0.
\e py:     Tangential Distortion (vertical).Defaults to 0.

Default values correspond to a pinhole camera with view field of 90 deg.

PATTERN
\e numPatternInnerCornersX: Number of inner corners (where two black squares touch) in x-direction (int). Defaults to 8.
\e numPatternInnerCornersY: Number of inner corners (where two black squares touch) in y-direction (int). Defaults to 6.
\e patternSquareSideLength: Length of one side of a square in the pattern in mm (double). Defaults to 28mm.



\section tested_os_sec Tested OS

Linux and Windows.

\section example_sec Example Instantiation of the Module

A typical use of the module is the following:

\code
camextrinsicscalib.exe --file camextrinsicscalib.ini --groupPattern CALIBRATION_PATTERN --groupIntrinsics INTRINSICS
\endcode

\author Alexandre Bernardino

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/camExtrinsicsCalib/camExtrinsicsCalib.cpp.
**/

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/sig/Image.h>

//OpenCV
#include <cv.h>
#include <iostream>
#include <math.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;


class camExtrinsicsCalibModule : public Module
{
private:
    int _input_lines;
    int _input_cols;
    double _fx;            
    double _fy;           
    double _cx;
    double _cy;
    double _kx;
    double _ky;
    double _px;
    double _py;

    int _numPatternInnerCornersX; // number of inner corners of the chessboard pattern (black touches black in corner) 
    int _numPatternInnerCornersY; // 
    double _patternSquareSideLength;  

    CvPoint2D32f *_corners;
    CvMat *_imagePoints;
    CvMat *_objectPoints;
    CvMat *_intrinsicMatrix;
    CvMat *_distortionCoeffs;
    CvMat *_rotationVector;
    CvMat *_translationVector;
    CvMat *_rotationMatrix;
    CvMat *_translationVector2;
    CvMat *_rotationMatrix2;
    CvMat *_camera2normalMatrix;
    CvMat *_normal2cameraMatrix;

    IplImage *_iplimage;

	BufferedPort< ImageOf<PixelRgb> > inPort;
    BufferedPort< ImageOf<PixelRgb> > outPort;
	
public:

    camExtrinsicsCalibModule()
    {
        _iplimage = NULL;
        _corners = NULL;
        _imagePoints = NULL;
        _objectPoints = NULL;
        _intrinsicMatrix = NULL;    
        _distortionCoeffs = NULL;       
        _rotationVector = NULL;       
        _translationVector = NULL;
        _rotationMatrix = NULL;
        _translationVector2 = NULL;
        _rotationMatrix2 = NULL;
        _normal2cameraMatrix = NULL;
        _camera2normalMatrix = NULL;
    };
    ~camExtrinsicsCalibModule(){};
    
    virtual bool open(Searchable& config)
    {
        if (config.check("help","if present, display usage message")) {
            printf("Call with --name </portprefix> --file <configfile.ini> --groupIntrinsics <intrinsicsgroupname> --groupPattern <patterngroupname>\n");
            return false;
        }

       // check --groupIntrinsics options (required)
        Bottle botIntrinsics(config.toString().c_str());
        botIntrinsics.setMonitor(config.getMonitor());
        // is group option present?
        Value *valGroup; // check assigns pointer to reference (no delete required (?))
        if(config.check("groupIntrinsics", valGroup, "Name of the configuration group with camera intrinsics (string).")){
            string strGroup = valGroup->asString().c_str();
            // is group a valid bottle?
            if (!config.findGroup(strGroup.c_str()).isNull()){
                botIntrinsics.clear();
                botIntrinsics.fromString(config.findGroup(strGroup.c_str()).toString().c_str());
            }
            else{
                cout << endl << "Group " << strGroup.c_str() << " not found." << endl;
                return false;
            }
        }
         else{
            cout << "Please specify a valid configuration group containing camera calibration parameters the input image (--groupIntrinsics)" << endl;
            return false;
        }

        // check --groupPattern options (required)
        Bottle botPattern;
        botPattern.setMonitor(config.getMonitor());
        // is group option present?
        if(config.check("groupPattern", valGroup, "Name of the configuration group with pattern parameters (string).")){
            string strGroup = valGroup->asString().c_str();
            // is group a valid bottle?
            if (!config.findGroup(strGroup.c_str()).isNull()){
                botPattern.clear();
                botPattern.fromString(config.findGroup(strGroup.c_str()).toString().c_str());
            }
            else{
                cout << endl << "Group " << strGroup.c_str() << " not found." << endl;
                return false;
            }
        }
        else{
            cout << "Please specify a valid configuration group containing calibration pattern parameters (--groupPattern)" << endl;
            return false;
        }

        // Defaults will correspond to a view field of 90 deg.
        _input_lines = botIntrinsics.check("h", 480, "Input image lines").asInt();
        _input_cols = botIntrinsics.check("w", 640, "Input image columns").asInt();
        _fx = botIntrinsics.check("fx", 320, "Focal distance (on horizontal pixel size units)").asDouble();
        _fy = botIntrinsics.check("fy", 240, "Focal distance (on vertical pixel size units)").asDouble();
        _cx = botIntrinsics.check("cx", 320, "Image center (on horizontal pixel size units)").asDouble();
        _cy = botIntrinsics.check("cy", 240, "Image center (on vertical pixel size units)").asDouble();
        _kx = botIntrinsics.check("kx", 0, "Radial Distortion (horizontal)").asDouble();
        _ky = botIntrinsics.check("ky", 0, "Radial Distortion (vertical)").asDouble();
        _px = botIntrinsics.check("px", 0, "Tangential Distortion (horizontal)").asDouble();
        _py = botIntrinsics.check("py", 0, "Tangential Distortion (vertical)").asDouble();

        _numPatternInnerCornersX = botPattern.check("numPatternInnerCornersX", Value(8), "Number of inner corners (where two black squares touch) in x-direction (int)").asInt();
        _numPatternInnerCornersY = botPattern.check("numPatternInnerCornersY", Value(6),"Number of inner corners (where two black squares touch) in y-direction (int)").asInt();
        _patternSquareSideLength = botPattern.check("patternSquareSideLength", Value(28.0), "Length of one side of a square in the pattern in mm (double)").asDouble();


        _corners = new CvPoint2D32f[_numPatternInnerCornersX *_numPatternInnerCornersY];

        _imagePoints = cvCreateMat( _numPatternInnerCornersX * _numPatternInnerCornersY, 2, CV_32F);

        _objectPoints = cvCreateMat( _numPatternInnerCornersX * _numPatternInnerCornersY, 3, CV_32F);

        
        // size of the pattern (2D) in 3D world measurements (mm)
        int pos = 0;
        for (int y = 0; y < _numPatternInnerCornersY; y++){
            for (int x = 0; x < _numPatternInnerCornersX; x++){
                CV_MAT_ELEM( *_objectPoints, float, pos+x, 0) = (float)(x * _patternSquareSideLength + _patternSquareSideLength);
                CV_MAT_ELEM( *_objectPoints, float, pos+x, 1) = (float)(y * _patternSquareSideLength + _patternSquareSideLength);
                CV_MAT_ELEM( *_objectPoints, float, pos+x, 2) = 0.0f;
            }
            pos += _numPatternInnerCornersX;
        }

       
        _intrinsicMatrix = cvCreateMat( 3, 3, CV_32F);   
        CV_MAT_ELEM( *_intrinsicMatrix, float, 0, 0) = (float)_fx;
        CV_MAT_ELEM( *_intrinsicMatrix, float, 0, 1) = 0.0f;
        CV_MAT_ELEM( *_intrinsicMatrix, float, 0, 2) = (float)_cx;
        CV_MAT_ELEM( *_intrinsicMatrix, float, 1, 0) = 0.0f;
        CV_MAT_ELEM( *_intrinsicMatrix, float, 1, 1) = (float)_fy;
        CV_MAT_ELEM( *_intrinsicMatrix, float, 1, 2) = (float)_cy;
        CV_MAT_ELEM( *_intrinsicMatrix, float, 2, 0) = 0.0f;
        CV_MAT_ELEM( *_intrinsicMatrix, float, 2, 1) = 0.0f;
        CV_MAT_ELEM( *_intrinsicMatrix, float, 2, 2) = 1.0f;

        
        
        _distortionCoeffs = cvCreateMat( 1, 4, CV_32F);
        CV_MAT_ELEM( *_distortionCoeffs, float, 0, 0) = (float)_kx;
        CV_MAT_ELEM( *_distortionCoeffs, float, 0, 1) = (float)_ky;
        CV_MAT_ELEM( *_distortionCoeffs, float, 0, 2) = (float)_px;
        CV_MAT_ELEM( *_distortionCoeffs, float, 0, 3) = (float)_py;

        _normal2cameraMatrix = cvCreateMat( 3, 3, CV_32F);   
        CV_MAT_ELEM( *_normal2cameraMatrix, float, 0, 0) = 0.0f;
        CV_MAT_ELEM( *_normal2cameraMatrix, float, 0, 1) = 0.0f;
        CV_MAT_ELEM( *_normal2cameraMatrix, float, 0, 2) = 1.0f;
        CV_MAT_ELEM( *_normal2cameraMatrix, float, 1, 0) = -1.0f;
        CV_MAT_ELEM( *_normal2cameraMatrix, float, 1, 1) = 0.0f;
        CV_MAT_ELEM( *_normal2cameraMatrix, float, 1, 2) = 0.0f;
        CV_MAT_ELEM( *_normal2cameraMatrix, float, 2, 0) = 0.0f;
        CV_MAT_ELEM( *_normal2cameraMatrix, float, 2, 1) = -1.0f;
        CV_MAT_ELEM( *_normal2cameraMatrix, float, 2, 2) = 0.0f;

        _camera2normalMatrix = cvCreateMat( 3, 3, CV_32F);   
        cvTranspose(_normal2cameraMatrix, _camera2normalMatrix);


        // output data structures
        _rotationVector = cvCreateMat( 3, 1, CV_32F);
        _translationVector = cvCreateMat( 3, 1, CV_32F);
        _rotationMatrix = cvCreateMat(3,3,CV_32F);
        _translationVector2 = cvCreateMat( 3, 1, CV_32F);
        _rotationMatrix2 = cvCreateMat(3,3,CV_32F);


        inPort.open(getName("img:i"));
        outPort.open(getName("img:o"));
  
	    _iplimage = cvCreateImage( cvSize(_input_cols,_input_lines), 8, 3 );
        
        return true;
    };

    virtual bool close()
    {
        inPort.close();
        outPort.close();
       
        cvReleaseImage(&_iplimage);
        if (_corners != NULL)
            delete [] _corners;
        _corners = NULL;
        if (_imagePoints != NULL)
            cvReleaseMat(&_imagePoints);
        _imagePoints = NULL;
        if (_objectPoints != NULL)
            cvReleaseMat(&_objectPoints);
        _objectPoints = NULL;
        if (_intrinsicMatrix != NULL)
            cvReleaseMat(&_intrinsicMatrix);
        _intrinsicMatrix = NULL;
        if (_distortionCoeffs != NULL)
            cvReleaseMat(&_distortionCoeffs);
        _distortionCoeffs = NULL;
        if (_rotationVector != NULL)
            cvReleaseMat(&_rotationVector);
        _rotationVector = NULL;
        if (_translationVector != NULL)
            cvReleaseMat(&_translationVector);
        _translationVector = NULL;
        if (_rotationMatrix != NULL)
            cvReleaseMat(&_rotationMatrix);
        _rotationMatrix = NULL;
        if (_translationVector2 != NULL)
            cvReleaseMat(&_translationVector2);
        _translationVector2 = NULL;
        if (_rotationMatrix2 != NULL)
            cvReleaseMat(&_rotationMatrix2);
        _rotationMatrix2 = NULL;
        if (_normal2cameraMatrix != NULL)
            cvReleaseMat(&_normal2cameraMatrix);
        _normal2cameraMatrix = NULL;
        if (_camera2normalMatrix != NULL)
            cvReleaseMat(&_camera2normalMatrix);
        _camera2normalMatrix = NULL;

       return true;
    };
 
    virtual bool interruptModule()
    {
       inPort.interrupt();
       outPort.interrupt();
       return true;
    };

    virtual bool updateModule()
    {
        yarp::sig::ImageOf<PixelRgb> *yrpImgIn;
		yrpImgIn = inPort.read(true);						//read image
        if( yrpImgIn == NULL)  // this is the case if module is requested to quit while waiting for image
            return true;

        // convert to BGR color order for opencv (in any case)
        cvCvtColor(yrpImgIn->getIplImage(), _iplimage, CV_RGB2BGR);

        int found = 0;
        int numCorners = -1;
        
        // find pattern
        found = cvFindChessboardCorners(_iplimage, 
                                        cvSize(_numPatternInnerCornersX,_numPatternInnerCornersY), 
                                        _corners, 
                                        &numCorners,
                                        CV_CALIB_CB_NORMALIZE_IMAGE+CV_CALIB_CB_ADAPTIVE_THRESH);

        // draw pattern
        if (found)
        {
            cvDrawChessboardCorners( _iplimage, 
                                    cvSize(_numPatternInnerCornersX,_numPatternInnerCornersY),
                                    _corners, 
                                    numCorners,
                                    found );
            // copy corners to imagePoints matrix
            for (int i = 0; i < numCorners; i++){
                CV_MAT_ELEM( *_imagePoints, float, i, 0) = _corners[i].x;
                CV_MAT_ELEM( *_imagePoints, float, i, 1) = _corners[i].y;
            }
            
            //finds the pose of the calibration rig in the camera reference frame
            cvFindExtrinsicCameraParams2( _objectPoints,
                                          _imagePoints,
                                          _intrinsicMatrix,
                                          _distortionCoeffs,
                                          _rotationVector,
                                          _translationVector );

            cvRodrigues2( _rotationVector, _rotationMatrix );

            //compute the pose of the camera in the world frame
            cvTranspose(_rotationMatrix, _rotationMatrix2);
            cvMatMul(_rotationMatrix2, _translationVector, _translationVector2);

            //rotate axes such as:
            // X - front
            // Y - left
            // Z - up
            cvMatMul(_normal2cameraMatrix, _rotationMatrix2, _rotationMatrix);
            cvMatMul(_normal2cameraMatrix, _translationVector2, _translationVector);
            cvMatMul(_rotationMatrix, _camera2normalMatrix, _rotationMatrix2);
            
            //computation of the Roll-Pitch-Yaw angles
            
            double roll, pitch, yaw, temp1, temp2;

            if (    (CV_MAT_ELEM( *_rotationMatrix2, float, 0, 0) == 0) &&
                    (CV_MAT_ELEM( *_rotationMatrix2, float, 1, 0) == 0) )
            {
                yaw = 0;
                pitch = 90;
                roll = atan2(CV_MAT_ELEM( *_rotationMatrix2, float, 0, 1), 
                             CV_MAT_ELEM( *_rotationMatrix2, float, 1, 1));
            }
            else
            {
                temp1 = CV_MAT_ELEM( *_rotationMatrix2, float, 0, 0)*CV_MAT_ELEM( *_rotationMatrix2, float, 0, 0);
                temp2 = CV_MAT_ELEM( *_rotationMatrix2, float, 1, 0)*CV_MAT_ELEM( *_rotationMatrix2, float, 1, 0);
                pitch = atan2(-(double)CV_MAT_ELEM( *_rotationMatrix2, float, 2, 0), sqrt(temp1+temp2));
                yaw = atan2((double)CV_MAT_ELEM( *_rotationMatrix2, float, 1, 0),(double)CV_MAT_ELEM( *_rotationMatrix2, float, 0, 0));
                roll = atan2((double)CV_MAT_ELEM( *_rotationMatrix2, float, 2, 1),(double)CV_MAT_ELEM( *_rotationMatrix2, float, 2, 2));
            }

            cout << "ATTITUDE:" << endl;
            cout << "YAW " << yaw << " ";
            cout << "PITCH " << pitch << " ";
            cout << "ROLL " << roll << " ";
            cout << endl;
            cout << "POSITION:" << endl;
            cout << "X " << CV_MAT_ELEM( *_translationVector, float, 0, 0) << " ";
            cout << "Y " << CV_MAT_ELEM( *_translationVector, float, 1, 0) << " ";
            cout << "Z " << CV_MAT_ELEM( *_translationVector, float, 2, 0) << " ";
            cout << endl;
            
        }
		
        // convert image back to RGB image for yarp
        
        yarp::sig::ImageOf<PixelRgb>& yrpImgOut = outPort.prepare();
        yrpImgOut.resize(_input_cols, _input_lines);
        cvCvtColor(_iplimage, yrpImgOut.getIplImage(), CV_BGR2RGB);
        outPort.write();

		
        return true;
    };
};

int main(int argc, char *argv[]) {
    Network yarp;
    yarp::os::Time::turboBoost();

    camExtrinsicsCalibModule module;
    module.setName("/extrinsics"); // set default name of module
    return module.runModule(argc,argv);
}
