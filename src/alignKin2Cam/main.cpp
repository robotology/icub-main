/** 
\defgroup alignKin2Cam alignKin2Cam
 
@ingroup icub_module  
 
Find the two homogeneous transformations which align the 
kinematic eyes to the image planes [UNDER DEV.]

Copyright (C) 2008 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description 
 
Since the kinematic Z axis of the eye is not perfectly in line 
with the real optical axis of the camera (which is orthogonal to 
the image plane), an automatic procedure has been put in place 
in order to find the homegeneous transformations capable of 
compensating for this misalignment. Indeed, reducing this offset 
may greatly improve the localization of a point in the 3D space 
relying on a stereo-vision system coupled with an accurate 
kinematic knowledge of cameras positions. 
 
The calibration consists of two consecutive stages: 
 
- When launched, the module moves the robot head in front of the 
  calibration rig (which has to be in sight - use the output
  ports to check) following a pre-assigned path. Data from
  encoders together with the position of the rig with respect to
  the cameras frames for both eyes are stored in a file. See
  \ref camExtrinsicsCalibModule "camExtrinsicsCalibModule" for
       more info.
  The output of first stage is a data file (<outFile1>).
 
- In the second stage, user has to launch a MATLAB function 
  (located in the \e scripts directory) over the collected data.
  The algorithm runs the \e interior-point minimization in order
  to reduce the spread of the observed rig's positions for both
  eyes and to make them to overlap as much as possible. By doing
  that, two final rototranslation matrices Hxl and Hxr are
  determined, one per eye, which have to be added to the
  kinematic structure.
 
  Let Hl{i} and Hr{i} be the kinematic transformations from the
  left and right eye to the root reference for the acquired
  sample i in <outFile1>, and Hle{i} and Hre{i} be the
  corresponding transformations from the rig's reference to the
  eyes references.
 
  The task to be minimized is:
 
    min on (Hxl,Hxr) of f=mean([dist(xl{i},xm); dist(xr{i},xm)])
 
  where:
 
  - xl{i}=Hl{i}*Hlx*Hle{i}*[0 0 0 1]' is the rig's position of
    the acquired sample i (line i in the file) in the root
    reference passing by the left eye.
 
  - xr{i}=Hr{i}*Hrx*Hre{i}*[0 0 0 1]'is the rig's position in
    the root reference passing by the right eye.
 
  - xm=mean([xl{i}; xr{i}]) is the center of mass of all the
    samples.

  - dist(.,.) computes the euclidean norm in the 3D space.
 
  - mean([.; .]) computes the mean over the complete set.
 
  The second stage is accomplished by simply typing \e
  runMin(<outFile1>,<outFile2>) from within MATLAB. As result,
  Hxl and Hxr will be stored in <outFile2> as described in the
  following sections.
 
A plot is also produced showing the uncalibrated and calibrated 
points along with the new computed z kinematic axes aligned to 
the optical axes through the transformations Hxl and Hxr.
\image html alignKin2Cam.jpg 
 
\section lib_sec Libraries 
- YARP libraries. 
- OpenCV libraries. 
- MATLAB R2008b installed (Optimization Toolbox shall be 
  available).

\section parameters_sec Parameters 
--name </portprefix> 
- sets the module name
 
--config <configfile.ini> 
- see Configuration Files section 
 
\section portsa_sec Ports Accessed
 
Direct access to motor and camera ports of the robot without the
user intervention. 

\section portsc_sec Ports Created 
 
- <modulename>/<part>/img:o : streams a 
  yarp::sig::ImageOf<PixelRGB> containing the current image of
  the <part> eye with overlayed chessboard corners (when
  detected). The tag <part> can be left or right.
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
 
During the acquiring stage data are collected from the robot and
saved in a file whose name is specified through the option \e 
outputFile of the configuration input file (see below). 
The data format is as follows, row by row: 
 
\code 
encs + leftR + leftT + rightR + rightT; 
\endcode 
 
where: 
\code 
encs   = 9 double of torso+head positions [rad]
leftR  = 3 double of left camera rotational vector [rad]
leftT  = 3 double of left camera translational vector [m]
rightR = 3 double of right camera rotational vector [rad] 
rightT = 3 double of right camera translational vector [m]
\endcode 
 
The couple [leftR,leftT] describes the rototranslation of the
observed rig's position with respect to the left camera frame. 
The same holds for the couple [rightR,rightT] wrt right eye.
 
Once data have been acquired and saved in <outFile1>, user can 
call the MATLAB function \e runMin(<outFile1>,<outFile2>) from 
within the directory \e scripts in order to obtain in <outFile2> 
the final description of homogeneous transformations which align 
the kinematic eye with the image planes. The default name for 
<outFile2> is 'alignKin2Cam.ini' and should be put in the 
robot-dependent configuration directory, i.e. under 
$ICUB_DIR/app/<robotname>/conf.
 
Here's how the file <outFile2> will look like: 
\code 
[LEFT]
R      0.0324596 -0.0329982 0.0229948               // [rad]
T      8.9604e-009 1.51339e-007 9.28864e-007        // [m]
length 4.15777e-007 -4.44085e-007                   // [m]
offset 1.11013e-006 -1.84301e-007                   // [m]
twist  0.163382 -0.12788                            // [rad]
joint  -0.181402 0.20654                            // [rad]

[RIGHT]
R      0.0872497 -0.0872643 -0.0546534              // [rad]
T      -4.21262e-007 4.47732e-008 -9.24493e-007     // [m]
length 0.000100176 -0.000105106                     // [m]
offset 0.000103259 -9.96225e-005                    // [m]
twist  0.296114 -0.194269                           // [rad]
joint  -0.34062 0.294211                            // [rad]
\endcode 
 
\note The vector \e R represents the rotation given in axis 
      form.
 
\note A pair of virtual D-H links is also returned for each eye. 
      Call T1 and T2 the two rototranslation matrices associated
      to the D-H links, it follows that T1*T2=[R,T].

\section conf_file_sec Configuration Files 
An example of configuration file is as follows: 
\code 
[GENERAL]
outputFile data.log     // the name of file where all 
                        // information taken from the
                        // robot are saved

[LEFT]                  // intrinsic left camera's params
w  320
h  240
fx 219.567
fy 219.407
cx 151.175
cy 120.975
k1 -0.377126
k2 0.137344
p1 -0.000203182
p2 -0.000525078

[RIGHT]                 // intrinsic right camera's params 
w 320 
h 240 
fx 218.985 
fy 219.076 
cx 152.319 
cy 120.238 
k1 0.370301 
k2 0.133322 
p1 0.000470798 
p2 0.000218484 

[PATTERN]               // rig's info 
numPatternInnerCornersX 9 
numPatternInnerCornersY 6 
patternSquareSideLength 32
\endcode 
 
\section tested_os_sec Tested OS
Windows, Linux
 
\author Ugo Pattacini

This file can be edited at 
\in src/alignKin2Cam/main.cpp. 
*/ 

#include <gsl/gsl_math.h>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <cv.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;


#define EYETILT_LO         -20.0
#define EYETILT_UP          00.0
#define EYEPAN_LO          -15.0
#define EYEPAN_UP          +15.0
#define EYEVER_LO          +03.0
#define EYEVER_UP          +10.0
#define STARTING_VERGENCE   08.0
#define VELOCITY            10.0
#define TRAJPOINTS          15

class alignKin2Cam : public RFModule
{
private:
    int _input_lines;
    int _input_cols;
    double _fxL,_fxR;            
    double _fyL,_fyR;           
    double _cxL,_cxR;
    double _cyL,_cyR;
    double _kxL,_kxR;
    double _kyL,_kyR;
    double _pxL,_pxR;
    double _pyL,_pyR;

    int _numPatternInnerCornersX;
    int _numPatternInnerCornersY;
    double _patternSquareSideLength;  

    CvPoint2D32f *_corners;
    CvMat *_imagePoints;
    CvMat *_objectPoints;
    CvMat *_intrinsicMatrixL;
    CvMat *_intrinsicMatrixR;
    CvMat *_distortionCoeffsL;
    CvMat *_distortionCoeffsR;
    CvMat *_rotationVector;
    CvMat *_translationVector;

    IplImage *_iplimage;

	BufferedPort<ImageOf<PixelRgb> > inPortLeft;
    BufferedPort<ImageOf<PixelRgb> > inPortRight;
    BufferedPort<ImageOf<PixelRgb> > outPortLeft;
    BufferedPort<ImageOf<PixelRgb> > outPortRight;

    PolyDriver       *drvTorso;
    PolyDriver       *drvHead;
    IPositionControl *posTorso;
    IPositionControl *posHead;
    IEncoders        *encsTorso;
    IEncoders        *encsHead;

    Matrix traj;
    int    cnt;

    string   outputFile;
    ofstream fout;
	
public:

    alignKin2Cam()
    {
        _iplimage=NULL;
        _corners=NULL;
        _imagePoints=NULL;
        _objectPoints=NULL;
        _intrinsicMatrixL=NULL;
        _intrinsicMatrixR=NULL;
        _distortionCoeffsL=NULL;
        _distortionCoeffsR=NULL;
        _rotationVector=NULL;       
        _translationVector=NULL;
    };
    
    virtual bool configure(ResourceFinder &rf)
    {      
        Property optTorso(""), optHead("");

        optTorso.put("device","remote_controlboard");
        optTorso.put("remote","/icub/torso");
        optTorso.put("local",getName("torso"));

        optHead.put("device","remote_controlboard");
        optHead.put("remote","/icub/head");
        optHead.put("local",getName("head"));

        drvTorso=new PolyDriver(optTorso);
        drvHead =new PolyDriver(optHead);

        if (!drvTorso->isValid() || !drvHead->isValid())
            return false;

        Property options(rf.toString());
        string configFile;

        if (options.check("config"))
            configFile=options.find("config").asString();
        else
            configFile.clear();

        Property parameters;
        parameters.fromConfigFile(configFile.c_str());
        Bottle parGeneral=parameters.findGroup("GENERAL");
        Bottle parLeft=parameters.findGroup("LEFT");
        Bottle parRight=parameters.findGroup("RIGHT");
        Bottle parPattern=parameters.findGroup("PATTERN");

        if (!parGeneral.size() || !parLeft.size() ||
            !parRight.size()   || !parPattern.size())
        {
            cerr << "Unable to load all the required params!" << endl;
            return false;
        }

        _input_lines=parLeft.find("h").asInt();
        _input_cols=parLeft.find("w").asInt();

        _fxL=parLeft.find("fx").asDouble();
        _fyL=parLeft.find("fy").asDouble();
        _cxL=parLeft.find("cx").asDouble();
        _cyL=parLeft.find("cy").asDouble();
        _kxL=parLeft.find("kx").asDouble();
        _kyL=parLeft.find("ky").asDouble();
        _pxL=parLeft.find("px").asDouble();
        _pyL=parLeft.find("py").asDouble();

        _fxR=parRight.find("fx").asDouble();
        _fyR=parRight.find("fy").asDouble();
        _cxR=parRight.find("cx").asDouble();
        _cyR=parRight.find("cy").asDouble();
        _kxR=parRight.find("kx").asDouble();
        _kyR=parRight.find("ky").asDouble();
        _pxR=parRight.find("px").asDouble();
        _pyR=parRight.find("py").asDouble();

        _numPatternInnerCornersX=parPattern.find("numPatternInnerCornersX").asInt();
        _numPatternInnerCornersY=parPattern.find("numPatternInnerCornersY").asInt();
        _patternSquareSideLength=parPattern.find("patternSquareSideLength").asDouble();        

        outputFile=parGeneral.find("outputFile").asString();

        _corners=new CvPoint2D32f[_numPatternInnerCornersX*_numPatternInnerCornersY];
        _imagePoints=cvCreateMat(_numPatternInnerCornersX*_numPatternInnerCornersY,2,CV_32F);
        _objectPoints=cvCreateMat(_numPatternInnerCornersX*_numPatternInnerCornersY,3,CV_32F);        
        
        int pos=0;
        for (int y=0; y<_numPatternInnerCornersY; y++)
        {
            for (int x=0; x<_numPatternInnerCornersX; x++)
            {
                CV_MAT_ELEM(*_objectPoints,float,pos+x,0)=(float)(x*_patternSquareSideLength+_patternSquareSideLength);
                CV_MAT_ELEM(*_objectPoints,float,pos+x,1)=(float)(y*_patternSquareSideLength+_patternSquareSideLength);
                CV_MAT_ELEM(*_objectPoints,float,pos+x,2)=0.0f;
            }

            pos+=_numPatternInnerCornersX;
        }
       
        _intrinsicMatrixL=cvCreateMat(3,3,CV_32F);   
        CV_MAT_ELEM(*_intrinsicMatrixL,float,0,0)=(float)_fxL;
        CV_MAT_ELEM(*_intrinsicMatrixL,float,0,1)=0.0f;
        CV_MAT_ELEM(*_intrinsicMatrixL,float,0,2)=(float)_cxL;
        CV_MAT_ELEM(*_intrinsicMatrixL,float,1,0)=0.0f;
        CV_MAT_ELEM(*_intrinsicMatrixL,float,1,1)=(float)_fyL;
        CV_MAT_ELEM(*_intrinsicMatrixL,float,1,2)=(float)_cyL;
        CV_MAT_ELEM(*_intrinsicMatrixL,float,2,0)=0.0f;
        CV_MAT_ELEM(*_intrinsicMatrixL,float,2,1)=0.0f;
        CV_MAT_ELEM(*_intrinsicMatrixL,float,2,2)=1.0f;

        _intrinsicMatrixR=cvCreateMat(3,3,CV_32F);   
        CV_MAT_ELEM(*_intrinsicMatrixR,float,0,0)=(float)_fxR;
        CV_MAT_ELEM(*_intrinsicMatrixR,float,0,1)=0.0f;
        CV_MAT_ELEM(*_intrinsicMatrixR,float,0,2)=(float)_cxR;
        CV_MAT_ELEM(*_intrinsicMatrixR,float,1,0)=0.0f;
        CV_MAT_ELEM(*_intrinsicMatrixR,float,1,1)=(float)_fyR;
        CV_MAT_ELEM(*_intrinsicMatrixR,float,1,2)=(float)_cyR;
        CV_MAT_ELEM(*_intrinsicMatrixR,float,2,0)=0.0f;
        CV_MAT_ELEM(*_intrinsicMatrixR,float,2,1)=0.0f;
        CV_MAT_ELEM(*_intrinsicMatrixR,float,2,2)=1.0f;        
        
        _distortionCoeffsL=cvCreateMat(1,4,CV_32F);
        CV_MAT_ELEM(*_distortionCoeffsL,float,0,0)=(float)_kxL;
        CV_MAT_ELEM(*_distortionCoeffsL,float,0,1)=(float)_kyL;
        CV_MAT_ELEM(*_distortionCoeffsL,float,0,2)=(float)_pxL;
        CV_MAT_ELEM(*_distortionCoeffsL,float,0,3)=(float)_pyL;

        _distortionCoeffsR=cvCreateMat(1,4,CV_32F);
        CV_MAT_ELEM(*_distortionCoeffsR,float,0,0)=(float)_kxR;
        CV_MAT_ELEM(*_distortionCoeffsR,float,0,1)=(float)_kyR;
        CV_MAT_ELEM(*_distortionCoeffsR,float,0,2)=(float)_pxR;
        CV_MAT_ELEM(*_distortionCoeffsR,float,0,3)=(float)_pyR;

        _rotationVector=cvCreateMat(3,1,CV_32F);
        _translationVector=cvCreateMat(3,1,CV_32F);

        inPortLeft.open(getName("left/img:i"));
        outPortLeft.open(getName("left/img:o"));
  
        inPortRight.open(getName("right/img:i"));
        outPortRight.open(getName("right/img:o"));

        Network::connect("/icub/cam/left",inPortLeft.getName());
        Network::connect("/icub/cam/right",inPortRight.getName());

        fout.open(outputFile.c_str());

        _iplimage=cvCreateImage(cvSize(_input_cols,_input_lines),8,3);

        // go to home position
        drvTorso->view(posTorso);
        drvHead->view(posHead);
        drvTorso->view(encsTorso);
        drvHead->view(encsHead);

        Vector v(6); v=VELOCITY;
        Vector a(6); a=1e6;
        posHead->setRefSpeeds(v.data());
        posHead->setRefAccelerations(a.data());

        Vector q(6); q=0.0;
        posTorso->positionMove(q.data());
        q(5)=STARTING_VERGENCE;
        posHead->positionMove(q.data());

        bool doneTorso=false;
        bool doneHead=false;

        while (doneTorso && doneHead)
        {
            posTorso->checkMotionDone(&doneTorso);
            posHead->checkMotionDone(&doneHead);
        }        

        // compute trajectory points
        traj.resize(TRAJPOINTS,6);
        unsigned int i=0;
        for (; i<TRAJPOINTS/3; i++)
        {
            traj(i,0)=-15.0;
            traj(i,1)=0.0;
            traj(i,2)=0.0;
            traj(i,3)=0.0;
            traj(i,4)=EYEPAN_LO+(EYEPAN_UP-EYEPAN_LO)*((float)i/float(TRAJPOINTS/3));
            traj(i,5)=STARTING_VERGENCE;
        }

        for (; i<2*TRAJPOINTS/3; i++)
        {
            traj(i,0)=-15.0;
            traj(i,1)=0.0;
            traj(i,2)=0.0;
            traj(i,3)=EYETILT_LO+(EYETILT_UP-EYETILT_LO)*((float)(i-TRAJPOINTS/3)/float(TRAJPOINTS/3));
            traj(i,4)=0.0;
            traj(i,5)=STARTING_VERGENCE;
        }

        for (; i<TRAJPOINTS; i++)
        {
            traj(i,0)=-15.0;
            traj(i,1)=0.0;
            traj(i,2)=0.0;
            traj(i,3)=0.0;
            traj(i,4)=0.0;
            traj(i,5)=EYEVER_LO+(EYEVER_UP-EYEVER_LO)*((float)(i-2*TRAJPOINTS/3)/float(TRAJPOINTS/3));
        }

        cnt=0;        

        return true;
    };

    virtual bool close()
    {
        inPortLeft.close();
        inPortRight.close();
        outPortLeft.close();
        outPortRight.close();
       
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
        if (_intrinsicMatrixL != NULL)
            cvReleaseMat(&_intrinsicMatrixL);
        _intrinsicMatrixR = NULL;
        if (_intrinsicMatrixR != NULL)
            cvReleaseMat(&_intrinsicMatrixR);
        if (_distortionCoeffsL != NULL)
            cvReleaseMat(&_distortionCoeffsL);
        _distortionCoeffsR = NULL;
        if (_distortionCoeffsR != NULL)
            cvReleaseMat(&_distortionCoeffsR);
        if (_rotationVector != NULL)
            cvReleaseMat(&_rotationVector);
        _rotationVector = NULL;
        if (_translationVector != NULL)
            cvReleaseMat(&_translationVector);
        _translationVector = NULL;

        delete drvTorso;
        delete drvHead;

        fout.close();

        return true;
    };
 
    virtual bool interruptModule()
    {
       inPortLeft.interrupt();
       inPortRight.interrupt();
       outPortLeft.interrupt();
       outPortRight.interrupt();
       return true;
    };

    int processImages(const string &type, Vector &data)
    {
        BufferedPort<ImageOf<PixelRgb> > *inPort;
        BufferedPort<ImageOf<PixelRgb> > *outPort;
        CvMat *_intrinsicMatrix;
        CvMat *_distortionCoeffs;

        if (type=="left")
        {
            inPort=&inPortLeft;
            outPort=&outPortLeft;
            _intrinsicMatrix=_intrinsicMatrixL;
            _distortionCoeffs=_distortionCoeffsL;
        }
        else
        {
            inPort=&inPortRight;
            outPort=&outPortRight;
            _intrinsicMatrix=_intrinsicMatrixR;
            _distortionCoeffs=_distortionCoeffsR;
        }

        ImageOf<PixelRgb> *yrpImgIn=inPort->read(true);
        if (yrpImgIn==NULL)
            return 0;

        // convert to BGR color order for opencv (in any case)
        cvCvtColor(yrpImgIn->getIplImage(),_iplimage,CV_RGB2BGR);

        int found=0;
        int numCorners=-1;

        // find pattern
        found = cvFindChessboardCorners(_iplimage, 
                                        cvSize(_numPatternInnerCornersX,_numPatternInnerCornersY), 
                                        _corners, 
                                        &numCorners,
                                        CV_CALIB_CB_NORMALIZE_IMAGE+CV_CALIB_CB_ADAPTIVE_THRESH);

        // draw pattern
        if (found)
        {
            cvDrawChessboardCorners(_iplimage, 
                                    cvSize(_numPatternInnerCornersX,_numPatternInnerCornersY),
                                    _corners, 
                                    numCorners,
                                    found);

            // copy corners to imagePoints matrix
            for (int i=0; i<numCorners; i++)
            {
                CV_MAT_ELEM(*_imagePoints,float,i,0)=_corners[i].x;
                CV_MAT_ELEM(*_imagePoints,float,i,1)=_corners[i].y;
            }

            // find the pose of the calibration rig in the camera reference frame
            cvFindExtrinsicCameraParams2(_objectPoints,
                                          _imagePoints,
                                          _intrinsicMatrix,
                                          _distortionCoeffs,
                                          _rotationVector,
                                          _translationVector);

            data.resize(6);
            unsigned int i=0;
            for (; i<3; i++)
                data[i]=CV_MAT_ELEM(*_rotationVector,float,i,0);    // in radians

            for (; i<6; i++)
                data[i]=0.001*CV_MAT_ELEM(*_translationVector,float,i-3,0); // in meters
        }

        // convert image back to RGB image for yarp
        ImageOf<PixelRgb> &yrpImgOut=outPort->prepare();
        yrpImgOut.resize(_input_cols,_input_lines);
        cvCvtColor(_iplimage, yrpImgOut.getIplImage(),CV_BGR2RGB);
        outPort->write();

        return found;
    }

    virtual bool updateModule()
    {
        Vector q=traj.getRow(cnt);
        posHead->positionMove(q.data());

        bool doneHead=false;
        while (doneHead)
            posHead->checkMotionDone(&doneHead);

        Vector qTorso(3);
        Vector qHead(6);
        while (!encsTorso->getEncoders(qTorso.data()));
        while (!encsHead->getEncoders(qHead.data()));

        Vector dataLeft, dataRight;
        int retLeft=processImages("left",dataLeft);
        int retRight=processImages("right",dataRight);
		
        cout << "#" << cnt << ":\t";

        if (retLeft && retRight)
        {
            qTorso=(M_PI/180.0)*qTorso;
            qHead =(M_PI/180.0)*qHead;

            string s=(string)qTorso.toString()+"\t"+(string)qHead.toString()+"\t"+
                     (string)dataLeft.toString()+"\t"+(string)dataRight.toString()+"\n";

            cout << s;
            fout << s;
        }
        else
            cout << "--- CANNOT DETECT ONE or BOTH IMAGES ---" << endl;

        if (++cnt>=TRAJPOINTS)
            return false;
        else
            return true;
    };

    virtual double getPeriod() { return 1.0; }
};

int main(int argc, char *argv[])
{
    Network yarp;
    Time::turboBoost();

    ResourceFinder rf;    
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    alignKin2Cam module;
    module.setName("/alignKin2Cam");

    return module.runModule(rf);
}


