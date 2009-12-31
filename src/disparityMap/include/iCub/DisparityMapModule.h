/** 
 * @ingroup icub_module
 *
 * \defgroup icub_DisparityMapModule DisparityMapModule
 *
 *
 * This is a module calculates an active-disparity map taking advantage of the encoders data.
 *
 *
 * The module takes both images and the values of the encoders to calculate the rectified images. These rectified images 
 * ensure that the epipolar line is the horizontal line in the image. Then the correspondance between the images is done 
 * and the result is the disparity map.
 * 
 * \section lib_sec Libraries
 *
 * YARP.
 * OpenCV.
 * libdc1394-2 (Linux) PGRFlyCapture (Windows).
 * 
 * \section parameters_sec Parameters
 * 
 *
 * <b> Configuration File Parameters </b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *
 *
 * - \c deviationLeft \c 0.0013 \n
 *   specifies in grades the deviation of the left camera to the Y axes
 *
 * - \c deviationRight \c -0.0017 \n
 *   specifies in grades the deviation of the right camera to the Y axes
 *
 * - \c deviationX \c 0.0013 \n
 *   specifies in grades the deviation of the right camera to the left camera in the X axes
 *
 * - \c deviationZ \c 0.0013 \n
 *   specifies in grades the deviation of the right camera to the keft camera in the Z axes
 *
 * - \c Leftfx \c 440.41 \n
 *   specifies left camera calibration parameter fx
 *
 * - \c Leftcx \c 168 \n
 *   specifies left camera calibration parameter cx
 *
 * - \c Leftfy \c 440.41 \n
 *   specifies left camera calibration parameter fy
 *
 * - \c Leftcx \c 115.41 \n
 *   specifies left camera calibration parameter cx
 *
 * - \c Rightfx \c 440.41 \n
 *   specifies right camera calibration parameter fx
 *
 * - \c Rightcx \c 168 \n
 *   specifies right camera calibration parameter cx
 *
 * - \c Rightfy \c 440.41 \n
 *   specifies right camera calibration parameter fy
 *
 * - \c Rightcx \c 115.41 \n
 *   specifies right camera calibration parameter cx
 *
 * 
 * \section portsa_sec Ports Accessed
 * 
 *  /icub/head/
 *
 *                      
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 *  - \c /disparitymap/right \n
 *    This port is used to introduce the right image 320X240
 *
 *  - \c /disparitymap/left \n
 *    This port is used to introduce the left image 320X240
 * 
 *
 * <b>Output ports</b>
 *
 *  - \c /disparitymap/transformright \n
 *    This port show the right image after the rectification.
 *
 *  - \c /disparitymap/transformleft \n
 *    This port show the left image after the rectification.
 *
 *  - \c /disparitymap/outColor \n
 *    This port show the disparoty map in colors white is the zero, blue is after the zero region
 *    and the green is between the robot and the zero region.  
 *
 *  - \c /disparitymap/mapOpenCv \n
 *    This port show the disparity map in gray scale. 
 *
 *
 *
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>disparitymap --nameControlboard /icub/head --file E:/RobotCub/iCub/conf/disparitymap.ini</tt>
 *
 * \author 
 * 
 * Harold Martinez
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at \c $ICUB_ROOT/src/disparityMap/include/iCub/DisparityMapModule.h
 * 
 */


/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Harold Martinez
 * email:   martinez@ifi.uzh.ch
 *
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

#ifndef _DISPARITYMAPMODULE_
#define _DISPARITYMAPMODULE_

// std
#include <iostream>
#include <string>
#include <math.h>
#include <time.h>
#include <fstream> 

//opencv
#include <cv.h>
#include <highgui.h>

// yarp
#include <yarp/os/Module.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>




using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
/**
 * Classes developed as part of the iCub project.
 */
namespace iCub {
    /**
     *
     * Contributed classes.  Developers can put anything they like
     * here.  Over time, these classes may get reorganized into other
     * namespaces and made "official".  In that case, a typedef (or
     * equivalent) should be retained here for some time so that users
     * don't have to worry about the classes disappearing.
     *
     */
    namespace contrib {
        class DisparityMapModule;
    }
}


using namespace iCub::contrib;

/**
 * Module to compute the depth map
 */
class iCub::contrib::DisparityMapModule :  public Module{
								 //public IDepthMap	{


private:
    BufferedPort<ImageOf<PixelRgb> > imgPortRight;
	BufferedPort<ImageOf<PixelRgb> > imgPortLeft;
	BufferedPort<ImageOf<PixelRgb> > imgPortTransfomRight;
	BufferedPort<ImageOf<PixelRgb> > imgPortTransfomLeft;
	BufferedPort<ImageOf<PixelRgb> > imgOutColor;
    BufferedPort<ImageOf<PixelFloat> > imgPortDepthOpenCv;
	BufferedPort<ImageOf<PixelFloat> > imgPortDerivXDepth;
	BufferedPort<ImageOf<PixelFloat> > imgPortDerivYDepth;
	BufferedPort<ImageOf<PixelFloat> > imgZDP;
	BufferedPort<ImageOf<PixelMono> > imgOutD;
	BufferedPort<ImageOf<PixelFloat> > imgDepthMap;
	
	//BufferedPort<ImageOf<PixelFloat> > imgPortGrayRight;
	//BufferedPort<ImageOf<PixelFloat> > imgPortGrayLeft;
	
	BufferedPort<yarp::os::Bottle> _configPort;
	BufferedPort<yarp::os::Bottle>  encoders;
	

    
	
	
    Semaphore mutex;
	
	int maxDisparity;
	int minDisparity;
	int oldWidth;
	int oldHeight;
	//camera parameters

	

	CvMat *aRight;
	CvMat *aLeft;
	CvMat *tRight;
	CvMat *tLeft;
	CvMat *rotRight;
	CvMat *rotLeft;
	
	double deviationLeft;
	double deviationRight;
	double deviationZ;
	double deviationX;
	double Leftfx ;
	double Leftfy ;
	double Leftcx ;
	double Leftcy ;

	double Rightfx ;
	double Rightfy ;
	double Rightcx ;
	double Rightcy;


	 // controlboard
    PolyDriver			                _dd;
    IEncoders			                *_ienc;
    IPositionControl                    *_ipos;
    double                              *_encoders;
    int                                 _numAxes;

	//file
	ofstream outClientFile;

	//OpenCV StereoVision
	CvStereoBMState * state;
	int filterSize,filterCap,windowSize,
	  numDisparities, threshold, uniqueness;

	


	
	virtual void finalMapOpenCV(IplImage *intenseRigth, 
                          IplImage *intenseLeft, 
						  CvMat* disp,int minD, int numD);

	void depthMapCalculation(IplImage *disparity,IplImage *depthMap);

	virtual void rectification(IplImage *left,IplImage *right,CvMat * Ar,CvMat * Al,CvMat *Rr, CvMat *Rl,
							CvMat *Tr, CvMat *Tl,IplImage *destLeft,IplImage *destRight,
							int originalWidth, int originalHeight);

	virtual void transformationMatrix(CvMat * Ar,CvMat * Al,CvMat *Rr, CvMat *Rl,CvMat *Tr, CvMat *Tl,
							double dx1,double dx2, double dy,
							CvMat *TTl,CvMat *TTr);


	void showColorDepthMap(ImageOf<PixelRgb>& imgColorOut, CvMat* disp);

	
	void upPyramidInformation(CvMat *lowLayer,CvMat *layer, int minD);

	
	
	double diffclock(clock_t clock1,clock_t clock2);
	void rotationTraslation(double j4,double j5);

    void saveFiles( ImageOf<PixelFloat> &img,string name);
	void saveFile( float *intens,string name);
    
public:

    DisparityMapModule();
    virtual ~DisparityMapModule();

    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
	virtual bool respond(const Bottle &command,Bottle &reply);




};

#endif
