// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea, Vadim Tikhanoff
 * email:   francesco.rea@iit.it
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

#include <iCub/disparityTool.h>
// yarp
#include <yarp/os/RateThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Image.h>
// std
#include <stdio.h>
#include <iostream>

#include <iCub/iKin/iKinFwd.h>
#include <yarp/math/Math.h>

const double F = 4;						/// camera F length.
const double PixScaleX = 120;			/// camera mm to pixel conversion factor.
const double PixScaleY = 98;			/// same along the y coord.
const double Periphery2Fovea = 2.0;	
const int _centerX = 128/2;
const int _centerY = 128/2;

const int CenterPeripheryX = 256/2;
const int CenterPeripheryY = 256/2;
const int CenterFoveaX = 128/2;
const int CenterFoveaY = 128/2;

const double _maxVerg = 50 * M_PI/180;
const double _minVerg = 0 * M_PI/180;

class disparityProcessor : public yarp::os::RateThread {
private:

    yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelRgb > > imageInLeft;  //left camera port
    yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelRgb > >  imageInRight; //right camera port
    yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelMono > >  histoOutPort; //output histogram
    yarp::os::Port cmdOutput;
	std::string outputPortName; 
    yarp::os::Property optionsHead, optionsTorso;
    yarp::dev::IEncoders *encHead, *encTorso;
	
    yarp::dev::PolyDriver *robotHead, *robotTorso;

    iCub::iKin::iCubEye *leftEye, *rightEye;
	
	iCub::iKin::iKinLink *leftLink, *rightLink;
	iCub::iKin::iKinChain *chainRightEye,  *chainLeftEye;

	yarp::sig::Matrix HL, HR ;
	yarp::sig::Vector fb, zl, pl, dl, ml;

	yarp::sig::Matrix _Ti, _Ti0, _TB0;

	yarp::sig::Vector _q, _it, _o, _epx, _tmp, _tmpEl;

	yarp::sig::Vector _leftJoints, _rightJoints, _joints;

	yarp::sig::Vector _fixationPoint, _fixationPolar;
	
	int _nFrame;

	double leftMax, leftMin, rightMax, rightMin;

   	bool needLeft, needRight;
	float ratio;

    yarp::sig::ImageOf<yarp::sig::PixelRgb> Limg;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> Rimg;

	DisparityTool Disp;

	shift_Struct maxes[4];

    std::string moduleName, robotName; 
    yarp::sig::Vector tempV, tmpPos;
    yarp::sig::Vector _head, _torso;
    bool dispInit;
    

public:
	
	typedef enum { KIN_LEFT = 1, KIN_RIGHT = 2, KIN_LEFT_PERI = 3, KIN_RIGHT_PERI = 4 } __kinType;

	disparityProcessor();
	~disparityProcessor();
    /**
    *	initialization of the thread 
    */
    bool threadInit();
    /**
    * active loop of the thread
    */
    void run();
    /**
    *	releases the thread
    */
    void threadRelease();
    void onStop();

    /**
    * function that computes the ray vector passing from x,y coordinates in image plane
    */
    void computeRay (__kinType k, Vector& v, int x, int y); 
    void peripheryToFovea (int x, int y, int& rx, int& ry) { rx = int(x * Periphery2Fovea); ry = int(y * Periphery2Fovea); }
    void foveaToPeriphery (int x, int y, int& rx, int& ry) { rx = int(x / Periphery2Fovea); ry = int(y / Periphery2Fovea); } 
    void intersectRay (__kinType k, const Vector& v, int& x, int& y);
    void computeDirect (const Vector &joints);
    void computeFixation (const Matrix &T1, const Matrix &T2);
    void setName(string module, string robot);

    void _cartesianToPolar(const Vector &cartesian, Vector &polar) {
	    polar(0) = atan2(cartesian(1), -cartesian(0));
	    double tmp = sqrt(cartesian(0)*cartesian(0) + cartesian(1)*cartesian(1));// azimuth		
	    polar(1) = atan2(cartesian(2),tmp);// elevation
	    polar(2) = sqrt(cartesian(0)*cartesian(0) + cartesian(1)*cartesian(1)+cartesian(2)*cartesian(2));// distance
    }

    virtual double getPeriod() {return 0.05;}
	
    bool respond(const Bottle& command, Bottle& reply) {
    if (command.get(0).asString() == "quit")
	    return false;     
        else if (command.get(0).asString() == "help"){
            cout << " \nWell quite simple....\n" << endl;
            cout << " quit    - quits the program ....hhhmmm difficult this one" << endl;
        }
        else
        {
            cout << "command not known - type help for more info" << endl;
        }
        return true;
    }

    yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgInL; //reference to the left log polar image

    yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgInR; //reference to the right log polar image

    yarp::sig::ImageOf<yarp::sig::PixelMono> histo; //image where the histogram is provided

    double angle;//angle displacement for zero disparity

};

