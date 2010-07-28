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

#include <iCub/DisparityTool.h>
// yarp
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
// std
#include <stdio.h>
#include <iostream>

#include <iCub/iKin/iKinFwd.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::os;
using namespace iCub::iKin;

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

class disparityProcessor : public RateThread {
private:

    BufferedPort < ImageOf<PixelRgb > > imageInLeft;  //left camera port
    BufferedPort < ImageOf<PixelRgb > >  imageInRight; //right camera port
    BufferedPort < ImageOf<PixelMono > >  histoOutPort; //output histogram
    Port cmdOutput;
	string outputPortName; 
    Property optionsHead, optionsTorso;
    IEncoders *encHead, *encTorso;
	
    PolyDriver *robotHead, *robotTorso;

    iCubEye *leftEye, *rightEye;
	
	iKinLink *leftLink, *rightLink;
	iKinChain *chainRightEye,  *chainLeftEye;

	Matrix HL, HR ;
	Vector fb, zl, pl, dl, ml;

	Matrix _Ti, _Ti0, _TB0;

	Vector _q, _it, _o, _epx, _tmp, _tmpEl;

	Vector _leftJoints, _rightJoints, _joints;

	Vector _fixationPoint, _fixationPolar;
	
	int _nFrame;

	double leftMax, leftMin, rightMax, rightMin;

   	bool needLeft, needRight;
	float ratio;

    ImageOf<PixelRgb> Limg;
    ImageOf<PixelRgb> Rimg;

	DisparityTool Disp;

	shift_Struct maxes[4];

    string moduleName, robotName; 
    Vector tempV, tmpPos;
    Vector _head, _torso;
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

    ImageOf<PixelRgb> *imgInL; //reference to the left log polar image

    ImageOf<PixelRgb> *imgInR; //reference to the right log polar image

    ImageOf<PixelMono> histo; //image where the histogram is provided

    double angle;//angle displacement for zero disparity

};

