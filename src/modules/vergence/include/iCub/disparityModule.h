


#include "DisparityTool.h"
//#include <yarp/sig/Image.h>
//#include <yarp/sig/Vector.h>
//#include <yarp/sig/ImageFile.h>
/*#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/sig/Vector.h>*/
//// yarp
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

// std
#include <stdio.h>
#include <iostream>

#include <iCub/iKinFwd.h>
#include <yarp/math/Math.h>
//#include <YARPRobotMath.h>
//#include <math.h>

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::os;
//using namespace iKin;

const double F = 4;						/// camera F length.
const double PixScaleX = 120;			/// camera mm to pixel conversion factor.
const double PixScaleY = 98;			/// same along the y coord.
const double Periphery2Fovea = 2.0;	
const int _centerX = 128/2;
const int _centerY = 128/2;

/*
const int CenterPeripheryX = _logpolarParams::_xsize/2;
const int CenterPeripheryY = _logpolarParams::_ysize/2;
const int CenterFoveaX = _logpolarParams::_xsizefovea/2;
const int CenterFoveaY = _logpolarParams::_ysizefovea/2;
*/

const int CenterPeripheryX = 256/2;
const int CenterPeripheryY = 256/2;
const int CenterFoveaX = 128/2;
const int CenterFoveaY = 128/2;


const double _maxVerg = 50* M_PI/180;
const double _minVerg = 0 * M_PI/180;


class disparityModule : public Module {

private:

	BufferedPort<ImageOf<PixelRgb> >  imageOutputPort;
	Port  cmdOutput;

	string outputPortName; 
	//property to get encoders 
	Property optionsHead, optionsTorso;
	IEncoders *encHead, *encTorso;
	
	PolyDriver *robotHead, *robotTorso;

	iCubEye *leftEye;
	iCubEye *rightEye;
	
	iKinLink *leftLink;
	iKinLink *rightLink;
	iKinChain *chainRightEye,  *chainLeftEye;

	Matrix HL, HR ;
	Vector fb, zl, pl, dl, ml;

	Matrix _Ti;
	Matrix _Ti0;
	Matrix _TB0;

	Vector _q;
	Vector _it;
	Vector _o;
	Vector _epx;
	Vector _tmp;
	Vector _tmpEl;

	Vector _leftJoints;
	Vector _rightJoints;
	Vector _joints;

	Vector _fixationPoint;
	Vector _fixationPolar;
	
	int _nFrame;

	double leftMax, leftMin, rightMax, rightMin;

	BufferedPort < ImageOf<PixelRgb > >    imageInLeft;  //left camera port
	BufferedPort < ImageOf<PixelRgb > >    imageInRight; //right camera port
	BufferedPort < ImageOf<PixelMono > >   histoOutPort; //output histogram

   	bool needLeft, needRight;
    int imgNumb;
	float ratio;
	FILE *fout;

   	ImageOf<PixelRgb> *imgInL;
	ImageOf<PixelRgb> *imgInR;
	ImageOf<PixelMono> histo;
	ImageOf<PixelRgb> Limg;
	ImageOf<PixelRgb> Rimg;

	DisparityTool Disp;

	shift_Struct maxes[4];

public:
	
	typedef enum { KIN_LEFT = 1, KIN_RIGHT = 2, KIN_LEFT_PERI = 3, KIN_RIGHT_PERI = 4 } __kinType;

	disparityModule();
	~disparityModule();
	void computeRay (__kinType k, Vector& v, int x, int y); // computes ray that intesects with image plane
	void peripheryToFovea (int x, int y, int& rx, int& ry) { rx = int(x * Periphery2Fovea); ry = int(y * Periphery2Fovea); }
	void foveaToPeriphery (int x, int y, int& rx, int& ry) { rx = int(x / Periphery2Fovea); ry = int(y / Periphery2Fovea); } 
	void intersectRay (__kinType k, const Vector& v, int& x, int& y);
	void computeDirect (const Vector &joints);
	void computeFixation (const Matrix &T1, const Matrix &T2);

	void _cartesianToPolar(const Vector &cartesian, Vector &polar)
	{
		polar(0) = atan2(cartesian(1), -cartesian(0));
		// azimuth
		double tmp = sqrt(cartesian(0)*cartesian(0) + cartesian(1)*cartesian(1));			
		// elevation
		polar(1) = atan2(cartesian(2),tmp);
		// distance
		polar(2) = sqrt(cartesian(0)*cartesian(0) + cartesian(1)*cartesian(1)+cartesian(2)*cartesian(2));
	}

	virtual bool open(Searchable& config);
	virtual bool close();
	virtual bool interruptModule();
	virtual bool updateModule();
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
};

