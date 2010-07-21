#include <iCub/DisparityTool.h>

//// yarp
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

// std
#include <stdio.h>
#include <iostream>

#include <iCub/iKin/iKinFwd.h>
#include <yarp/math/Math.h>
//#include <iCub/YARPRobotMath.h>
//#include <math.h>

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


class disparityProcessor : public RateThread {
private:
	
    yarp::os::Port cmdOutput;
    yarp::os::Port cmdPort;

	string outputPortName; 
	//property to get encoders 
    yarp::os::Property optionsHead, optionsTorso;
    yarp::dev::IEncoders *encHead, *encTorso;
	
    yarp::dev::PolyDriver *robotHead, *robotTorso;

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

    /**
    * left camera port
    */
    yarp::os::BufferedPort < ImageOf<PixelRgb > > imageInLeft;  //
    /**
    * right camera port
    */
    yarp::os::BufferedPort < ImageOf<PixelRgb > >    imageInRight; //
    /**
    * output histogram
    */
    yarp::os::BufferedPort < ImageOf<PixelMono > >   histoOutPort; //
    /**
    * image output port
    */
    BufferedPort<ImageOf<PixelRgb> >  imageOutputPort; 

   	bool needLeft, needRight;
    int imgNumb;
	float ratio;
	FILE *fout;
    
   	
    ImageOf<PixelRgb> Limg;
    ImageOf<PixelRgb> Rimg;

	DisparityTool Disp;

	shift_Struct maxes[4];

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

	//virtual bool open(Searchable& config);
	//virtual bool close();
	//virtual bool interruptModule();
	//virtual bool updateModule();

    /**
    * DEPRECATED
    */
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

    // ______ public attributes __________
    /**
    * reference to the left log polar image
    */
    ImageOf<PixelRgb> *imgInL;
    /**
    * reference to the right log polar image
    */
	ImageOf<PixelRgb> *imgInR;
    /**
    * image where the histogram is provided
    */
    ImageOf<PixelMono> histo;
    /**
    * angle displacement for zero disparity
    */
    double angle;

};

