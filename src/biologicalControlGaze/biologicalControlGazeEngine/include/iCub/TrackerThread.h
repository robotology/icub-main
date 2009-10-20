// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/**
 * @ingroup icub_module
 *
 * \defgroup icub_yet_another_headcontrol yaHeadControl
 *
 * This is YetAnotherHeadControl. Written for fun.
 * Ports opened:
 * /icub/right/target
 * /icub/left/target
 * Both accept a bottle of three numbers x,y,code, output of a 
 * tracker. The controller connects to iCubInterface. Uses 
 * monocular info form either one of the eyes or stereo from 
 * both if available.
 * 
 * Read configuration info from file
 *
 * Run as:
 * yaHeadControl --file filename
 *
 * E.g. yaHeadControl --file icub_yaheadcontrol.ini
 *
 * 
 * \author Lorenzo Natale
 *
 */

#include <ace/config.h>
#include <ace/OS.h>
#include <ace/Log_Msg.h>

#include <yarp/os/PortablePair.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Terminator.h>
#include <yarp/String.h>
#include <yarp/os/RateThread.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Vector.h>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

const double TIMEOUT=0.1;
const double STOP_TIME=3;
const int THREAD_RATE=30;

const double START_POSITION[]={0,0,0,0,0,10,0,0};
const double START_SPEED[]={10,10,10,10,10,10,0,0};

class TrackerThread: public RateThread
{
public:
    BufferedPort<yarp::sig::Vector> *targetLeft; 
	BufferedPort<yarp::sig::Vector> *targetRight;
	Port *trackingPort;
	BufferedPort<yarp::sig::Vector> *inertial;
	double vorx;
    double vory;
    double vorz;
    double Kvergence;
    double Kpan;
    double Ktilt;
    double Kn_pan;
    double Kn_tilt;

    PolyDriver dd;
    bool init;
	bool just_eyes;
    Property options;
    IEncoders *iencs;
    IVelocityControl *ivel;
    IPositionControl *ipos;
    int joints;
	Vector *v_r,*v_l;

    double *encoders;
    double *command;
    double *accelerations;
    double *prev;

    double timeStampL;
    double timeStampR;
    double timeStampLprev;
    double timeStampRprev;
    double timeStampTO;

    Vector targetL;
    Vector targetR;

    bool enableVOR;
    bool enableTracking;

public:
	/**
	* constructor of the thread
	*/
	TrackerThread(Property &op);
	/**
	* destructor of the thread
	*/
	~TrackerThread();
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
	* set the left vector for the gaze motion
	* @param a first component of the 3dimensional vector
	* @param b second component of the 3dimensional vector
	* @param c third component of the 3dimensional vector
	*/
	void setLeftVector(double a, double b, double c);
	/**
	*	set the vector right for the gaze motion
	* @param a first component of the 3dimensional vector
	* @param b second component of the 3dimensional vector
	* @param c third component of the 3dimensional vector
	*/
	void setRightVector(double a, double b, double c);
	/**
	*	reset the motion vector
	*/
	void resetVector();
};