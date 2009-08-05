#pragma once

// std
#include <math.h>
#include <iostream>

// yarp
#include <yarp/String.h>
#include <yarp/os/Property.h>
#include <yarp/os/Module.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
// ipp
#include "ipp.h"

using namespace yarp;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

class CTRLmodule : public Module {
private:
	static const int dim_head = 6;	// Number of active motors
	static const int dim_net = 12;	// Number of neurons
	int k;//, dim_head;
	bool verb;						// Verbose for...

	Ipp64f gain_eyes_yaw;		// Time constant of excitatory field
	Ipp64f gain_eyes_pitch;		// Time constant of inhibitory field
	Ipp64f gain_neck_yaw;		// Time constant for building memory traces
	Ipp64f gain_neck_pitch;		// Time constant for memory traces to decay

	Ipp64f *U;
	Ipp64f *S;

	// Drivers
    PolyDriver			head_driver;
    IEncoders			*head_enc;
    IPositionControl	*head_pos;
    IVelocityControl	*head_vel;
    IPidControl			*head_pid;
	IAmplifierControl	*head_amp;

	// Ports
    String remote_head;
    String local_head;
	BufferedPort< Vector >	inPort;		// Port for reading data

public:
	CTRLmodule(void);
	~CTRLmodule(void);

    virtual bool open( Searchable& );
    virtual bool close( void );
    virtual bool interruptModule( void );
    virtual bool updateModule( void );

	bool setZeroPosition( void );
	bool setZeroVelocity( void );

	FILE *file_m;// --- This file will save all my data ---

	Ipp64f *buffer_work1;
	Ipp64f *buffer_work2;

};
