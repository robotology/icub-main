/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
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

/** 
\defgroup joystickCtrl joystickCtrl
 
@ingroup icub_tools
 
A configurable tool to send on a yarp port the data retrieved from a joystick.
 
Copyright (C) 2010 RobotCub Consortium
 
Author: Marco Randazzo

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
 
This module reads the axis data from a connected joystick and
outputs them on a yarp port. The format of the output can be specified using a configuration file.
 
\section lib_sec Libraries 
- YARP libraries. 
- SDL libraries.

\section parameters_sec Parameters
The only used parameter is the name of the configuration file. The configuration file name can be specified using --from \e file 
You can also use the \e --context option to change the current context (e.g. the directory where to search the configuration file).
 
\section portsa_sec Ports Accessed
None. 
 
\section portsc_sec Ports Created 
The module creates the port /joystickCtrl:o used to transmit the joystick data.
The output of the port consists in a sequence of <n> doubles (<n> depending on the number of axes specified
in the configuration file) containing the readings.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files
A description of the available configuration options can be found in the example files located under /app/joystickControl/conf
 
\section tested_os_sec Tested OS
Windows, Linux

\author Marco Randazzo
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>

//#include <gsl/gsl_math.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <math.h>
#include <SDL.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

#define PRINT_STATUS_PER 0.1

enum { JTYPE_UNDEF=-1, JTYPE_POLAR=0, JTYPE_CARTESIAN=1, JTYPE_CONSTANT=2 };

class CtrlThread: public RateThread
{
	struct struct_jointProperties
	{
		int type; 
		int param[3];
	};

protected:
    ResourceFinder      &rf;
    BufferedPort<Bottle> port_command;

    double defaultExecTime;
    double t0;
	int numAxes;
	int numBalls;
	int numHats;
	int numButtons;
	SDL_Joystick* joy1;

	//variables read from the joystick
	int*    rawButtons;
	double* rawAxes;
	double* outAxes;

	//variables read from the configuration file
	int  num_inputs;
	int  num_outputs;
	double* jointGain;
	double* jointMax;
	double* jointMin;
	double* jointOffset;
	struct_jointProperties* jointProperties;

public:
    CtrlThread(unsigned int _period, ResourceFinder &_rf) :
               RateThread(_period),     rf(_rf)
	{
		rawButtons=0;
		rawAxes=0;
		outAxes=0;
		joy1=0;
		num_inputs=0;
		num_outputs=0;
		jointGain=0;
		jointMax=0;
		jointMin=0;
		jointOffset=0;
		jointProperties=0;
	}

    virtual bool threadInit()
    {
		// get params from the RF
		if (rf.findGroup("INPUTS").check("InputsNumber"))
		{
			num_inputs = rf.findGroup("INPUTS").find("InputsNumber").asInt();
			jointGain = new double [num_inputs];
			jointMax = new double [num_inputs];
			jointMin = new double [num_inputs];
			jointOffset = new double [num_inputs];
			fprintf ( stderr, "Number of input axes in the configuration options: %d \n",num_inputs);
		}
		else
		{
			fprintf ( stderr, "Unable to find number of input axes in the configuration options\n");
			return false;
		}

		Bottle b;
		
		b = rf.findGroup("INPUTS").findGroup("Gain");
		if (b.size()-1 == num_inputs)
			{
				for (int i = 1; i < b.size(); i++) jointGain[i-1] = b.get(i).asDouble();
			}
		else {fprintf ( stderr, "Configuration error: invalid number of entries 'Gain'\n"); return false;}
		
		b = rf.findGroup("INPUTS").findGroup("Max");
		if (b.size()-1 == num_inputs)
			{
				for (int i = 1; i < b.size(); i++) jointMax[i-1] = b.get(i).asDouble();
			}
		else {fprintf ( stderr, "Configuration error: invalid number of entries 'Max'\n"); return false;}
		
		b = rf.findGroup("INPUTS").findGroup("Min");
		if (b.size()-1 == num_inputs)
			{
				for (int i = 1; i < b.size(); i++) jointMin[i-1] = b.get(i).asDouble();
			}
		else {fprintf ( stderr, "Configuration error: invalid number of entries 'Min'\n"); return false;}

		b = rf.findGroup("INPUTS").findGroup("Offset");
		if (b.size()-1 == num_inputs)
			{
				for (int i = 1; i < b.size(); i++) jointOffset[i-1] = b.get(i).asDouble();
			}
		else {fprintf ( stderr, "Configuration error: invalid number of entries 'Offset'\n"); return false;}

		if (rf.findGroup("OUTPUTS").check("OutputsNumber"))
		{
			num_outputs = rf.findGroup("OUTPUTS").find("OutputsNumber").asInt();
			jointProperties = new struct_jointProperties [num_outputs];
		}
		else
		{
			fprintf ( stderr, "Unable to find number of output axes in the configuration options\n");
			return false;
		}

		for (int i=0; i<num_outputs; i++)
		{
			char tmp [20];
			sprintf (tmp,"Ax%d",i);
			if (!rf.findGroup("OUTPUTS").check(tmp))
			{
				fprintf ( stderr, "Error reading [OUTPUT] block, unable to find Ax%d identifier\n",i);
				return false;
			}
			b = rf.findGroup("OUTPUTS").findGroup(tmp);
			if (b.get(1).asString()=="polar_r_theta")
			{
				jointProperties[i].type=JTYPE_POLAR;
				jointProperties[i].param[0]=b.get(2).asInt();
				jointProperties[i].param[1]=b.get(3).asInt();
				jointProperties[i].param[2]=b.get(4).asInt();
			}
			else
			if (b.get(1).asString()=="cartesian_xyz")
			{
				jointProperties[i].type=JTYPE_CARTESIAN;
				jointProperties[i].param[0]=b.get(2).asInt();
				jointProperties[i].param[1]=0;
				jointProperties[i].param[2]=0;
			}
			else
			if (b.get(1).asString()=="constant")
			{
				jointProperties[i].type=JTYPE_CONSTANT;
				jointProperties[i].param[0]=b.get(2).asInt();
				jointProperties[i].param[1]=0;
				jointProperties[i].param[2]=0;
			}
			else
			{
				fprintf ( stderr, "Unknown [OUTPUT] property\n");
				return false;
			}
		}
		//string s = b.toString(); //this causes compilation issues under linux

		// open the output port
		port_command.open("/joystickCtrl:o");

		// start SDL subsystem
		if ( SDL_InitSubSystem ( SDL_INIT_JOYSTICK ) < 0 )
		{
			fprintf ( stderr, "Unable to initialize Joystick: %s\n", SDL_GetError() );
			return false;
		}

		// get the list of available joysticks
		fprintf ( stderr, "\n");
		int joy_id=0;
		int joystick_num = SDL_NumJoysticks ();
		if (joystick_num == 0)
		{
			fprintf ( stderr, "Error: No joysticks found\n"); return false;
		}
		else if (joystick_num == 1)
		{
			fprintf ( stderr, "One joystick found \n");
			fprintf ( stderr, "Using joystick: %s \n", SDL_JoystickName(0));
		}
		else
		{
			fprintf ( stderr, "More than one joystick found:\n");
			for (int i=0; i<joystick_num; i++)
			{
				fprintf ( stderr, "%d: %s\n",i,SDL_JoystickName(i));
			}
			fprintf ( stderr, "\n");

			// choose between multiple joysticks
			if (rf.findGroup("GENERAL").check("DefaultJoystickNumber"))
			{
				joy_id = rf.findGroup("GENERAL").find("DefaultJoystickNumber").asInt();
				fprintf ( stderr, "Multiple joysticks found, using #%d, as specified in the configuration options\n", joy_id);
			}
			else
			{
				fprintf ( stderr, "No default joystick specified in the configuration options\n");
				fprintf ( stderr, "Which joystick you want to use? (choose number) \n");
				cin >> joy_id;
			}
		}

		// Open the Joystick driver
		joy1 = SDL_JoystickOpen ( joy_id );
		if ( joy1 == NULL )
		{
			printf ( "Could not open joystick\n" );
			return false;
		}

		// Obtaining Joystick capabilities
		numAxes = SDL_JoystickNumAxes ( joy1 );
		numBalls = SDL_JoystickNumBalls ( joy1 );
		numHats = SDL_JoystickNumHats ( joy1 );
		numButtons = SDL_JoystickNumButtons ( joy1 );
		fprintf ( stderr, "Characteristics of joy %d: \n", joy_id);
		fprintf ( stderr, "%i Axes\n", numAxes );
		fprintf ( stderr, "%i Balls\n", numBalls );
		fprintf ( stderr, "%i Hats\n",  numHats );
		fprintf ( stderr, "%i Buttons\n", numButtons );
		fprintf ( stderr, "\n");

		// check: selected joint MUST have at least one axis
		if (numAxes>0) 
		{
			rawAxes=new double [numAxes];
			outAxes=new double [numAxes];
		}
		else
		{
			fprintf ( stderr, "Error: selected joystick has 0 Axes?!\n" );
			return false;
		}

		if (numAxes!=num_inputs)
		{
			fprintf ( stderr, "Warning: # of joystick axes (%d) differs from # of configured input axes (%d)!\n",numAxes,num_inputs );
			fprintf ( stderr, "This probably means that your .ini file does not containt a correct configuration.\n");
			fprintf ( stderr, "Do you want to continue anyway (y/n)?\n");
			char input[255];
			cin >> input;
			if (input[0]!='y' && input[0]!='Y') 
			{
				fprintf ( stderr, "Quitting...\n");
				return false;
			}
			else
			{
				fprintf ( stderr, "Ovveriding the number of axes specified in the configuration file. Using %d axes.\n",numAxes);
			}
		}

		/*
		// check: selected joint MUST have at least one button
		if (numButtons>0)
			rawButtons=new int [numButtons];
		else
		{
			printf ( "Error reading numButtons\n" );
			return false;
		}*/
        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            fprintf ( stderr, "Thread started successfully\n");
        else
            fprintf ( stderr, "Thread did not start\n");

        t0=Time::now();
    }

    virtual void run()
    {
		// This is needed in the event queue
		SDL_JoystickUpdate ();

		// Reading joystick data (axes/buttons...)
		for ( int i=0; i < numButtons; ++i )
		{
//			rawButtons[i] = SDL_JoystickGetButton ( joy1, i );
		}

		for ( int i=0; i < numAxes; ++i )
		{
			rawAxes[i] = (double)SDL_JoystickGetAxis ( joy1, i );
		}

		// Formatting input data
		for(int i=0;i<num_inputs;i++)
		{
			double v = rawAxes[i];
			v = v+jointOffset[i];
			v = v*jointGain[i];
			v = (v<jointMax[i]) ? v : jointMax[i];
			v = (v>jointMin[i]) ? v : jointMin[i];
			rawAxes[i]=v;
		}

		// Sending data out on a Yarp port
		Bottle data;
		for(int i=0;i<num_outputs;i++)
		{			
			if (jointProperties[i].type == JTYPE_POLAR)
			{
				if (jointProperties[i].param[2] == 0)
				{
					outAxes[i]= atan2(  (rawAxes[jointProperties[i].param[0]]),
								  		(rawAxes[jointProperties[i].param[1]]) ) * 180.0 / 3.14159265;
				}
				else if (jointProperties[i].param[2] == 1)
				{
					outAxes[i]= sqrt (  pow((rawAxes[jointProperties[i].param[0]]),2)+
										pow((rawAxes[jointProperties[i].param[1]]),2) );
				}
				else
				{
					outAxes[i]=0.0;
					fprintf ( stderr, "Unknown parameter for JTYPE_POLAR, joint %d\n",i);
				}
			}
			else if (jointProperties[i].type == JTYPE_CARTESIAN)
			{
				outAxes[i]=rawAxes[jointProperties[i].param[0]];
			}
			else if (jointProperties[i].type == JTYPE_CONSTANT)
			{
				outAxes[i]=(jointProperties[i].param[0]);
			}
			else
			{
				outAxes[i]=0.0;
				fprintf ( stderr, "Unknown property, joint %d\n",i);
			}
		}

		// Sending data on the yarp port
		for(int i=0;i<num_outputs;i++)
			{			
				data.addDouble(outAxes[i]);
			}
		port_command.prepare() = data;
		port_command.write();

		// Displaying status
        printStatus();
    }

    virtual void threadRelease()
    {    
		if (rawAxes)         delete [] rawAxes;
		if (outAxes)         delete [] outAxes;
		if (rawButtons)      delete [] rawButtons;
		if (jointGain)       delete [] jointGain;
		if (jointMax)        delete [] jointMax;
		if (jointMin)        delete [] jointMin;
		if (jointOffset)     delete [] jointOffset;
		if (jointProperties) delete [] jointProperties;
        port_command.interrupt();
        port_command.close();
    }


    void printStatus()
    {
        double t=Time::now();

        if (t-t0>=PRINT_STATUS_PER)
        {
			//for (int i=0;i <numButtons; i++)
			//	printf ( "%+6d", rawButtons[i] );

			for (int i=0;i <numAxes; i++)
			{
				fprintf ( stderr, "%+9.1f", rawAxes[i] );
			}
			fprintf ( stderr, " ---> " );
			for (int i=0;i <num_outputs; i++)
			{
				fprintf ( stderr, "%+9.1f", outAxes[i] );
			}
			//printf ( "\r");
			printf ( "\n");
            t0=t;
        }
    }
};



class CtrlModule: public RFModule
{
protected:
    CtrlThread *thr;
    //Port        rpcPort;

public:
    CtrlModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

		int rateThread = 10;
		if (rf.findGroup("GENERAL").check("rateThread"))
		{
			rateThread = rf.findGroup("GENERAL").find("rateThread").asInt();
		}
		else
		{
			fprintf ( stderr, "rateThread option not found, assuming %d ms\n", rateThread);
		}
        thr=new CtrlThread(rateThread,rf);
        if (!thr->start())
        {
            delete thr;
            return false;
        }

		//rpcPort.open("/joystickCtrl/rpc");
        //attach(rpcPort);

        return true;
    }

    virtual bool close()
    {
        thr->stop();
        delete thr;

        //rpcPort.interrupt();
        //rpcPort.close();

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};



int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("joystickControl.ini");			  //overridden by --from parameter
    rf.setDefaultContext("../../app/joystickControl/conf");   //overridden by --context parameter
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        fprintf (stderr, "Options:\n");
        fprintf (stderr, "\tNo options at the moment\n");
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

	//SDL_JoystickEventState (SDL_ENABLE);
	// this will alter the behaviour of the event queue of the sdl system
	SDL_JoystickEventState ( SDL_QUERY );

    CtrlModule mod;

    return mod.runModule(rf);
}



