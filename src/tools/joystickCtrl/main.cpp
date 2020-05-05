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
The port /joystickCtrl/axis:o contains the axis output only (raw data).
The port /joystickCtrl/buttons:o contains the buttons output only (raw data).

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files
A description of the available configuration options can be 
found in the example files located under joystickControl 
context. 
 
\section tested_os_sec Tested OS
Windows, Linux

\author Marco Randazzo
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
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
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

#define PRINT_STATUS_PER 0.1
#define MAX_AXES         32

enum { JTYPE_UNDEF=-1, JTYPE_POLAR=0, JTYPE_CARTESIAN=1, JTYPE_CONSTANT=2, JTYPE_STRING=3 };

class CtrlThread: public PeriodicThread
{
    struct struct_jointProperties
    {
        int type; 
        int param[3];
        string param_s;
    };

protected:
    ResourceFinder       &rf;
    BufferedPort<Bottle> port_command;
    BufferedPort<Bottle> port_axis_only;
    BufferedPort<Bottle> port_buttons_only;
    bool                 silent;
    bool                 force_cfg;

    double defaultExecTime;
    double t0;
    int numAxes;
    int numBalls;
    int numHats;
    int numButtons;
    int joy_id;
    SDL_Joystick* joy1;
    
    //button actions;
    string button_actions [20];

    //hats actions;
    string hat_actions [20];

    //variables read from the joystick
    int*    rawButtons;
    int*    rawButtonsOld;
    int*    rawHats;
    double* rawAxes;
    double* outAxes;

    //variables read from the configuration file
    int  num_inputs;
    int  num_outputs;
    double* inputMax;
    double* inputMin;
    double* outputMax;
    double* outputMin;
    double* jointDeadband;
    int*    reverse;
    struct_jointProperties* jointProperties;

public:
    CtrlThread(unsigned int _period, ResourceFinder &_rf) :
               PeriodicThread((double)_period/1000.0), rf(_rf)
    {
        joy_id=0;
        rawButtons=0;
        rawButtonsOld=0;
        rawAxes=0;
        rawHats=0;
        outAxes=0;
        joy1=0;
        num_inputs=0;
        num_outputs=0;
        inputMax=0;
        inputMin=0;
        outputMax=0;
        outputMin=0;
        jointDeadband=0;
        reverse=0;
        jointProperties=0;
        silent = false;
        force_cfg = false;
    }

    virtual bool threadInit()
    {
        // get params from the RF
        if (rf.check("silent"))
        {
            yInfo ("Running in silent mode\n");
            silent=true;
        }

        if (rf.check("force_configuration"))
        {
            yInfo ( "Force configuration option found\n");
            force_cfg=true;
        }

        if (rf.findGroup("INPUTS").check("InputsNumber"))
        {
            num_inputs = rf.findGroup("INPUTS").find("InputsNumber").asInt();
            inputMax = new double [num_inputs];
            inputMin = new double [num_inputs];
            outputMax = new double [num_inputs];
            outputMin = new double [num_inputs];
            jointDeadband = new double [num_inputs];
            reverse = new int [num_inputs];
            yDebug ("Number of input axes in the configuration options: %d \n",num_inputs);
        }
        else
        {
            yError ("Unable to find number of input axes in the configuration options\n");
            return false;
        }

        Bottle b;
        

        b = rf.findGroup("INPUTS").findGroup("Reverse");
        if (b.size()-1 == num_inputs)
            {
                for (int i = 1; i < b.size(); i++) reverse[i-1] = b.get(i).asInt();
            }
        else {yError ( "Configuration error: invalid number of entries 'Reverse'\n"); return false;}
        b = rf.findGroup("INPUTS").findGroup("InputMax");
        if (b.size()-1 == num_inputs)
            {
                for (int i = 1; i < b.size(); i++) inputMax[i-1] = b.get(i).asDouble();
            }
        else {yError ( "Configuration error: invalid number of entries 'InputMax'\n"); return false;}
        b = rf.findGroup("INPUTS").findGroup("InputMin");
        if (b.size()-1 == num_inputs)
            {
                 for (int i = 1; i < b.size(); i++) inputMin[i-1] = b.get(i).asDouble();
            }
        else {yError ( "Configuration error: invalid number of entries 'InputMin'\n"); return false;}
        b = rf.findGroup("INPUTS").findGroup("OutputMax");
        if (b.size()-1 == num_inputs)
            {
                for (int i = 1; i < b.size(); i++) outputMax[i-1] = b.get(i).asDouble();
            }
        else {yError ( "Configuration error: invalid number of entries 'OutputMax'\n"); return false;}
        b = rf.findGroup("INPUTS").findGroup("OutputMin");
        if (b.size()-1 == num_inputs)
            {
                for (int i = 1; i < b.size(); i++) outputMin[i-1] = b.get(i).asDouble();
            }
        else {yError ( "Configuration error: invalid number of entries 'OutputMin'\n"); return false;}

        b = rf.findGroup("INPUTS").findGroup("Deadband");
        if (b.size()-1 == num_inputs)
            {
                for (int i = 1; i < b.size(); i++) jointDeadband[i-1] = b.get(i).asDouble();
            }
        else {yError ( "Configuration error: invalid number of entries 'Deadband'\n"); return false;}

        if (rf.findGroup("OUTPUTS").check("OutputsNumber"))
        {
            num_outputs = rf.findGroup("OUTPUTS").find("OutputsNumber").asInt();
            jointProperties = new struct_jointProperties [num_outputs];
        }
        else
        {
            yError ( "Unable to find number of output axes in the configuration options\n");
            return false;
        }

        for (int i=0; i<num_outputs; i++)
        {
            char tmp [20];
            sprintf (tmp,"Ax%d",i);
            if (!rf.findGroup("OUTPUTS").check(tmp))
            {
                yError ( "Error reading [OUTPUT] block, unable to find Ax%d identifier\n",i);
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
            if (b.get(1).asString()=="string")
            {
                jointProperties[i].type=JTYPE_STRING;
                jointProperties[i].param[0]=0;
                jointProperties[i].param[1]=0;
                jointProperties[i].param[2]=0;
                jointProperties[i].param_s=b.get(2).asString();
            }
            else
            {
                yError() << "Unknown [OUTPUT] property";
                return false;
            }
        }
        //string s = b.toString(); //this causes compilation issues under linux

        // open the output port
        string output_port_name;
        if (rf.findGroup("GENERAL").check("outputPortName"))
        {
            output_port_name = rf.findGroup("GENERAL").find("outputPortName").asString();
        }
        else
        {
            output_port_name = "/joystickCtrl:o";
            yWarning ( "outputPortName not found, using %s \n", output_port_name.c_str());
        }
        bool ret=true;
        ret &= port_command.open(output_port_name.c_str());
        //@@@ TO BE COMPLETED: port name prefix to be set in the ini file
        ret &= port_axis_only.open("/joystickCtrl/raw_axis:o");
        ret &= port_buttons_only.open("/joystickCtrl/raw_buttons:o");
        if (ret==false)
        {
            yError() << "Unable to open module ports";
            return false;
        }

        //get the list of the commands to be executed with the buttons
        Bottle& exec_comm_bottle = rf.findGroup("BUTTONS_EXECUTE");
        int joystick_actions_count = 0;
        if (!exec_comm_bottle.isNull())
        {
            yInfo ( "associating the following actions to the buttons: \n");
            do
            {
                char tmp[80];
                sprintf(tmp, "button%d", joystick_actions_count); 
                if (exec_comm_bottle.check(tmp))
                {
                    button_actions[joystick_actions_count] = exec_comm_bottle.find(tmp).toString();
                    printf ("%s %s\n", tmp, button_actions[joystick_actions_count].c_str());
                }
                else
                {
                    break;
                }
                joystick_actions_count++;
            }
            while (joystick_actions_count<20);
            printf ("\n");
        }
        if (joystick_actions_count==0)
        {
            yInfo ( "no actions specified for the joystick buttons. \n");
        }


        //get the list of the commands to be executed with the hats
        Bottle& hats_exec_bottle = rf.findGroup("HATS_EXECUTE");
        int hats_actions_count = 0;
        if (!hats_exec_bottle.isNull())
        {
            yInfo ( "associating the following actions to the hats: \n");
            do
            {
                char tmp[80];
                sprintf(tmp, "hat%d", hats_actions_count);
                if (hats_exec_bottle.check(tmp))
                {
                    hat_actions[hats_actions_count] = hats_exec_bottle.find(tmp).toString();
                    printf ("%s %s\n", tmp, hat_actions[hats_actions_count].c_str());
                }
                else
                {
                    break;
                }
                hats_actions_count++;
            }
            while (hats_actions_count<20);
            printf ("\n");
        }
        if (joystick_actions_count==0)
        {
            yInfo ( "no actions specified for the joystick hats. \n");
        }


        // start SDL subsystem
        //SDL_Init(SDL_INIT_VIDEO);
        //SDL_SetVideoMode(640, 480, 16, SDL_DOUBLEBUF);
        if ( SDL_InitSubSystem ( SDL_INIT_JOYSTICK ) < 0 )
        {
            yError ( "Unable to initialize Joystick: %s\n", SDL_GetError() );
            return false;
        }

        // get the list of available joysticks
        fprintf ( stderr, "\n");
        int joy_id=0;
        int joystick_num = SDL_NumJoysticks ();
        if (joystick_num == 0)
        {
            yError ( "Error: No joysticks found\n"); return false;
        }
        else if (joystick_num == 1)
        {
            joy_id=0;
            yInfo ( "One joystick found \n");
#if (SDL_MAJOR_VERSION == 2)
            yInfo ( "Using joystick: %s \n", SDL_JoystickNameForIndex(joy_id));
#else
            yInfo ( "Using joystick: %s \n", SDL_JoystickName(joy_id));
#endif

        }
        else
        {
            yInfo ( "More than one joystick found:\n");
            for (int i=0; i<joystick_num; i++)
            {
#if (SDL_MAJOR_VERSION == 2)
                yInfo ( "%d: %s\n",i,SDL_JoystickNameForIndex(i));
#else
                yInfo ( "%d: %s\n",i,SDL_JoystickName(i));
#endif
            }
            yInfo ( "\n");

            // choose between multiple joysticks
            if (rf.findGroup("GENERAL").check("DefaultJoystickNumber"))
            {
                joy_id = rf.findGroup("GENERAL").find("DefaultJoystickNumber").asInt();
                yInfo ( "Multiple joysticks found, using #%d, as specified in the configuration options\n", joy_id);
            }
            else
            {
                yWarning ( "No default joystick specified in the configuration options\n");
                yWarning ( "Which joystick you want to use? (choose number) \n");
                cin >> joy_id;
            }
        }

        // Open the Joystick driver
        joy1 = SDL_JoystickOpen ( joy_id );
        if ( joy1 == NULL )
        {
            yError ( "Could not open joystick\n" );
            return false;
        }

        // Obtaining Joystick capabilities
        numAxes = SDL_JoystickNumAxes ( joy1 );
        numBalls = SDL_JoystickNumBalls ( joy1 );
        numHats = SDL_JoystickNumHats ( joy1 );
        numButtons = SDL_JoystickNumButtons ( joy1 );
        yInfo (  "Characteristics of joy %d: \n", joy_id);
        yInfo (  "%i Axes\n", numAxes );
        yInfo (  "%i Balls\n", numBalls );
        yInfo (  "%i Hats\n",  numHats );
        yInfo (  "%i Buttons\n", numButtons );
        yInfo (  "\n");

        // check: selected joint MUST have at least one axis
        if (numAxes<=0) 
        {
            yError ( "Error: selected joystick has %d Axes?!\n",numAxes );
            return false;
        }

        if (numAxes!=num_inputs)
        {
            if (force_cfg == false)
            {
                yWarning (  "Warning: # of joystick axes (%d) differs from # of configured input axes (%d)!\n",numAxes,num_inputs );
                yWarning (  "This probably means that your .ini file does not containt a correct configuration.\n");
                yWarning (  "Do you want to continue anyway (y/n)?\n");
                char input[255];
                cin >> input;
                if (input[0]!='y' && input[0]!='Y') 
                {
                    yInfo ( "Quitting...\n");
                    return false;
                }
                else
                {
                    yWarning ( "Overriding the number of axes specified in the configuration file. Using %d axes.\n",numAxes);
                }
            }
            else
            {
                yWarning ( "Warning: # of joystick axes (%d) differs from # of configured input axes (%d)!\n",numAxes,num_inputs );
                yWarning ( "This probably means that your .ini file does not containt a correct configuration.\n");
                yWarning ( "However, --force_configuration option is enabled. This will override the number of axes specified in the configuration file.\n");
                yWarning ( "Using %d axes.\n",numAxes);
            }
        }

        rawAxes       = new double [MAX_AXES];
        rawHats       = new int    [MAX_AXES];
        rawButtons    = new int    [MAX_AXES];
        rawButtonsOld = new int    [MAX_AXES];
        outAxes       = new double [MAX_AXES];

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
            yInfo ( "Thread started successfully\n");
        else
            yError ( "Thread did not start\n");

        t0=Time::now();
    }

    virtual void run()
    {
         //check if driver is connected
        /*
        if (joy_id>SDL_NumJoysticks())
        {
            fprintf ( stderr, "Lost connection to driver!\n");
        }
        fprintf ( stderr, "%d\n",SDL_NumJoysticks());
        */

        // Updates the joystick status
        SDL_JoystickUpdate ();

        // Reading joystick data (axes/buttons...)
        for ( int i=0; i < numButtons; ++i )
        {
            rawButtonsOld[i] = rawButtons[i];
            rawButtons[i] = SDL_JoystickGetButton ( joy1, i );
        }

        for ( int i=0; i < numHats; ++i )
        {
            rawHats[i] = SDL_JoystickGetHat ( joy1, i );
        }

        for ( int i=0; i < numAxes; ++i )
        {
            rawAxes[i] = (double)SDL_JoystickGetAxis ( joy1, i );
        }

        // Formatting input data
        for(int i=0;i<num_inputs;i++)
        {
            double v = rawAxes[i];
            if (jointDeadband[i]>0)
            {
                if (fabs(v)<jointDeadband[i]) v=0;
            }
            if (reverse[i]==1)
                v=-v;

            if (inputMax[i]==inputMin[i])
            {
                v = 0;
            }
            else
            {
                v = (v<inputMax[i]) ? v : inputMax[i];
                v = (v>inputMin[i]) ? v : inputMin[i];
                v = v - ((inputMax[i]-inputMin[i])/2+inputMin[i]);
                v = v / (inputMax[i]-inputMin[i]);
                v = v * (outputMax[i]-outputMin[i]);
                v = v + ((outputMax[i]-outputMin[i])/2+outputMin[i]);
            }

            /*
            v = v+jointOffset[i];
            v = v*jointGain[i];
            v = (v<jointMax[i]) ? v : jointMax[i];
            v = (v>jointMin[i]) ? v : jointMin[i];
            */

            rawAxes[i]=v;
        }

        // Sending data out on a Yarp port
        Bottle data;
        Bottle axis_data;
        Bottle buttons_data;
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
                    yWarning ( "Unknown parameter for JTYPE_POLAR, joint %d\n",i);
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
                yWarning ( "Unknown property, joint %d\n",i);
            }
        }

        //execute button actions
        for (int i=0;i<numButtons;i++)
        {
            if (rawButtonsOld[i] == 0 && rawButtons[i] == 1)
            {
                //execute script
                if (!button_actions[i].empty())
                {
                    yInfo ("executing script %d: %s\n", i, button_actions[i].c_str());
                    int ret = system(button_actions[i].c_str());
                }
                else
                {
                    yWarning ("no scripts associated to button %d\n", i);
                }
            }
        }

        //execute button actions
        for (int i=0;i<numHats;i++)
        {
            if (rawHats[i] != SDL_HAT_CENTERED)
            {
                //execute script
                if (!hat_actions[i].empty())
                {
                    string action(hat_actions[i]);
                    action += " ";
                    switch(rawHats[i]) {
                    case SDL_HAT_UP:
                        action += "up";
                        break;
                    case SDL_HAT_RIGHT:
                        action += "right";
                        break;
                    case SDL_HAT_DOWN:
                        action += "down";
                        break;
                    case SDL_HAT_LEFT:
                        action += "left";
                        break;
                    case SDL_HAT_RIGHTUP:
                        action += "rightup";
                        break;
                    case SDL_HAT_RIGHTDOWN:
                        action += "rightdown";
                        break;
                    case SDL_HAT_LEFTUP:
                        action += "leftup";
                        break;
                    case SDL_HAT_LEFTDOWN:
                        action += "leftdown";
                        break;
                    default:
                        break;
                    }
                    yInfo ("executing script %d: %s\n", i, action.c_str());
                    int ret = system(action.c_str());
                }
                else
                {
                    yWarning ("no scripts associated to button %d\n", i);
                }
            }
        }

        // Preparing data to be sent on the yarp ports
        for(int i=0;i<num_outputs;i++)
        {    
            if ( jointProperties[i].type == JTYPE_STRING)
                data.addString(jointProperties[i].param_s.c_str());
            else
                data.addDouble(outAxes[i]);
        }
        for (int i=0;i<numButtons;i++)
        {
            buttons_data.addDouble(rawButtons[i]);
        }
        for (int i=0;i<numAxes; i++)
        {
            axis_data.addDouble(rawAxes[i]);
        }

        // Sending data on the yarp ports
        if (port_command.getOutputCount()>0)
        {
            port_command.prepare() = data;
            port_command.write();
        }
        if (port_axis_only.getOutputCount()>0)
        {
            port_axis_only.prepare() = axis_data;
            port_axis_only.write();
        }
        if (port_buttons_only.getOutputCount()>0)
        {
            port_buttons_only.prepare() = buttons_data;
            port_buttons_only.write();
        }

        // Displaying status
        if (!silent) printStatus();
    }

    virtual void threadRelease()
    {    
        if (rawAxes)         delete [] rawAxes;
        if (rawHats)         delete [] rawHats;
        if (outAxes)         delete [] outAxes;
        if (rawButtons)      delete [] rawButtons;
        if (inputMax)        delete [] inputMax;
        if (inputMin)        delete [] inputMin;
        if (outputMax)       delete [] outputMax;
        if (outputMin)       delete [] outputMin;
        if (jointDeadband)   delete [] jointDeadband;
        if (jointProperties) delete [] jointProperties;
        if (reverse)         delete [] reverse;
        port_command.interrupt();
        port_command.close();
        port_axis_only.interrupt();
        port_axis_only.close();
        port_buttons_only.interrupt();
        port_buttons_only.close();
    }


    void printStatus()
    {
        double t=Time::now();
        static char buff [1000];
        buff[0] = 0;

        if (t-t0>=PRINT_STATUS_PER)
        {
            //for (int i=0;i <numButtons; i++)
            //    sprintf (buff, "%+6d", rawButtons[i] );

            for (int i=0;i <numAxes; i++)
            {
                sprintf(buff + strlen(buff), "%+9.1f", rawAxes[i]);
            }
            sprintf(buff + strlen(buff), " ---> ");
            for (int i=0;i <num_outputs; i++)
            {
                sprintf(buff + strlen(buff), "%+9.1f", outAxes[i]);
            }
            yDebug() << buff;
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
        int rateThread = 10;
        if (rf.findGroup("GENERAL").check("rateThread"))
        {
            rateThread = rf.findGroup("GENERAL").find("rateThread").asInt();
        }
        else
        {
            yWarning ("rateThread option not found, assuming %d ms\n", rateThread);
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
    rf.setDefaultConfigFile("joystickControl.ini");     //overridden by --from parameter
    rf.setDefaultContext("joystickControl");            //overridden by --context parameter
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yInfo ( "Options:\n");
        yInfo ( "--silent: supress text output\n");
        yInfo ( "--force_configuration: force a joystick configuration for a joystick with differnt # of axes, buttons etc.\n");
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        yError("Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    //SDL_JoystickEventState (SDL_ENABLE);
    // this will alter the behaviour of the event queue of the sdl system
    SDL_JoystickEventState ( SDL_QUERY );

    CtrlModule mod;

    return mod.runModule(rf);
}



