/**
*
* Library that controls the iCub facial expressions based on whether a tracker is tracking the object of interest or not.
* See \ref icub_trackerexpressions \endref
*
* Copyright (C) 2009 RobotCub Consortium
*
* Author: Matteo Taiana
*
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#include <highgui.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <time.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>

#include <iCub/trackerExpressions.hpp>




//constructor
TrackerExpressions::TrackerExpressions()
{
    ;
}



//destructor
TrackerExpressions::~TrackerExpressions()
{
    cout<<"oh my god! they killed kenny!    you bastards!\n";
}



//member function that set the object up.
bool TrackerExpressions::open(Searchable& config)
{

    //***********************************
    //Read options from the command line.
    //***********************************
    // pass configuration over to bottle
    Bottle botConfig(config.toString().c_str());
    botConfig.setMonitor(config.getMonitor()); //is this needed?

    _inputPortName = botConfig.check("inputPort",
                                       Value("/icub/trackerExpressions/in"),
                                       "Input port (string)").asString();
    _inputPort.open(_inputPortName);

    _outputPortName = botConfig.check("outputPort",
                                       Value("/icub/trackerExpressions/out"),
                                       "Output port (string)").asString();
    _outputPort.open(_outputPortName);

    _counter=0;
    return true;  //the object was set up successfully.
}



//member that closes the object.
bool TrackerExpressions::close()
{
    _inputPort.close();
    _outputPort.close();
    return true;
}

//member that closes the object.
bool TrackerExpressions::interruptModule()
{
    _inputPort.interrupt();
    _outputPort.interrupt();
    return true;
}


//member that is repeatedly called by YARP, to give this object the chance to do something.
//should this function return "false", the object would be terminated.
bool TrackerExpressions::updateModule()
{
     Bottle *tracker=_inputPort.read(false);

       if(tracker!=NULL)
       {
	  //get the flag that tells whether the likelihood is above the threshold.
	  if((tracker->get(6)).asInt()==1)
	  {
	   //cout<<"A\n";   
           Bottle& output=_outputPort.prepare();
           output.clear();
           output.fromString("set all hap");
           _outputPort.write();
           _counter=0;
	  }
	  else
	  {
	      
           _counter++;
	   if(_counter>10)
           {
	   //cout<<"B\n";   
           Bottle& output=_outputPort.prepare();
           output.clear();
           output.fromString("set all ang");
           _outputPort.write();
           
           }
	  }
       }
	//else
        //{
        //  cout<<"C\n";   
        //}


    yarp::os::Time::delay(0.05);
    return true; //continue. //in this case it means everything is fine.
}


