/**
*
@ingroup icub_tools
\defgroup icub_portsMerge portsMerge

Merge the data (bottles) of multiple ports into one single port.

\section intro_sec Description
The user has to specifies a list of streaming ports (i0, i1, ..., iN) that he wants to merge in a single output port (o0).
The module creates a corresponding number of input ports and automatically makes the connections.
The values read from the input ports are merged and synchronized into the output port according to the following rules:
- the data are appended in the ouput vector using the same order in which the input ports are specified by the user, i.e. o0 = [i0 | i1 | ... | iN].
- the timestamp of the output port o0 is assigned by the module.
- if in a time instant no data is received from ALL the input ports (i0, i1, ..., iN), no data is sent on the output port (o0).
- if in a time instant no data is received from SOME ports (e.g. iJ), the output vector will be o0 = [i0 | i1 | ... |iJ-1| ... |in], i.e. the previous sample will be used.
The output of the module can be logged in order to obtain a quick log of multiple data streams on a single file.

\section lib_sec Libraries
YARP libs.

\section parameters_sec Parameters
No parameters.

\section portsa_sec Ports Accessed
The module automatically connects to the ports passed as argument to command line.

\section portsc_sec Ports Created
The module creates multiple input ports to receive data, and one outport port to produce the merged result.

Output ports:
- /portsMerge/o0 the output port

Input ports:
- /portsMerge/i* the input ports (*=1,2...n)

\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module

Just run:

\code
portsMerge /icub/left_arm/state:o /icub/left_arm/analog:o
\endcode

the output of the module can be logged on a file:

\code
yarp read ... /portsMerge/o0 envelope &> logfile.txt
\endcode

\author Marco Randazzo

Copyright (C)2012  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/portsMerge/main.cpp
**/

#include <yarp/sig/Image.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/BufferedPort.h>

#include <string>
#include <iostream>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

int main(int argc, char *argv[])
{
    Network yarp;

    BufferedPort<Bottle >   outPort;
    BufferedPort<Bottle >*  inPort  = 0;
    Bottle*                 inData  = 0;
    yarp::os::Stamp*        inStamp = 0;
    yarp::os::Stamp         outStamp;

    int nPorts = argc-1;
    if (nPorts == 0)
    {
        printf ("No input ports specified! \n");
        return -1;
    }

    inPort  = new BufferedPort<Bottle > [nPorts];
    inData  = new Bottle                [nPorts];
    inStamp = new yarp::os::Stamp       [nPorts];

    //open the ports
    char buff[255];
    string s = "/portsMerge";
    for (int i = 0; i< nPorts; i++)
    {
        sprintf(buff,"%s/i%d", s.c_str(), i);
        inPort[i].open(buff);
    }
    sprintf(buff,"%s/o0", s.c_str());
    outPort.open(buff);

    //makes the connection
    for (int i=1; i<argc; i++)
    {
        string tmp = argv[i];
        bool b = yarp::os::Network::connect(tmp.c_str(),inPort[i-1].getName().c_str(),"udp",false);     
        if (b == false) return -1;
    }

    printf ("Module running\n");
    while(true)
    {
        //read
        int empty =0;
        for (int i = 0; i< nPorts; i++)
        {       
            Bottle *b = inPort[i].read(false);
            if (b!=0) 
            {
                inPort[i].getEnvelope(inStamp[i]);
                inData[i] = *b;
            }
            else
            {
                empty++;
            }
        }
        if (empty == nPorts) 
        {
            yarp::os::Time::delay(0.001);
            continue;
        }

        //write
        outStamp.update();
        if (outPort.getOutputCount()>0)
        {       
            Bottle &out=outPort.prepare();
            out.clear();
            for (int i = 0; i< nPorts; i++)
            {
                out.append(inData[i]);
            }
            outPort.setEnvelope(outStamp);
            outPort.write();
        }
        
        //delay
        yarp::os::Time::delay(0.001);
    }

    delete [] inPort  ;
    delete [] inData  ;
    delete [] inStamp ;
    return 0;
}


