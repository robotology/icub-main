/** 
\defgroup eye2RootFrameTransformer eye2RootFrameTransformer
 
@ingroup icub_module  
 
A kinematic translator which returns the equivalent 3-d position
wrt the root frame of the robot starting from a given 3-d 
position in the frame attached to the eye.

Copyright (C) 2009 RobotCub Consortium
 
Author: Matteo Taiana

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
... 
 
\section lib_sec Libraries 
... 
 
\section parameters_sec Parameters
... 
 
\section portsa_sec Ports Accessed
... 
 
\section portsc_sec Ports Created 
... 
 
\section in_files_sec Input Data Files
...

\section out_data_sec Output Data Files 
...

\section tested_os_sec Tested OS
...

\author Matteo Taiana
*/ 

#include <yarp/os/Network.h>
#include <yarp/math/Math.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>

#include <iostream>
#include <iomanip>
#include <string>

#include <iCub/eye2RootFrameTransformer.hpp>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iKin;


// member function that set the object up.
bool eye2RootFrameTransformer::configure(ResourceFinder &rf)
{
    //***********************************
    //Read options from the command line.
    //***********************************
    string eyeType=rf.find("eye").asString().c_str();
    string name=rf.find("name").asString().c_str();
    string stemNamePort="/"+name;

    _inputHeadPort.open((stemNamePort+"/head:i").c_str());
    _inputTorsoPort.open((stemNamePort+"/torso:i").c_str());
    _inputTargetPosPort.open((stemNamePort+"/targetPos:i").c_str());
    _outputTargetPosPort.open((stemNamePort+"/targetPos:o").c_str());
    
    eye = new iCubEye(eyeType);
    cout<<"Working with the "<<eye->getType()<<" eye"<<endl;

    eye->releaseLink(0);
    eye->releaseLink(1);
    eye->releaseLink(2);
    cout<<"eye Degrees Of Freedom = "<<eye->getDOF()<<endl;

    chainEye=*(eye->asChain());
    receivedHead = false;
    receivedTorso = false;
    receivedTargetPos = false;
    isLeftEye=eye->getType()=="left";

    v.resize(8);
    eyeTargetPos.resize(4);
    rootTargetPos.resize(4);
    transformation.resize(4,4);

    return true;
}

// member that closes the object.
bool eye2RootFrameTransformer::close()
{
    delete eye;
    return true;
}

// member that is repeatedly called by YARP, to give this object the chance to do something.
// should this function return "false", the object would be terminated.
// I already have one image, when I get here (I either acquire it in the initialization method or in the end of this same method).
bool eye2RootFrameTransformer::updateModule()
{
    if (Bottle *head=_inputHeadPort.read(false))
    {
        //get data and convert from degrees to radians
        head0=head->get(0).asDouble()*M_PI/180;
        head1=head->get(1).asDouble()*M_PI/180;
        head2=head->get(2).asDouble()*M_PI/180;
        head3=head->get(3).asDouble()*M_PI/180;
        head4=head->get(4).asDouble()*M_PI/180;
        head5=head->get(5).asDouble()*M_PI/180;
        receivedHead = true;
    }

    if (Bottle *torso=_inputTorsoPort.read(false))
    {
        //get data and convert from degrees to radians
        torso0=torso->get(0).asDouble()*M_PI/180;
        torso1=torso->get(1).asDouble()*M_PI/180;
        torso2=torso->get(2).asDouble()*M_PI/180;
        receivedTorso = true;
    }

    if (Bottle *targetPosIn=_inputTargetPosPort.read(false))
    {
        //get the data, already in meters
        targetPosInX=targetPosIn->get(0).asDouble();
        targetPosInY=targetPosIn->get(1).asDouble();
        targetPosInZ=targetPosIn->get(2).asDouble();
        targetPosInGood=targetPosIn->get(6).asDouble();

        if ( gsl_isnan(targetPosInX) || gsl_isnan(targetPosInY) || gsl_isnan(targetPosInZ))
            targetPosInGood=0;

        receivedTargetPos = true;
    }

    if (receivedHead && receivedTorso && receivedTargetPos)
    {
        //DO THE PROCESSING AND THE OUTPUT

        v[0]=torso2;
        v[1]=torso1;
        v[2]=torso0;
        v[3]=head0;
        v[4]=head1;
        v[5]=head2;
        v[6]=head3;
        if (isLeftEye)
            v[7]=head4+head5/2;
        else
            v[7]=head4-head5/2;

        //Matrix transformation;
        transformation = chainEye.getH(v);

        eyeTargetPos[0]=targetPosInX;
        eyeTargetPos[1]=targetPosInY;
        eyeTargetPos[2]=targetPosInZ;
        eyeTargetPos[3]=1;

        rootTargetPos=transformation*eyeTargetPos;

        //send the data on the output port, ONLY if likelihood is over threshold (flag==1)
        if (targetPosInGood==1)
        {
            Bottle& output=_outputTargetPosPort.prepare();
            output.clear();
            output.addDouble(rootTargetPos[0]);
            output.addDouble(rootTargetPos[1]);
            output.addDouble(rootTargetPos[2]);

            _outputTargetPosPort.write();
        }
    }

    return true;
}


double eye2RootFrameTransformer::getPeriod()
{
    return 0.03;
}


int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
        return -1;

    ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefault("eye","left");
    rf.setDefault("name","eye2RootFrameTransformer");
    rf.configure("ICUB_ROOT",argc,argv);

    eye2RootFrameTransformer moduleID;
    moduleID.setName(rf.find("name").asString().c_str());

    return moduleID.runModule(rf);
}



