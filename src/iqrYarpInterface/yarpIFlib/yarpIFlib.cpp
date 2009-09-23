#include "yarpIFlib.hpp"

#include <iostream>
#include <iomanip>

#include <yarp/os/Network.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarpIF;

void openYarpNetwork() {
     Network::init();
}

void closeYarpNetwork() {
     Network::fini();
}

iCubLimbIF::iCubLimbIF() {
    nAxes=0;
    maxValueCell=1.0;
    configured=false;
    verbose=false;
}

bool iCubLimbIF::configure(Property& options) {

    if (!options.check("limb") || !options.check("local"))
        return false;
   
    Property optDrv("(device remote_controlboard)");

    string remoteName="/icub/";
    remoteName+=options.find("limb").asString().c_str();

    optDrv.put("remote",remoteName.c_str());
    optDrv.put("local",options.find("local").asString().c_str());

    if (options.check("maxValueCell"))
        maxValueCell=options.find("maxValueCell").asDouble();

    drv=new PolyDriver(optDrv);

    if (drv->isValid())
    {
        drv->view(lim);
    	drv->view(enc);
	drv->view(pos);
        drv->view(vel);

        enc->getAxes(&nAxes);

        min.resize(nAxes);
        max.resize(nAxes);

        for (int i=0; i<nAxes; i++)
        {
            lim->getLimits(i,&min[i],&max[i]);
            
            if (verbose)
                cout<<"jnt #"<<i<<" in ["<<min[i]<<","<<max[i]<<"]"<<endl;
        }

        return configured=true;
    }
    else
    {
        cerr << "Argh: device driver not available!" << endl;
        return false;
    }
}

bool iCubLimbIF::readEncs(Vector& encs) {
    if (configured)
    {
        enc->getEncoders(encs.data());
        encs=scaleFromRobot(encs);
        return true;
    }
    else
        return false;
}

bool iCubLimbIF::setRefSpeeds(const Vector& vels) {
    if (configured)
    {
        pos->setRefSpeeds(vels.data());
        return true;
    }
    else
        return false;
}

bool iCubLimbIF::writePosCmds(const Vector& cmds) {
    if (configured)
    {
        Vector _cmds=scaleToRobot(cmds);
        pos->positionMove(_cmds.data());
        return true;
    }
    else
        return false;
}

bool iCubLimbIF::writeVelCmds(const Vector& vels) {
    if (configured)
    {
        Vector _vels=scaleToRobot(vels);
        vel->velocityMove(_vels.data());
        return true;
    }
    else
        return false;
}

Vector iCubLimbIF::scaleFromRobot(const Vector& v) {
    Vector ret(nAxes);
    
    for (int i=0; i<nAxes; i++)
    {
        ret[i]=(v[i]-min[i])/(max[i]-min[i]);
        ret[i]=ret[i]<0.0?0.0:ret[i];
        ret[i]=ret[i]>maxValueCell?maxValueCell:ret[i];
    }    

    return ret;
}

Vector iCubLimbIF::scaleToRobot(const Vector& v) {
    Vector ret(nAxes);
    
    for (int i=0; i<nAxes; i++)
        ret[i]=v[i]*(max[i]-min[i])+min[i];   

    return ret;
}

iCubLimbIF::~iCubLimbIF(){
    delete drv;
}


