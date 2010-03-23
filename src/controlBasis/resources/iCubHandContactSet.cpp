// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "iCubHandContactSet.h"
#include <yarp/os/Network.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


void CB::iCubHandContactSet::startResource() {
    // if the resource hasnt connected to the YARP device, do it now
    if(!connectedToDevice) {
        if(!connectToDevice()) {
            cout << "Couldn't connect to YARP port in startResource()..." << endl;
            return;
        }
    }
    start();     // mandatory start function
}

void CB::iCubHandContactSet::stopResource() {
  stop();     // mandatory stop function
}

bool CB::iCubHandContactSet::connectToDevice() {

    // if this is the first time running, need to connec to the PolyDriver.
    bool ok;
    int idx;
    cout << "iCubHandContactSet() -- connecting to port..." << endl;
    connectedToDevice = false;

    string robot_prefix;
    if(simulationMode) {
        robot_prefix = "/icubSim";
    } else {
        robot_prefix = "/icub";
    }

    string iCubContactsPort = robot_prefix + "/touch";

    cout << "iCubHandContactSet::connectToDevice() -- connecting port for touch values..." << endl;
    ok &= Network::connect(iCubContactsPort.c_str(), inputPortName[0].c_str(),"udp");

    if(!ok) {
        cout << "YARPAttentionMechanismHeading::connectToAttentionMechanism() -- connecting to image coordinates port failed..." << endl;
        return ok;
    }

    // set the connected flag cause everything seems to be okay
    connectedToDevice = true;

    return ok; 

}

bool CB::iCubHandContactSet::updateResource() {

    Bottle *b;
    bool ok =true;
    int idx;

    // read the current configuration variables from the input port
    b = inputPort[0]->read(false);
    if(b!=NULL) {

        // order: [palm index middle ring little thumb]

        if(id==0) { // left hand
            values[0] = (double)(b->get(1).asDouble());
            values[1] = (double)(b->get(5).asDouble());
            values[2] = (double)(b->get(7).asDouble());
            values[3] = (double)(b->get(9).asDouble());
            values[4] = (double)(b->get(11).asDouble());
            values[5] = (double)(b->get(13).asDouble());
        } else { // right
            values[0] = (double)(b->get(3).asDouble());
            values[1] = (double)(b->get(15).asDouble());
            values[2] = (double)(b->get(17).asDouble());
            values[3] = (double)(b->get(19).asDouble());
            values[4] = (double)(b->get(21).asDouble());
            values[5] = (double)(b->get(23).asDouble());
        }

        /*
        cout << "Contacts[" << id << "]=( ";
        for(int i=0; i<6; i++) {            
            cout <<  values[i] << " ";
        }
        cout << ")" << endl;
        */

    } 


    return ok;
}

void CB::iCubHandContactSet::postData() {
            
    // prepare the output bottles to post information           
    yarp::os::Bottle &b0 = outputPort[0]->prepare();
    b0.clear();
    b0.addInt(size);
    for(int i = 0; i < values.size(); i++) {       
        b0.addDouble(values[i]);
    }
    
    // write the information to the ports
    outputPort[0]->write(); 

    
}

void CB::iCubHandContactSet::getInputData() {
}

