// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "YARPAttentionMechanismHeading.h"
#include <yarp/os/Network.h>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

void CB::YARPAttentionMechanismHeading::startResource() {

    // if not connected to the configuration resource, do that now
    if(!connectedToAttentionMechanism) {
        if(!connectToAttentionMechanism()) {
            cout << "YARP Atten. Mechanism couldn't connect to ports in startResource()..." << endl;
            return;
        }
    }
    running = true;
    start();     // mandatory start function
}

void CB::YARPAttentionMechanismHeading::stopResource() {
  stop();     // mandatory stop function
  estimate.zero();
}

bool CB::YARPAttentionMechanismHeading::updateResource() {
  
    Bottle *b;
    int nj;
    int offset;
    bool ok = true;
    int idx = 0;
    Vector sample(2);
    double focal_length = 0.01;

    // read the current configuration variables from the input port
    b = inputPort[0]->read(false);
    if(b!=NULL) {

        for(int i=0; i<2; i++) {            
            imageCoordinates[i] = b->get(i).asDouble();
        }

        // turn uv coordinates into a filtered heading
        for(int i=0; i<2; i++) {
            sample[i] = atan2(imageCoordinates[i],focal_length);
            estimate[i] = estimate[i] + alpha*sample[i];
            values[i] = estimate[i];
        }
        cout << "Heading=[" << values[0] << " " << values[1] << "]" << endl;
    } 

    return ok;
    
}

bool CB::YARPAttentionMechanismHeading::connectToAttentionMechanism() {
    
    bool ok = true;

    if(inputPort.size() != numInputs) {
        return false;
    }    

    // connect to config port for reading config values
    string coordinatesOutputName = "/blobFinder/"+deviceName+"/triangulation:o";

    cout << "YARPAttentionMechanismHeading::connectToAttentionMechanism() -- connecting ports for configuration values..." << endl;
    ok &= Network::connect(coordinatesOutputName.c_str(),inputPortName[0].c_str(),"udp");
    if(!ok) {
        cout << "YARPAttentionMechanismHeading::connectToAttentionMechanism() -- connecting to image_coordinates port falied..." << endl;
        return ok;
    }

    connectedToAttentionMechanism = true;
   
    return ok;
}
