// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "YARPAttentionMechanismStereoHeading.h"
#include <yarp/os/Network.h>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

void CB::YARPAttentionMechanismStereoHeading::startResource() {

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

void CB::YARPAttentionMechanismStereoHeading::stopResource() {
  stop();     // mandatory stop function
  estimate.zero();
}

bool CB::YARPAttentionMechanismStereoHeading::updateResource() {
  
    Bottle *b[2];
    int nj;
    int offset;
    bool ok = true;
    int idx = 0;
    Vector sample(4);
    double focal_length = 0.01;

    // read the current configuration variables from the input ports
    for(int i=0; i<2; i++) {            
        b[i] = inputPort[i]->read(false);
        if(b[i]!=NULL) {       
            for(int k=0; k<2; k++) {            
                imageCoordinates[i][k] = b[i]->get(k).asDouble();
            }
        } else {
            ok = false;
        }
    }

    // turn uv coordinates into a filtered heading
    for(int i=0; i<4; i++) {            
        sample[i] = atan2(imageCoordinates[(int)(i/2)][i%2],focal_length);
        estimate[i] = estimate[i] + alpha*sample[i];
        values[i] = estimate[i];
    }
    cout << "StereoHeading: LEFT=[" << values[0] << " " << values[1] << "]" << ", RIGHT=[" << values[2] << " " << values[3] << "]" << endl;
    
    return ok;
    
}

bool CB::YARPAttentionMechanismStereoHeading::connectToAttentionMechanism() {
    
    bool ok = true;

    if(inputPort.size() != numInputs) {
        return false;
    }    

    // connect to config port for reading config value
    string leftCoordinatesOutputName  = "/blobFinder/"+deviceName+"/left_cam/triangulation:o";
    string rightCoordinatesOutputName = "/blobFinder/"+deviceName+"/right_cam/triangulation:o";

    cout << "YARPAttentionMechanismStereoHeading::connectToAttentionMechanism() -- connecting ports for left image coordinates..." << endl;
    ok &= Network::connect(leftCoordinatesOutputName.c_str(),inputPortName[0].c_str(),"udp");
    if(!ok) {
        cout << "YARPAttentionMechanismStereoHeading::connectToAttentionMechanism() -- connecting to left image coordinates port failed..." << endl;
        return ok;
    }

    cout << "YARPAttentionMechanismStereoHeading::connectToAttentionMechanism() -- connecting ports for right image coordinates..." << endl;
    ok &= Network::connect(rightCoordinatesOutputName.c_str(),inputPortName[1].c_str(),"udp");
    if(!ok) {
        cout << "YARPAttentionMechanismStereoHeading::connectToAttentionMechanism() -- connecting to right image coordinates port failed..." << endl;
        return ok;
    }

    connectedToAttentionMechanism = true;
   
    return ok;
}
