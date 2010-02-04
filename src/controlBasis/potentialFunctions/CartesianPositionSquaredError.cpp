// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "CartesianPositionSquaredError.h"
#include <yarp/math/Math.h>
#include <yarp/os/Network.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

void CB::CartesianPositionSquaredError::startPotentialFunction() {
    if(!connectedToInputs) {
        if(!connectToInputs()) {
            cout << "Couldn't connect to YARP input port in startPotentialFunction()..." << endl;
            return;
        }
    }
    running = true;
    start();     // mandatory start function
}

void CB::CartesianPositionSquaredError::stopPotentialFunction() {
    cout << "CartesianPositionSquaredError::stopPotentialFunction()" << endl;
    stop();     // mandatory stop function
}

bool CB::CartesianPositionSquaredError::updatePotentialFunction() {
    
    Bottle *b[2];
    double offset;
    bool ok = true;
    Vector diff(3);
 
    // get data from ports (should be more safety checks...)
    b[0] = inputPort[0].read(false);
    b[1] = inputPort[1].read(false);

    bool b0 = (b[0]==NULL);
    bool b1 = (b[1]==NULL);

    if( b0 || b1 ) {
        return ok;
    }
    
    offset = 1;
    if(size==0) {
        size = 3;
        input[0].resize(size);
        input[1].resize(size);
        gradient.resize(size);
        diff.resize(3);
        cout << "ConfigurationSquaredError setting size: " << size << endl;
    }
    
    cout << "ref     cur" << endl;
    for(int i=0; i<size; i++) {
        input[0][i] = b[0]->get(i+offset).asDouble();
        input[1][i] = b[1]->get(i+offset).asDouble();
        cout << input[1][i] << "    " << input[0][i] << endl;
    }
    cout << endl;

    diff = input[1] - input[0];
    gradient = -1.0*diff;
    potential = 0.5*dot(diff,diff);

    //cout << "CartesianPositionSquaredError Potential = " << potential << endl;
    return ok;

}

bool CB::CartesianPositionSquaredError::connectToInputs() {
    
    bool ok = true;

    cout << "CartesianSquaredError::connectToInputs():\n\t" <<
        inputName[0].c_str() << "\n\t" << inputName[1].c_str() << endl << endl;

    connectedToInputs = false;

    string posCurName = inputName[0] + "/data:o";
    string posRefName = inputName[1] + "/data:o";

    string prefixStr = "/cb/" + inputSpace;
    int s = prefixStr.size();
    string tmp0 = inputName[0];
    string tmp1 = inputName[1];
    tmp0.erase(0,s);
    tmp1.erase(0,s);

    string posCurNameIn = "/cb/cartesianposition/squared_error_pf" + tmp0 + ":i";
    string posRefNameIn = "/cb/cartesianposition/squared_error_pf" + tmp1 + ":i";

    cout << "CartesianPositionSquaredError::connectToInputs() -- opening current input port..." << endl;
    ok &= inputPort[0].open(posCurNameIn.c_str());
    if(!ok) {
        cout << "CartesianSquaredError::connectToInputs() -- failed opening current input port..." << endl;
        return ok;
    }

    cout << "CartesianPositionSquaredError::connectToInputs() -- opening reference input port..." << endl;
    ok &= inputPort[1].open(posRefNameIn.c_str());
    if(!ok) {
        cout << "ConfigurationSquaredError::connectToInputs() -- failed opening reference input port..." << endl;
        return ok;
    }

    //    Time::delay(0.1);
    cout << "CartesianSquaredError::connectToInputs() -- connecting:\n\t" << 
        posCurName.c_str() << " -> " << posCurNameIn.c_str() << "\n\t" << 
        posRefName.c_str() << " -> " << posRefNameIn.c_str() << endl << endl;

    ok &= Network::connect(posCurName.c_str(),posCurNameIn.c_str(),"udp");
    if(!ok) {
        cout << "CartesianSquaredError::connectToInputs() -- failed connecting to current input pors...\n\n\n" << endl;
        return ok;
    }
    ok &= Network::connect(posRefName.c_str(),posRefNameIn.c_str(),"udp");
    if(!ok) {
        cout << "CartesianSquaredError::connectToInputs() -- failed connecting to reference input pors...\n\n\n" << endl;
        return ok;
    }
    
    cout << "CartesianPositionSquaredError done connecting to YARP input ports..." << endl;

    connectedToInputs = true;
    return ok; 
}


