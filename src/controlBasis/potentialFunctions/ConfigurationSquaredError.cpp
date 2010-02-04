// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "ConfigurationSquaredError.h"
#include <yarp/math/Math.h>
#include <yarp/os/Network.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

void CB::ConfigurationSquaredError::startPotentialFunction() {
    if(!connectedToInputs) {
        if(!connectToInputs()) {
            cout << "Couldn't connect to YARP input port in startPotentialFunction()..." << endl;
            return;
        }
    }
    running = true;
    start();     // mandatory start function
}

void CB::ConfigurationSquaredError::stopPotentialFunction() {
    cout << "ConfigurationSquaredError::stopPotentialFunction()" << endl;
    stop();     // mandatory stop function
}

bool CB::ConfigurationSquaredError::updatePotentialFunction() {
    
    cout << "ConfigurationSquaredError updating potential" << endl;
    
    Bottle *b[2];
    double offset;
    bool ok = true;
    Vector diff;

    // get data from ports (should be more safety checks...)
    b[0] = inputPort[0].read(false);
    b[1] = inputPort[1].read(false);
    
    if( (b[0]==NULL) || (b[1]==NULL) ) {
        cout << "ConfigurationSquaredError::update() problem reading data!!" << endl;
        if(size==0) {
            diff.resize(1);
            diff.zero();
        }
        return ok;
    }

    offset = 1;
    if(size==0) {
        size = b[0]->get(0).asInt();
        input[0].resize(size);
        input[1].resize(size);
        gradient.resize(size);
        diff.resize(size);
        cout << "ConfigurationSquaredError setting size: " << size << endl;
    }

    cout << "ref     cur" << endl;
    for(int i=0; i<size; i++) {
        input[0][i] = b[0]->get(i+offset).asDouble();
        input[1][i] = b[1]->get(i+offset).asDouble();
        cout << input[1][i] << "  " << input[0][i] << endl;
    }
    cout << endl;

    diff = input[1] - input[0];
    gradient = -1.0*diff;
    potential = 0.5*dot(diff,diff);

cout << "ConfigurationSquaredError Potential = " << potential << endl;
    return ok;

}

bool CB::ConfigurationSquaredError::connectToInputs() {
    
    bool ok = true;

    cout << "ConfigurationSquaredError::connectToInputs():\n\t" << inputName[0].c_str() << " \n\t" << inputName[1].c_str() << "\n\n\n"; 
  
    connectedToInputs = false;

    string configCurName = inputName[0] + "/data:o";
    string configRefName = inputName[1] + "/data:o";

    string prefixStr = "/cb/" + inputSpace;
    int s = prefixStr.size();
    string tmp0 = inputName[0];
    string tmp1 = inputName[1];
    tmp0.erase(0,s);
    tmp1.erase(0,s);
    
    string configCurNameIn = "/cb/configuration/squared_error_pf" + tmp0 + ":i";
    string configRefNameIn = "/cb/configuration/squared_error_pf" + tmp1 + ":i";

    cout << "ConfigurationSquaredError::connectToInputs() -- opening current input port..." << endl;
    ok &= inputPort[0].open(configCurNameIn.c_str());
    if(!ok) {
        cout << "ConfigurationSquaredError::connectToInputs() -- failed opening current input port..." << endl;
        return ok;
    }
    cout << "ConfigurationSquaredError::connectToInputs() -- opening reference input port..." << endl;
    ok &= inputPort[1].open(configRefNameIn.c_str());
    if(!ok) {
        cout << "ConfigurationSquaredError::connectToInputs() -- failed opening reference input port..." << endl;
        return ok;
    }

    Time::delay(1);
    cout << "ConfigurationSquaredError::connectToInputs() -- connecting:\n\t" << 
        configCurName.c_str() << " -> " << configCurNameIn.c_str() << "\n\t" << 
        configRefName.c_str() << " -> " << configRefNameIn.c_str() << endl << endl;
    
    ok &= Network::connect(configCurName.c_str(),configCurNameIn.c_str(), "udp");
    Time::delay(0.2);
    ok &= Network::connect(configRefName.c_str(),configRefNameIn.c_str(), "udp");
    if(!ok) {
        cout << "ConfigurationSquaredError::connectToInputs() -- failed connecting to input ports..." << endl << endl;
        return ok;
    }
    cout << "ConfiguartionSquaredError done connecting to YARP input ports..." << endl;
    connectedToInputs = true;
    return ok; 
}


