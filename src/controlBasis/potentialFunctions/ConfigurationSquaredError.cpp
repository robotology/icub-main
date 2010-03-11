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
    start();     // mandatory start function
}

void CB::ConfigurationSquaredError::stopPotentialFunction() {
    stop();     // mandatory stop function
}

bool CB::ConfigurationSquaredError::updatePotentialFunction() {
       
    Bottle *b[2];
    double offset;
    bool ok = true;
    Vector diff;

    // get data from ports (should be more safety checks...)
    if(inputPorts.size() != 2) {
        cout << "ConfigurationSquaredError::update() -- wrong number of input ports!!" << endl;
        return false;
    }
    b[0] = inputPorts[0]->read(false);
    b[1] = inputPorts[1]->read(false);
    
    if( (b[0]==NULL) || (b[1]==NULL) ) {
        // non fatal error
        //cout << "ConfigurationSquaredError::update() problem reading data!!" << endl;
        //        if(size==0) {
        //    diff.resize(1);
        //    diff.zero();
        //}
        return ok;
    }

    offset = 1;
    int s = b[0]->get(0).asInt();
    if(size!=s) {
        size = s;
        inputs[0]->resize(size);
        inputs[1]->resize(size);
        gradient.resize(size);
        cout << "ConfigurationSquaredError setting size: " << size << endl;
    }
    diff.resize(size);
    for(int i=0; i<size; i++) {
        (*inputs[0])[i] = b[0]->get(i+offset).asDouble();
        (*inputs[1])[i] = b[1]->get(i+offset).asDouble();
        diff[i] = (*inputs[1])[i] - (*inputs[0])[i];
    }

    // compute the potential and its gradient
    gradient = -1.0*diff;
    potential = 0.5*dot(diff,diff);

    /*
    cout << "ref   -  cur  =  diff" << endl;
    for(int i=0; i<size; i++) {
        cout << (*inputs[1])[i] << "  " << (*inputs[0])[i] << "  " << diff[i] << endl;
    }
    cout << endl;
    */

    //    cout << "ConfigurationSquaredError Potential = " << potential << endl;
    return ok;

}

bool CB::ConfigurationSquaredError::connectToInputs() {
    
    bool ok = true;

    if(inputNames.size() != 2) {
        cout << "ConfigurationSquaredError::connectToInputs() -- size mismatch!!" << endl;
        return false;
    }
    cout << "ConfigurationSquaredError::connectToInputs():\n\t" << inputNames[0].c_str() << " \n\t" << inputNames[1].c_str() << "\n\n"; 
  
    string configCurName = inputNames[0] + "/data:o";
    string configRefName = inputNames[1] + "/data:o";

    string prefixStr = "/cb/" + getSpace();
    int s = prefixStr.size();
    string tmp0 = inputNames[0];
    string tmp1 = inputNames[1];
    tmp0.erase(0,s);
    tmp1.erase(0,s);
    
    string configCurNameIn = "/cb/configuration/squared_error_pf" + tmp0 + ":i";
    string configRefNameIn = "/cb/configuration/squared_error_pf" + tmp1 + ":i";

    cout << "ConfigurationSquaredError::connectToInputs() -- opening current input port..." << endl;
    ok &= inputPorts[0]->open(configCurNameIn.c_str());
    if(!ok) {
        cout << "ConfigurationSquaredError::connectToInputs() -- failed opening current input port..." << endl;
        return ok;
    }
    cout << "ConfigurationSquaredError::connectToInputs() -- opening reference input port..." << endl;
    ok &= inputPorts[1]->open(configRefNameIn.c_str());
    if(!ok) {
        cout << "ConfigurationSquaredError::connectToInputs() -- failed opening reference input port..." << endl;
        return ok;
    }

    //    Time::delay(0.1);
    cout << "ConfigurationSquaredError::connectToInputs() -- connecting:\n\t" << 
        configCurName.c_str() << " -> " << configCurNameIn.c_str() << "\n\t" << 
        configRefName.c_str() << " -> " << configRefNameIn.c_str() << endl << endl;
    
    ok &= Network::connect(configCurName.c_str(),configCurNameIn.c_str(), "tcp");
    //Time::delay(0.1);
    ok &= Network::connect(configRefName.c_str(),configRefNameIn.c_str(), "tcp");
    if(!ok) {
        cout << "ConfigurationSquaredError::connectToInputs() -- failed connecting to input ports..." << endl << endl;
        return ok;
    }
    cout << "ConfiguartionSquaredError done connecting to YARP input ports..." << endl;
    connectedToInputs = true;
    return ok; 
}


