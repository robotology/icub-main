// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "CosineField.h"
#include <yarp/os/Network.h>
#include <math.h>

using namespace std;
using namespace yarp::os;

void CB::CosineField::startPotentialFunction() {
    if(!connectedToInputs) {
        if(!connectToInputs()) {
            cout << "Couldn't connect to YARP input port in startPotentialFunction()..." << endl;
            return;
        }
    }
    start();     // mandatory start function
}

void CB::CosineField::stopPotentialFunction() {
    stop();     // mandatory stop function
}

bool CB::CosineField::updatePotentialFunction() {
    
    Bottle *b;
    double offset;
    bool ok = true;
    double tmp;

    // get data from ports (should be more safety checks...)
    if(inputPorts.size() != 1) {
        cout << "CosineField::update() -- wrong number of input ports!!" << endl;
        return false;
    }
    b = inputPorts[0]->read(false);

    if(b==NULL) {
        // non fatal error (prob cause of asynchronous update rates...)
        //        cout << "CosineField::update() problem reading data!!" << endl;
        return ok;
    }

    // get the actual size of the device
    if(size==0) {
        size = b->get(0).asInt();
        if(size!=0) {
            inputs[0]->resize(size);
            gradient.resize(size);
            maxLimits.resize(size);
            minLimits.resize(size);
            centers.resize(size);
            ranges.resize(size);
            cout << "CosineField setting size: " << size << endl;
        }
    }

    offset = 1;
    potential = 0;

    for(int i=0; i<size; i++) {
        (*inputs[0])[i] = b->get(i+offset).asDouble();

        // compute gradient
        tmp = -(-((M_PI) / ranges[i]) * cos(M_PI * (((*inputs[0])[i] - minLimits[i]) / ranges[i])));        
    
        // check to make sure not too small
        if (fabs(tmp) < 1E-15) tmp = 0;

        // set it in the state
        gradient[i] = -1.0*tmp;
        potential += cos(((*inputs[0])[i] - centers[i]) / ranges[i]);

    }
    
    potential = size-potential;
    
    return ok;

}

bool CB::CosineField::connectToInputs() {
   
    bool ok = true;

    if(inputNames.size() != 1) {
        cout << "CosineField::connectToInputs() -- size mismatch!!" << endl;
        return false;
    }
    cout << "CosineField::connectToInputs():\n\t " << inputNames[0].c_str() << "\n\n";

    string configName = inputNames[0] + "/data:o";
    string limitsName = inputNames[0] + "/limits:o";

    string prefixStr = "/cb/" + getSpace();
    int s = prefixStr.size();
    string tmp = inputNames[0];
    tmp.erase(0,s);
    
    string configNameIn = "/cb/configuration/cosfield_pf" + tmp + ":i";
    string limitsNameIn = "/cb/configuration/cosfield_pf" + tmp + "/limits:i";

    cout << "CosineField::connectToInputs() -- opening current input port..." << endl;
    ok &= inputPorts[0]->open(configNameIn.c_str());
    if(!ok) {
        cout << "CosineField::connectToInputs() -- failed opening config input port..." << endl;
        return ok;
    }

    cout << "CosineField::connectToInputs() -- opening limits input port..." << endl;
    ok &= limitsInputPort.open(limitsNameIn.c_str());
    if(!ok) {
        cout << "CosineField::connectToInputs() -- failed opening limits input port..." << endl;
        return ok;
    }

    cout << "CosineField::connectToInputs() -- connecting:\n\t " 
         << configName.c_str() << "-> " << configNameIn.c_str() << endl << endl;

    cout << "CosineField::connectToInputs() -- connecting:\n\t " 
         << limitsName.c_str() << "-> " << limitsNameIn.c_str() << endl << endl;

    ok &= Network::connect(configName.c_str(),configNameIn.c_str(),"tcp");
    if(!ok) {
        cout << "CosineField::connectToInputs() -- failed connecting to input ports..." << endl << endl;
        return ok;
    }
    ok &= Network::connect(limitsName.c_str(),limitsNameIn.c_str(),"tcp");
    if(!ok) {
        cout << "CosineField::connectToInputs() -- failed connecting to limit input ports..." << endl << endl;
        return ok;
    }
   
        
    int thresh = 10;
    int t = 0;
    Bottle *b = limitsInputPort.read(true);
    while(b==NULL) {
        cout << "CosField::connect() -- port read failed on attempt " << t << endl;    
        b = limitsInputPort.read(true);
        t++;
        if(t==thresh) {
            cout << "CosField::connect() -- port read failed, giving up!!" << endl;    
            ok = false;
            return ok;
        }
        Time::delay(1);
    }

    int c=0;
    int offset = 1;
    cout << "CosField read limits from port successfully..." << endl;    

    ok &= Network::disconnect(limitsName.c_str(),limitsNameIn.c_str());            
    limitsInputPort.close();

    if(ok) {
        size = b->get(0).asInt();        
        inputs[0]->resize(size);
        minLimits.resize(size);
        maxLimits.resize(size);
        gradient.resize(size);
        centers.resize(size);
        ranges.resize(size);
            
        for(int i = 0; i<b->size(); i+=2) {
            minLimits[c] = b->get(offset+i).asDouble();
            maxLimits[c] = b->get(offset+i+1).asDouble();
            c++;
        }
        cout << "cosfield got limits" << endl;
        
        for(int i=0; i<size; i++) {
            
            (*inputs[0])[i] = b->get(i+offset).asDouble();
            cout << (*inputs[0])[i] << " in (" << minLimits[i] << " <-> " << maxLimits[i] << ")" << endl;
            
            ranges[i] = maxLimits[i] - minLimits[i];
            centers[i] = minLimits[i] + (ranges[i] / 2.0);
        }
        
        cout << "" << endl;
        
    }  
    else {
        cout << "could not read data..." << endl;
    }
    
    connectedToInputs = true;
    return ok; 
}
    

