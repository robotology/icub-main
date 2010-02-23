// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "ManipulabilityField.h"
#include <yarp/os/Network.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iKin;

void CB::ManipulabilityField::startPotentialFunction() {
    if(!connectedToInputs) {
        if(!connectToInputs()) {
            cout << "Couldn't connect to YARP input port in startPotentialFunction()..." << endl;
            return;
        }
    }
    running = true;
    start();     // mandatory start function
}

void CB::ManipulabilityField::stopPotentialFunction() {
    stop();     // mandatory stop function
}

bool CB::ManipulabilityField::updatePotentialFunction() {
    
    //    cout << "ManipulabilityField updating potential" << endl;

    Bottle *b;
    double offset;
    bool ok = true;
    bool normalize = true;

    Vector Vsim(size);
    Vector Vgrad(size);
    double inc = 0.025;
    Matrix Jsim(3,size);
    Matrix Jfull;
    double m,m1,m2;
    double epsilon = 1E-7;
    double mag;

    // get data from ports
    if(inputPorts.size() != 1) {
        cout << "ManipulabilityField::update() -- wrong number of input ports!!" << endl;
        return false;
    }
    b = inputPorts[0]->read(false);

    if(b==NULL) {
        // non fatal error
        // cout << "ManipulabilityField::update() problem reading data!!" << endl;
        return ok;
    }

    offset = 1;
    potential = 0;

    for(int i=0; i<size; i++) {
        (*inputs[0])[i] = b->get(i+offset).asDouble();
        Vsim[i] = (*inputs[0])[i];
    }
    mag = 0;

    for(int i=0; i<size; i++) {

        // create simulated joint angles        
        Vsim[i] += inc;

        // set joints in chain
        Vsim=kinChain->setAng(Vsim);

        // compute jacobian and manip metric
        Jfull = kinChain->GeoJacobian();    
        Jsim = Jfull.submatrix(0,2,0,(size-1));
        m2 = getManipulability(Jsim);
        if (fabs(m2) < epsilon) m2 = 0;

        // go back the other way
        Vsim[i] -= (2.0 * inc);

        // set joints in chain
        Vsim=kinChain->setAng(Vsim);
        
        // compute jacobian and manip metric
        Jfull = kinChain->GeoJacobian();    
        Jsim = Jfull.submatrix(0,2,0,(size-1));
        m1 = getManipulability(Jsim);       
        if (fabs(m1) < epsilon) m1 = 0;

        // do the interpolation
        Vgrad[i] = (m2 - m1) / 2.0;
        if (fabs(Vgrad[i]) < epsilon) Vgrad[i] = 0;

        mag += (Vgrad[i]*Vgrad[i]);
        // reset angles
        Vsim[i] += inc;
        
    }
    mag = sqrt(mag);

    // set a normalized version of the jacobian as the output gradient
    for (int i = 0; i < size; i++) {
        if(normalize) {
            gradient[i] = (Vgrad[i] / mag);
        } else {
            gradient[i] = Vgrad[i];
        }        
    }
    gradient = -1.0*gradient;

    // get manipulability for acutal joint angles
    (*inputs[0])=kinChain->setAng((*inputs[0]));
        
    // compute jacobian and manip metric
    Jfull = kinChain->GeoJacobian();    
    Jsim = Jfull.submatrix(0,2,0,(size-1));
    m = getManipulability(Jsim);       
    potential = 10*(0.02-m);  
    
    cout << "ManipulabilityField Potential = " << potential << endl;
    return ok;

}

bool CB::ManipulabilityField::connectToInputs() {
    
    bool ok = true;
    int numLinks;
    int numJoints;
    Matrix H(4,4);

    if(inputNames.size() != 1) {
        cout << "ManipulabilityField::connectToInputs() -- size mismatch!!" << endl;
        return false;
    }
    cout << "ManipulabilityField::connectToInputs():\n\t " << inputNames[0].c_str() << "\n\n";
    
    string configName = inputNames[0] + "/data:o";
    string paramsName = inputNames[0] + "/params:o";

    string prefixStr = "/cb/" + getSpace();
    int s = prefixStr.size();
    string tmp = inputNames[0];
    tmp.erase(0,s);
    
    string configNameIn = "/cb/configuration/manipulability_pf" + tmp + "/data:i";
    string paramsNameIn = "/cb/configuration/manipulability_pf" + tmp + "/params:i";

    cout << "ManipulabilityField::connectToInputs() -- opening current input port..." << endl;
    ok &= inputPorts[0]->open(configNameIn.c_str());
    if(!ok) {
        cout << "ManipulabilityField::connectToInputs() -- failed opening config input port..." << endl;
        return ok;
    }
    cout << "ManipulabilityField::connectToInputs() -- opening params input port..." << endl;
    ok &= paramsInputPort.open(paramsNameIn.c_str());
    if(!ok) {
        cout << "ManipulabilityField::connectToInputs() -- failed opening params input port..." << endl;
        return ok;
    }
    //    Time::delay(1);
    cout << "ManipulabilityField::connectToInputs() -- connecting:\n\t" << configName.c_str() << " -> " << configNameIn.c_str() << "\n\n";          
    cout << "ManipulabilityField::connectToInputs() -- connecting:\n\t" << paramsName.c_str() << " -> " << paramsNameIn.c_str() << "\n\n";

    ok &= Network::connect(configName.c_str(),configNameIn.c_str(),"tcp");
    if(!ok) {
        cout << "ManipulabilityField::connectToInputs() -- failed connecting to input port..." << endl << endl;
        return ok;
    }
    ok &= Network::connect(paramsName.c_str(),paramsNameIn.c_str(),"tcp");
    if(!ok) {
        cout << "ManipulabilityField::connectToInputs() -- failed connecting to param input port..." << endl << endl;
        return ok;
    }
       
    int thresh = 10;
    int t = 0;
    Bottle *b = paramsInputPort.read(true);

    while(b==NULL) {
        cout << "ManipulabilityField::connect() -- port read failed on attempt " << t << endl;    
        b = paramsInputPort.read(true);
        t++;
        if(t==thresh) {
            cout << "ManipulabilityField::connect() -- port read failed, giving up!!" << endl;    
            ok = false;
            return ok;
        }
        Time::delay(1);
    }

    int c=0;
    int offset = 1;
    int lnk_idx = 0;

    cout << "ManipulabilityField read params from port successfully..." << endl;    
    if(ok) {

        numLinks = b->get(0).asInt();
        LinkTypes.resize(numLinks);
        DHParams.resize(4,numLinks);
        numJoints = 0;

        kinChain = kinLimb.asChain();
        linkList.clear();
        H.eye();

        for(int i=0; i<numLinks; i++) {
            for(int j=0; j<4; j++) {
                DHParams[j][i] = b->get(offset+j).asDouble();
            }
            LinkTypes[i] = b->get(offset+4).asInt();
            offset+=5;

            cout << "Link[" << i << "] Info: [" <<
                DHParams[DH_A][i] << " " <<
                DHParams[DH_D][i] << " " <<
                DHParams[DH_ALPHA][i] << " " <<
                DHParams[DH_THETA][i] << " , type: " << LinkTypes[i] << endl;

            if( LinkTypes[i] == (int)LINK_TYPE_CONSTANT ) {
                
                iKinLink link(DHParams[DH_A][i],
                              DHParams[DH_D][i],
                              DHParams[DH_ALPHA][i],
                              DHParams[DH_THETA][i]);                           
                H = H*link.getH(0,false);
                kinChain->setH0(H);
             
            } else if(LinkTypes[i] == (int)LINK_TYPE_NONINTERFERING) {
                
                linkList.push_back(new iKinLink(DHParams[DH_A][i],
                                                DHParams[DH_D][i],
                                                DHParams[DH_ALPHA][i],
                                                DHParams[DH_THETA][i])); 
                kinChain->pushLink(*linkList[lnk_idx]);           
                kinChain->blockLink(lnk_idx);
                lnk_idx++;
                
            } else if(LinkTypes[i] == (int)LINK_TYPE_REVOLUTE) {
                
                linkList.push_back(new iKinLink(DHParams[DH_A][i],
                                                DHParams[DH_D][i],
                                                DHParams[DH_ALPHA][i],
                                                DHParams[DH_THETA][i]));
                kinChain->pushLink(*linkList[lnk_idx]);           
                lnk_idx++;
                numJoints++;
            } 
            
        }
        cout << kinChain->getDOF() << " DOFs available from iKinChain" << endl;
        size = numJoints;
        paramsSet = true;
    
        cout << "ManipulabilityField::connectToInputs() -- read parameter data, got size=" << size << endl;
        
        inputs[0]->resize(size); inputs[0]->zero();
        gradient.resize(size); gradient.zero();
        
    }  
    else {
        cout << "could not read data..." << endl;
    }
    
    ok &= Network::disconnect(paramsName.c_str(),paramsNameIn.c_str());            
    paramsInputPort.close();

    cout << "ManipulabilityField done connecting to YARP input ports..." << endl;
    connectedToInputs = true;
    return ok; 
}
    

double CB::ManipulabilityField::getManipulability(Matrix M) {

  double manip, d;
  Matrix MT;(M.cols(),M.rows());
  Matrix MMT(M.rows(),M.rows());
  MT = M.transposed();
  MMT = M*MT;
  d = det(MMT);
  manip = sqrt(d);
  return manip;

}
