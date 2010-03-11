// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "EndEffectorCartesianPosition.h"
#include <yarp/os/Network.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iKin;

void CB::EndEffectorCartesianPosition::startResource() {
    // if not connected to the configuration resource, do that now
    if(!connectedToConfiguration) {
        if(!connectToConfiguration()) {
            cout << "EE cartpos Couldn't connect to configuration port in startResource()..." << endl;
            return;
        }
    }
    start();     // mandatory start function
}

void CB::EndEffectorCartesianPosition::stopResource() {
  stop();     // mandatory stop function
}

bool CB::EndEffectorCartesianPosition::updateResource() {
  
    Bottle *b;
    int nj;
    int offset;
    bool ok = true;
    Vector xf, q0;
    int idx = 0;

    // read the current configuration variables from the input port
    b = inputPort[0]->read(false);
    if(b!=NULL) {

        nj = b->get(0).asInt();
        offset = 1;

        configVals.resize(nj);
        for(int i=0; i<nj; i++) {            
            configVals[i] = b->get(i+offset).asDouble();
        }

        // set the angles and get the axis-angle end-effector pose using iKin
        configVals=kinChain->setAng(configVals);
        xf=kinChain->EndEffPose(false);     

        // copy position
        values[0] = xf[0];
        values[1] = xf[1];
        values[2] = xf[2];

        //cout << "Pos=[" << values[0] << " " << values[1] << " " << values[2] << "]" << endl;
    } 

    return ok;
    
}

bool CB::EndEffectorCartesianPosition::connectToConfiguration() {
    
    Bottle *b_params;
    Bottle *b_limits;
    bool ok = true;
    int numLinks;
    int numJoints;
    Vector minLimits(1);
    Vector maxLimits(1);
    Matrix H(4,4);

    int offset_params = 1;
    int offset_limits = 1;
    int jnt_idx = 0;
    int lnk_idx = 0;

    if(inputPort.size() != numInputs) {
        return false;
    }    

    // connect to link port for configuration resource
    string linkOutputName = "/cb/configuration" + deviceName + "/params:o";
    string limitsOutputName = "/cb/configuration" + deviceName + "/limits:o";

    cout << "EndEffectorCartesianPosition::connectToConfiguration() -- connecting ports for link parameters..." << endl;
    ok &= Network::connect(linkOutputName.c_str(),inputPortName[1].c_str(),"tcp");
    if(!ok) {
        cout << "EndEffectorCartesianPosition::connectToConfiguration() -- connecting input port falied..." << endl;
        return ok;
    }

    cout << "EndEffectorCartesianPosition::connectToConfiguration() -- connecting ports for limit parameters..." << endl;
    ok &= Network::connect(limitsOutputName.c_str(),inputPortName[2].c_str(),"tcp");
    if(!ok) {
        cout << "EndEffectorCartesianPosition::connectToConfiguration() -- connecting input port falied..." << endl;
        return ok;
    }

    // get Link information (DHParameters and link limits (unused))
    int c = 0;
    int thresh = 10;
    b_params = inputPort[1]->read(true);
    b_limits = inputPort[2]->read(true);
    while( (b_params==NULL) || (b_limits==NULL) ) {
        cout << "EndEffectorCartesianPosition::connect() -- didn't read configuration parameter information. trying again..." << endl;
        b_params = inputPort[1]->read(true);
        b_limits = inputPort[2]->read(true);
        c++;
        if(c == thresh) {
            cout << "EndEffectorCartesianPosition::connect() -- could not read parameter information. Giving up!!" << endl;
            ok = false;
            return ok;        
        }
        Time::delay(1);
    }
    
    cout << "EndEffectorCartesianPosition::connect() -- read link parameter information, parsing..." << endl;
    numLinks = b_params->get(0).asInt();
    numJoints = b_limits->get(0).asInt();

    LinkTypes.resize(numLinks);
    DHParams.resize(4,numLinks);
    minLimits.resize(numJoints);
    maxLimits.resize(numJoints);
   
    kinChain = kinLimb.asChain();
    linkList.clear();
    H.eye();

    cout << "EndEffectorCartesianPosition::connectToConfiguration() -- setting up FK" << endl; 

    // now that we have the DH parameters, copy to local storage and send to iKin
    for(int i=0; i<numLinks; i++) {
        for(int j=0; j<4; j++) {
            DHParams[j][i] = b_params->get(offset_params+j).asDouble();
        }
        LinkTypes[i] = b_params->get(offset_params+4).asInt();
        offset_params+=5;

        cout << "Link[" << i  << "] Info: [" << 
            DHParams[DH_A][i] << " " <<
            DHParams[DH_D][i] << " " <<
            DHParams[DH_ALPHA][i] << " " <<
            DHParams[DH_THETA][i] << " ], type: " <<
            LinkTypes[i] << endl;

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

            minLimits[jnt_idx] = b_limits->get(offset_limits).asDouble();
            maxLimits[jnt_idx] = b_limits->get(offset_limits+1).asDouble();
            offset_limits+=2;
            
            linkList.push_back(new iKinLink(DHParams[DH_A][i],
                                            DHParams[DH_D][i],
                                            DHParams[DH_ALPHA][i],
                                            DHParams[DH_THETA][i], 
                                            minLimits[jnt_idx],maxLimits[jnt_idx]));
            kinChain->pushLink(*linkList[lnk_idx]);           
            jnt_idx++;
            lnk_idx++;

        } 

    }

    // clear some FP residual in input matrix
    for(int i=0; i<4; i++) {
        for(int j=0; j<4; j++) {
            if(fabs(H[i][j]) < 1E-10) H[i][j] = 0;
        }
    }

    // connect to config port for reading config values
    string configOutputName = "/cb/configuration" + deviceName + "/data:o";

    cout << "EndEffectorCartesianPosition::connectToConfiguration() -- connecting ports for configuration values..." << endl;
    ok &= Network::connect(configOutputName.c_str(),inputPortName[0].c_str(),"tcp");
    if(!ok) {
        cout << "EndEffectorCartesianPosition::connectToConfiguration() -- connecting to config port falied..." << endl;
        return ok;
    }

    connectedToConfiguration = true;
    paramsSet = true;
   
    return ok;
}
