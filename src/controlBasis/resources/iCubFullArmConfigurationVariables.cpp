// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "iCubFullArmConfigurationVariables.h"

#include <string.h>

using namespace std;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;

void CB::iCubFullArmConfigurationVariables::startResource() {
  
    if(!connectedToDevice) {
        if(!connectToDevice()) {
            cout << "Couldn't connect to YARP port in startResource()..." << endl;
            return;
        }
    }
    running = true;
    start();     // mandatory start function
}



void CB::iCubFullArmConfigurationVariables::stopResource() {

    cout << "iCubFullArmConfigurationVariables::stopResource()" << endl;
    stop();     // mandatory stop function

    // stop motor device
    dd[0]->close();
    dd[1]->close();
    connectedToDevice = false;        
    running = false;
}



bool CB::iCubFullArmConfigurationVariables::updateResource() {

    //cout << "YARPConfiguration updating robot=" << deviceName.c_str() << ", dof=" << numDOFs << ", links=" << numLinks << endl;

    // get current values of config variables from device
    bool ok;
    int jnts;
    double *tmp[2];
    int limit;

    // get torso info
    for(int i=0; i<2; i++) {

        ok = dd[i]->view(pos[i]);
        ok &= dd[i]->view(enc[i]);
        ok &= dd[i]->view(lim[i]);
        
        if (!ok) {
            cout << "Problems acquiring torso interfaces" << endl;
            return ok;
        }
        
        jnts = 0;
        pos[i]->getAxes(&jnts);

        tmp[i] = new double[jnts];
        enc[i]->getEncoders(tmp[i]);

        if(i==0) {
            limit = 3; 
        } else {
            limit = 7;
        }
        
        for(int k=0; k<limit; k++) {
            values[(i*3)+k] = tmp[i][k]*TORAD;
        } 
        delete tmp[i];
    }

    // send positions to robot
    if(!lock) {
        pos[0]->positionMove(0, desiredValues[0]*TODEG);
        pos[0]->positionMove(1, desiredValues[1]*TODEG);
        pos[0]->positionMove(2, desiredValues[2]*TODEG);
        pos[1]->positionMove(0, desiredValues[3]*TODEG);
        pos[1]->positionMove(1, desiredValues[4]*TODEG);
        pos[1]->positionMove(2, desiredValues[5]*TODEG);
        pos[1]->positionMove(3, desiredValues[6]*TODEG);
        pos[1]->positionMove(4, desiredValues[7]*TODEG);
        pos[1]->positionMove(5, desiredValues[8]*TODEG);
        pos[1]->positionMove(6, desiredValues[9]*TODEG);
    }

    //    cout  << "arm " << which_arm << ":\n"; 
    //for(int i=0; i<10; i++) cout << values[i] << endl;

    return ok;
}


bool CB::iCubFullArmConfigurationVariables::connectToDevice() {

    bool ok;
    cout << "iCubFullArmConfigurationVariables() -- connecting to port..." << endl;
    connectedToDevice = false;
    
    //    Property options;
    string torso_local_name = torsoDevPort + "_" + which_arm + "/client";
    string torso_remote_name = torsoDevPort;
    string arm_local_name = armDevPort + "/full/client";
    string arm_remote_name = armDevPort;
    
    options[0].put("device", "remote_controlboard");
    options[0].put("local", torso_local_name.c_str());
    options[0].put("remote", torso_remote_name.c_str());
    fprintf(stderr, "%s", options[0].toString().c_str());
    dd[0] = new PolyDriver(options[0]);       

    if (!dd[0]->isValid()) {
        cout << "Torso Device not available.  Here are the known devices:" << endl;
        cout << Drivers::factory().toString().c_str() << endl;
        ok = false;
    }    

    options[1].put("device", "remote_controlboard");
    options[1].put("local", arm_local_name.c_str());
    options[1].put("remote", arm_remote_name.c_str());
    fprintf(stderr, "%s", options[1].toString().c_str());
    dd[1] = new PolyDriver(options[1]);       

    if (!dd[1]->isValid()) {
        cout << "Arm Device not available.  Here are the known devices:" << endl;
        cout << Drivers::factory().toString().c_str() << endl;
        ok = false;
    }    

    int jnts;
    double *tmp[2];
    int limit;
    double min, max;

    // get torso info
    for(int i=0; i<2; i++) {
        ok = dd[i]->view(pos[i]);
        ok &= dd[i]->view(enc[i]);
        ok &= dd[i]->view(lim[i]);
        
        if (!ok) {
            cout << "Problems acquiring torso interfaces" << endl;
            return ok;
        }

        Time::delay(2);
        
        jnts = 0;
        pos[i]->getAxes(&jnts);

        tmp[i] = new double[jnts];
        enc[i]->getEncoders(tmp[i]);
        
        if(i==0) {
            limit = 3; 
        } else {
            limit = 7;
        }
        
        for(int k=0; k<limit; k++) {       
            values[(i*3)+k] = tmp[i][k]*TORAD;
            desiredValues[(i*3)+k] = tmp[i][k]*TORAD;
            // get joint limit information
            lim[i]->getLimits(k, &min, &max);
            minLimits[(i*3)+k] = min*TORAD;
            maxLimits[(i*3)+k] = max*TORAD;
        } 

        delete tmp[i];
    }

    // send positions to robot
    pos[0]->positionMove(0, desiredValues[0]*TODEG);
    pos[0]->positionMove(1, desiredValues[1]*TODEG);
    pos[0]->positionMove(2, desiredValues[2]*TODEG);
    pos[1]->positionMove(0, desiredValues[3]*TODEG);
    pos[1]->positionMove(1, desiredValues[4]*TODEG);
    pos[1]->positionMove(2, desiredValues[5]*TODEG);
    pos[1]->positionMove(3, desiredValues[6]*TODEG);
    pos[1]->positionMove(4, desiredValues[7]*TODEG);
    pos[1]->positionMove(5, desiredValues[8]*TODEG);
    pos[1]->positionMove(6, desiredValues[9]*TODEG);
    
    for(int k=0; k<10; k++) cout << "limits[" << k << "]: (" << minLimits[k] << "," << maxLimits[k] << ")" << endl;
    
    connectedToDevice = true;

    return ok; 

}

void CB::iCubFullArmConfigurationVariables::loadConfig(string fname) {

    FILE *fp;
    char line[128];
    Matrix inTransform(4,4);
    char *linkType = (char *)malloc(64);

    if( (fp=fopen(fname.c_str(), "r")) == NULL ) {
        cout << "problem opening " << fname.c_str() << endl;
        exit(-1);
    }

    float a, d, alpha, theta;
    float t0, t1, t2, t3;
    float maxVal;
    int lnk = 0;
    int start,stop;
    string lineStr, tmpStr;
    int mask_size; 
    int c=0;
    vector<int> tmp_mask;
    bool found_mask = false;

    while(fgets(line, 128, fp) != NULL) {
        if(!strncmp(line,"N:",2)) {
            lineStr = string(line);
            start = lineStr.find_first_of(" ", 0);
            stop = lineStr.find_first_of(" \n", start+1);
            tmpStr = lineStr.substr(start,stop-start);            
            numDOFs = atoi(tmpStr.c_str());
            if(numDOFs != values.size()) {
                values.resize(numDOFs); values.zero();
                desiredValues.resize(numDOFs); desiredValues.zero();
                maxLimits.resize(numDOFs); maxLimits.zero();    
                minLimits.resize(numDOFs); minLimits.zero();                
            }
        }
        else if(!strncmp(line,"L:",2)) {
            lineStr = string(line);
            start = lineStr.find_first_of(" ", 0);
            stop = lineStr.find_first_of(" \n", start+1);
            tmpStr = lineStr.substr(start,stop-start);            
            numLinks = atoi(tmpStr.c_str());
            if(numLinks != LinkTypes.size()) {
                LinkTypes.resize(numLinks); LinkTypes.zero();
                DHParameters.resize(4,numLinks); DHParameters.zero();                
            }
        } else if(!strncmp(line,"DH",2)) {
            for(int i=0; i<numLinks; i++) {
               
                if(fgets(line, 128, fp) !=NULL) {                    
                    sscanf (line, "%f %f %f %f %s", &a, &d, &alpha, &theta, linkType);
                    DHParameters[DH_A][i] = (double)a;
                    DHParameters[DH_D][i] = (double)d;
                    DHParameters[DH_ALPHA][i] = (double)alpha*TORAD;
                    DHParameters[DH_THETA][i] = (double)theta*TORAD;
                    
                    if(!strcmp(linkType, "CONSTANT")) {
                        LinkTypes[i] = LINK_TYPE_CONSTANT;
                    } else if(!strcmp(linkType, "PRISMATIC")) {
                        LinkTypes[i] = LINK_TYPE_PRISMATIC;
                    } else if(!strcmp(linkType, "REVOLUTE")) {
                        LinkTypes[i] = LINK_TYPE_REVOLUTE;
                    } else if(!strcmp(linkType, "NONINTERFERING")) {
                        LinkTypes[i] = LINK_TYPE_NONINTERFERING;
                    } else { 
                        LinkTypes[i] = LINK_TYPE_CONSTANT;
                    }

                    cout << "Link[" << i << "]: [";
                    cout << DHParameters[DH_A][i] << " ";
                    cout << DHParameters[DH_D][i] << " "; 
                    cout << DHParameters[DH_ALPHA][i] << " "; 
                    cout << DHParameters[DH_THETA][i];
                    cout << "] type: " << LinkTypes[i] << endl;
                }
            }
        }
        else if(!strncmp(line,"MAX:",4)) {
            lineStr = string(line);
            start = lineStr.find_first_of(" ", 0);
            stop = lineStr.find_first_of(" \n", start+1);
            tmpStr = lineStr.substr(start,stop-start);            
            maxSetVal = atof(tmpStr.c_str());
            cout << "MaxSetVal: " << maxSetVal << endl;        
        }
    }
    cout << "iCubFullArmConfigurationVariables::loadConfig() got DOFs=" << numDOFs << ", Links=" << numLinks << endl;
    free(linkType);        
    fclose(fp);

}
