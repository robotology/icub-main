// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "YARPConfigurationVariables.h"

#include <string.h>
#include <yarp/os/Network.h>

using namespace std;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;

void CB::YARPConfigurationVariables::startResource() {
    // if the resource hasnt connected to the YARP device, do it now
    if(!connectedToDevice) {
        if(!connectToDevice()) {
            cout << "Couldn't connect to YARP port in startResource()..." << endl;
            return;
        }
    }
    start();     // mandatory start function
}



void CB::YARPConfigurationVariables::stopResource() {
    cout << "YARPConfigurationVariables::stopResource()" << endl;
    stop();     // mandatory stop function
}



bool CB::YARPConfigurationVariables::updateResource() {

    // get current values of config variables from PolyDriver
    bool ok;
    ok = dd->view(pos);
    ok &= dd->view(enc);
    ok &= dd->view(lim);

    if (!ok) {
        cout << "Problems acquiring interfaces" << endl;
        return ok;
    }

    // get encoder data
    int jnts = 0;
    pos->getAxes(&jnts);
    double *tmp = new double[jnts];
    enc->getEncoders(tmp);

    // make sure the size of the mask vector matches the total number of DOFs of the device
    if( jnts != mask.size() ) {
        cout << "YARPConfiguration::updateResource() -- number of Joints from YARP motor interface mismatch!!" << endl;
        return false;
    }    

    // copy the device positions to the local resource data (masking out values if necessary)
    int idx = 0;
    for(int i = 0; i < mask.size(); i++) {
        if(!mask[i]) continue;
        values[idx++] = tmp[i]*TORAD;
    }

    // set device values for (unmasked) joints
    if(!velocityControlMode) {
        // send to PolyDriver
        if(moveable && !lock) {
            idx = 0;
            for(int i=0; i<mask.size(); i++) {
                if(!mask[i]) continue;
                pos->positionMove(i, desiredValues[idx++]*TODEG);            
                //cout << "sending position["<<i<<"] -> " << desiredValues[idx-1]*TODEG << endl;
            }
        }       
    } else {
        // send to velocityControl module
        if(moveable && !lock) {            
            Vector &v = velocityPort.prepare();
            v.resize(mask.size(),0);       
            idx = 0;
            for(int i=0; i<mask.size(); i++) {
                if(mask[i]) {
                    v[i] = desiredValues[idx++]*TODEG;            
                }
            }
            velocityPort.write();
        }
    }

    return ok;
}


bool CB::YARPConfigurationVariables::connectToDevice() {

    // if this is the first time running, need to connec to the PolyDriver.
    bool ok;
    int idx;
    cout << "YARPConfigurationVariables() -- connecting to port..." << endl;
    connectedToDevice = false;

    //    Property options;
    string local_name = localDevPort + "/client";
    string remote_name = remoteDevPort;
    
    options.put("device", "remote_controlboard");
    options.put("local", local_name.c_str());
    options.put("remote", remote_name.c_str());
    fprintf(stderr, "%s", options.toString().c_str());

    cout << endl << "creating new PolyDriver for " << remoteDevPort << endl;
    dd = new PolyDriver(options);       
    cout << "created." << endl;

    if (!dd->isValid()) {
        cout << "Device not available.  Here are the known devices:" << endl;
        cout << Drivers::factory().toString().c_str() << endl;
        ok = false;
    }    
    ok = dd->view(pos);
    ok &= dd->view(enc);
    ok &= dd->view(lim);

    if (!ok) {
        cout << "Problems acquiring interfaces" << endl;
        return ok;
    }

    Time::delay(2);  // necessary to make sure PolyDriver data is correct.

    // now get information from device
    int jnts = 0;
    pos->getAxes(&jnts);

    // if mask size = 1, it was not specified in the config file.  This assumes
    // that there is therefore no mask, and all device DOFs (up to the number 
    // specified) are controlled
    if(mask.size() == 1) {

        // first setup a virtual mask that passes through all joints
        mask.resize(jnts);
        for(int i=0; i<numDOFs; i++) mask[i] = 1;

        // set the number of DOFs and associated storage vectors
        numDOFs = jnts;
        cout << "Working with " << numDOFs << " axes" << endl;
        cout << "YARPConfigurationVariables::resizing storage vectors..." << endl;
        values.resize(numDOFs); values.zero();
        desiredValues.resize(numDOFs); desiredValues.zero();
        maxLimits.resize(numDOFs); maxLimits.zero();
        minLimits.resize(numDOFs); minLimits.zero();

    } else  if(jnts < numDOFs) {

        // less number of joints are availabe on the device that the number the resource wants to control 
        // this is bad, and so returns.
        cout << "numDOF size mismatch!!  numDOFs=" << numDOFs << ", device=" << jnts << endl;
        return 0;

    } else  if(jnts > numDOFs) {

        // there are more joints availabe from the device then the number the resource wants. 
        // in this case, lets just discard the excess...
        cout << "numDOF size mismatch!!  only taking first " << numDOFs << " vals. got " << jnts << " from device" << endl;

    } else {

        // everything is as it should be
        cout << "Working with axes " << numDOFs << ", with mask of: " << mask.size() << endl;
    }

    // get initial joint values (masking unwanted ones)
    double *tmp = new double[jnts];
    enc->getEncoders(tmp);
    idx = 0;
    for(int i = 0; i < mask.size(); i++) {
        if(!mask[i]) continue;
        values[idx] = tmp[i]*TORAD;
        desiredValues[idx] = values[idx]; // set initial desired to first read vals
        idx++;
    }

    // get joint limit information
    double min, max;
    idx = 0;
    for(int i = 0; i < mask.size(); i++) {
        if(!mask[i]) continue;
        lim->getLimits(i, &min, &max);
        minLimits[idx] = min*TORAD;
        maxLimits[idx] = max*TORAD;
        cout << "limits[" << idx << "]: (" << min << "," << max << ")" << endl;
        idx++;
    }    

    // set the connected flag cause everything seems to be okay
    connectedToDevice = true;

    return ok; 

}


void CB::YARPConfigurationVariables::setMask(Vector m) {

    // make sure the number of specified joints in the mask that are controlled
    // is equal to the number of joints the resource wants to specify (numDOFs)
    // if not, return...
    int c=0;
    for(int i=0; i<m.size(); i++) c += m[i];        

    if(c!=numDOFs) {
        cout << "YARPConfigurationVariables::setMask() -- mask size incorrect!!" << endl;
        return;
    } else {
        mask = m;
    }
}


void CB::YARPConfigurationVariables::loadConfig(string fname) {

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
        else if(!strncmp(line,"GAIN:",5)) {
            lineStr = string(line);
            start = lineStr.find_first_of(" ", 0);
            stop = lineStr.find_first_of(" \n", start+1);
            tmpStr = lineStr.substr(start,stop-start);            
            velocityGain = atof(tmpStr.c_str());
            cout << "Gain: " << velocityGain << endl;        
        }
        else if(!strncmp(line,"MASK:",5)) {
            lineStr = string(line);
            mask_size=0;
            tmp_mask.clear();
            stop = 0;
            while(true) {
                start = lineStr.find_first_of(" \n", stop);
                stop = lineStr.find_first_of(" \n", start+1);
                //cout << "start: " << start << ", stop: " << stop << endl;
                if(stop == -1) break;
                tmpStr = lineStr.substr(start,stop-start);            
                tmp_mask.push_back(atoi(tmpStr.c_str()));
                mask_size++;
            }
            mask.resize(mask_size);
            for(int i=0; i<mask_size; i++)  mask[i] = tmp_mask[i];
            cout << "Mask: ";
            for(int i=0; i<mask.size(); i++) cout << mask[i] << " ";
            cout << endl;        
            cout << "mask size: " << mask_size << endl;        
            
            c = 0;
            for(int i=0; i<mask.size(); i++) {
                c += mask[i];        
            }
            if(c!=numDOFs) {
                cout << "YARPConfigurationVariables::loadConfig() -- mask size incorrect!!" << endl;
                free(linkType);        
                fclose(fp);
                exit(-1);
            }
            found_mask = true;
            //exit(0);
        }
    }
    cout << "YARPConfigurationVariables::loadConfig() got DOFs=" << numDOFs << ", Links=" << numLinks << endl;
    free(linkType);        
    fclose(fp);

    // if there was no mask in config file, just create it here, assuming full passthrough...
    if(!found_mask) {
        mask.resize(numDOFs);
        for(int i=0; i<numDOFs; i++) mask[i] = 1;
    }
}


void CB::YARPConfigurationVariables::setVelocityControlMode(bool mode, string portName) {

    bool ok = true;

    // specify the local port names that connect to the velocityControl module
    string velocityOutputPortName = "/cb/configuration" + deviceName + "/vel:o";
    string velocityRPCOutputPortName = "/cb/configuration" + deviceName + "/vel/rpc:o";
    velocityPortName = portName + "/command";      
    velocityRPCPortName = portName + "/input";      
    Bottle b;

    if(mode && !velocityControlMode) {

        // if we're turning on the velocityControlMode (and it wasnt previously on)

        // open and connect the data and config portsport
        ok &= velocityPort.open(velocityOutputPortName.c_str());
        ok &= Network::connect(velocityOutputPortName.c_str(),velocityPortName.c_str(),"tcp");
        Time::delay(0.5);
        ok &= velocityRPCPort.open(velocityRPCOutputPortName.c_str());
        ok &= Network::connect(velocityRPCOutputPortName.c_str(),velocityRPCPortName.c_str(),"tcp");

        // send gain and maxVel paramaters to the vc module 
        for(int i=0; i<mask.size(); i++) {
            if(mask[i]) {
                b.clear();
                b.addString("gain");
                b.addInt(i);
                b.addDouble(velocityGain);
                velocityRPCPort.write(b);
                Time::delay(0.1);
                b.clear();
                b.addString("svel");
                b.addInt(i);
                b.addDouble(maxSetVal);
                velocityRPCPort.write(b);
                Time::delay(0.1);
            }
        }


    } else if(!mode && velocityControlMode) {

        // turning off velocity control mode by disconnecting and closing ports
        ok &= Network::disconnect(velocityOutputPortName.c_str(),velocityPortName.c_str());
        ok &= Network::disconnect(velocityRPCOutputPortName.c_str(),velocityRPCPortName.c_str());
        velocityRPCPort.close();
        velocityPort.close();

    }

    // set the current mode
    velocityControlMode = mode;


}

bool CB::YARPConfigurationVariables::getVelocityControlMode() {
    return velocityControlMode;
}
