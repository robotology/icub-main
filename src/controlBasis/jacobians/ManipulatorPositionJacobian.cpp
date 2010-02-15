// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "ManipulatorPositionJacobian.h"
#include <yarp/os/Network.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iKin;

void CB::ManipulatorPositionJacobian::startJacobian() {

    // if not conencted to inputs, do that now
  if(!connectedToInputs) {
    if(!connectToInputs()) {
      cout << "ManipulatorPositionJacobian couldn't connect to input ports in startJacobian()..." << endl;
      return;
    }
  }  
  start();     // mandatory start function
}

void CB::ManipulatorPositionJacobian::stopJacobian() {
  stop();     // mandatory stop function
}

bool CB::ManipulatorPositionJacobian::updateJacobian() {
  
  bool ok = true;
  Bottle *b;
  int nj;
  int offset;
  Matrix Jfull;

  // get the current configuration of the manipulator
  b = inputPort.read();
  if(b==NULL) {
      cout << "ManipulatorPositionJacobian::update() -- having trouble reading input data!" << endl;
      return ok;
  }
  nj = b->get(0).asInt();
  inputSize = nj;

  // check sizes, reinitialize if there is a mismatch
  if(configVals.size() != nj) {
    configVals.resize(nj); configVals.zero();
    J.resize(outputSize,inputSize); J.zero();
    cout << "ManipulatorPositionJacobian size mismatch, resizing=" << inputSize << endl;
  }

  // get the data
  offset = 1;  
  for(int i=0; i<nj; i++) {
    configVals[i] = b->get(i+offset).asDouble();
  }

  // send to iKin
  configVals=kinChain->setAng(configVals);
  //Jfull = kinChain->GeoJacobian();    
  Jfull = kinChain->AnaJacobian();    

  // copy the Jacobian to class var
  //  J.resize(3,Jfull.cols());
  for(int i=0; i<J.rows(); i++) {      
      for(int j=0; j<J.cols(); j++) {        
          J[i][j] = Jfull[i][j];
      }
  }

  return ok;
  
}

bool CB::ManipulatorPositionJacobian::connectToInputs() {

     bool ok = true;
     Bottle *b;
     int numLinks;
     int numJoints;
     Matrix H(4,4);

     // configue a random ID for this jacobian instance 
     int randomID = (int)(Random::uniform()*1000.0);
     char *c = (char *)malloc(16);
     sprintf(c,"%d", randomID);
     string randomIDstr(c);

     // connect to link port for configuration resource
     string linkOutputName = "/cb/configuration" + deviceName + "/params:o";
     string linkInputName = "/cb/manipulatorpositionjacobian/" + randomIDstr + "/" + deviceName + "/config/params:i";
     
     cout << "ManipulatorPositionJacobian::connectToInputs() -- opening input port for link parameters..." << endl;
     ok &= paramPort.open(linkInputName.c_str());
     if(!ok) {
       cout << "ManipulatorPositionJacobian::connectToInputs() -- could not open port for configuration link parameters!!" << endl;
       return ok;
     }

     cout << "ManipulatorPositionJacobian::connectToInputs() -- connecting ports for link parameters..." << endl;
     ok &= Network::connect(linkOutputName.c_str(),linkInputName.c_str(), "udp");
     if(!ok) {
       cout << "ManipulatorPositionJacobian::connectToInputs() -- could not connect to configuration link parameters!!" << endl;
       return ok;
     }
     
     // get Link information
     b = paramPort.read();
     if(b==NULL) {
       cout << "ManipulatorPositionJacobian::connectToInputs() -- could not read link information!!" << endl;
       return ok;
     }

     // initiallize all the link / DH info for the manipulator
     numLinks = b->get(0).asInt();
     LinkTypes.resize(numLinks); LinkTypes.zero();
     DHParams.resize(4,numLinks); DHParams.zero();
     numJoints = 0;

     // instantiate the iKin device
     kinChain = kinLimb.asChain();
     linkList.clear();
     H.eye();

     int offset = 1;
     int lnk_idx = 0;

     for(int i=0; i<numLinks; i++) {
         for(int j=0; j<4; j++) {
             DHParams[j][i] = b->get(offset+j).asDouble();
         }
         LinkTypes[i] = b->get(offset+4).asInt();
         offset+=5;

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

     // set the appropriate size for the Jacobian (just in case its not right)
     inputSize = numJoints;
     J.resize(outputSize,inputSize); J.zero();
     configVals.resize(inputSize); configVals.zero();
     cout << "ManipulatorPositionJacobian::connect() -- got links=" << numLinks << ", dofs=" << numJoints << endl;
    
     // disconnect the param port (we got what we needed from it)
     ok &= Network::disconnect(linkOutputName.c_str(),linkInputName.c_str());
     if(!ok) {
       cout << "ManipulatorPositionJacobian::connectToInputs() -- trouble disconnecting link parameter port!!" << endl;
       return ok;
     }

     paramsSet = true;

     // connect to config port for reading config values
     string configOutputName = "/cb/configuration" + deviceName + "/data:o";
     string configInputName = "/cb/manipulatorpositionjacobian/" + randomIDstr + "/" + deviceName + "/config/vals:i";

     cout << "ManipulatorPositionJacobian::connectToInputs() -- opening input port for configuration values..." << endl;
     ok &= inputPort.open(configInputName.c_str());
     if(!ok) {
       cout << "ManipulatorPositionJacobian::connectToInputs() -- could not open port for configuration values!!" << endl;
       return ok;
     }

     cout << "ManipulatorPositionJacobian::connectToInputs() -- connecting ports for configuration values..." << endl;
     ok &= Network::connect(configOutputName.c_str(),configInputName.c_str(),"udp");
     if(!ok) {
       cout << "ManipulatorPositionJacobian::connectToInputs() -- could not connect ports to read configuration values!!" << endl;
       return ok;
     }

     connectedToInputs = true;   
     return ok;

}
