// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "ManipulatorOrientationJacobian.h"
#include <yarp/os/Network.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iKin;

void CB::ManipulatorOrientationJacobian::startJacobian() {
    // if not conencted to inputs, do that now
    if(!connectedToInputs) {
        if(!connectToInputs()) {
            cout << "ManipulatorOrientationJacobian couldn't connect to input ports in startJacobian()..." << endl;
            return;
        }
    }  
    start();     // mandatory start function
}

void CB::ManipulatorOrientationJacobian::stopJacobian() {
  stop();     // mandatory stop function
}

bool CB::ManipulatorOrientationJacobian::updateJacobian() {
  
  bool ok = true;
  Bottle *b;
  int nj;
  int offset;
  Matrix Jfull;
  Matrix Jtmp(J.rows(),J.cols());
              
  // get the current configuration of the manipulator
  b = inputPort.read();
  if(b==NULL) {
      cout << "ManipulatorOrientationJacobian::update() -- having trouble reading input data!" << endl;
      return ok;
  }
  nj = b->get(0).asInt();
  inputSize = nj;

  // check sizes, reinitialize if there is a mismatch
  if(configVals.size() != nj) {
    configVals.resize(nj); configVals.zero();
    J.resize(outputSize,inputSize); J.zero();
    cout << "ManipulatorOrientationJacobian size mismatch, resizing=" << inputSize << endl;
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
  
  Matrix T = kinChain->getH();
  Matrix R = T.submatrix(0,2,0,2);

  Vector Vxyz = rotationToXYZEuler(R);
  Matrix Jxyz = computeEulerJacobian(Vxyz);

  // copy the Jacobian to class var
  //  J.resize(3,Jfull.cols());
  for(int i=0; i<Jtmp.rows(); i++) {      
      for(int j=0; j<Jtmp.cols(); j++) {        
          Jtmp[i][j] = Jfull[i+3][j];
      }
  }

  J = Jxyz*Jtmp;

  /*
  cout << "ORIENTATION JACOBIAN:\n-------------------------\n" << endl;
  cout << "T:" << endl;
  for(int i=0; i<4; i++) {
      for(int j=0; j<4; j++) {
          cout << T[i][j] << " ";
      }
      cout << endl;
  }
  cout << endl;

  cout << "R:" << endl;
  for(int i=0; i<3; i++) {
      for(int j=0; j<3; j++) {
          cout << R[i][j] << " ";
      }
      cout << endl;
  }
  cout << endl;

  cout << "Vxyz:" << endl;
  for(int i=0; i<Vxyz.size(); i++) {
      cout << Vxyz[i] << endl;
  }
  cout << endl;

  cout << "Jfull:" << endl;
  for(int i=0; i<Jfull.rows(); i++) {
      for(int j=0; j<Jfull.cols(); j++) {
          cout << Jfull[i][j] << " ";
      }
      cout << endl;
  }
  cout << endl;

  cout << "Jtmp:" << endl;
  for(int i=0; i<Jtmp.rows(); i++) {
      for(int j=0; j<Jtmp.cols(); j++) {
          cout << Jtmp[i][j] << " ";
      }
      cout << endl;
  }
  cout << endl;

  cout << "Jxyz:" << endl;
  for(int i=0; i<Jxyz.rows(); i++) {
      for(int j=0; j<Jxyz.cols(); j++) {
          cout << Jxyz[i][j] << " ";
      }
      cout << endl;
  }
  cout << endl;

  cout << "J:" << endl;
  for(int i=0; i<J.rows(); i++) {
      for(int j=0; j<J.cols(); j++) {
          cout << J[i][j] << " ";
      }
      cout << endl;
  }
  cout << endl;
  */

  return ok;
  
}

bool CB::ManipulatorOrientationJacobian::connectToInputs() {

     bool ok = true;
     Bottle *b;
     int numLinks;
     int numJoints;
     Matrix H(4,4);

     if(connectedToInputs) return true;

     // configue a random ID for this jacobian instance 
     int randomID = (int)(Random::uniform()*1000.0);
     char *c = (char *)malloc(16);
     sprintf(c,"%d", randomID);
     string randomIDstr(c);

     // connect to link port for configuration resource
     string linkOutputName = "/cb/configuration" + deviceName + "/params:o";
     string linkInputName = "/cb/manipulatororientationjacobian/" + randomIDstr + deviceName + "/config/params:i";
     
     cout << "ManipulatorOrientationJacobian::connectToInputs() -- opening input port for link parameters..." << endl;
     ok &= paramPort.open(linkInputName.c_str());
     if(!ok) {
       cout << "ManipulatorOrientationJacobian::connectToInputs() -- could not open port for configuration link parameters!!" << endl;
       return ok;
     }

     cout << "ManipulatorOrientationJacobian::connectToInputs() -- connecting ports for link parameters..." << endl;
     ok &= Network::connect(linkOutputName.c_str(),linkInputName.c_str(), "tcp");
     if(!ok) {
       cout << "ManipulatorOrientationJacobian::connectToInputs() -- could not connect to configuration link parameters!!" << endl;
       return ok;
     }
     
     // get Link information
     b = paramPort.read();
     if(b==NULL) {
       cout << "ManipulatorOrientationJacobian::connectToInputs() -- could not read link information!!" << endl;
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
     cout << "ManipulatorOrientationJacobian::connect() -- got links=" << numLinks << ", dofs=" << numJoints << endl;
    
     // disconnect the param port (we got what we needed from it)
     ok &= Network::disconnect(linkOutputName.c_str(),linkInputName.c_str());
     if(!ok) {
       cout << "ManipulatorOrientationJacobian::connectToInputs() -- trouble disconnecting link parameter port!!" << endl;
       return ok;
     }

     paramsSet = true;

     // close parameter port now that we have gotten all the relevent information
     paramPort.close();

     // connect to config port for reading config values
     string configOutputName = "/cb/configuration" + deviceName + "/data:o";
     string configInputName = "/cb/manipulatororientationjacobian/" + randomIDstr + deviceName + "/config/vals:i";

     cout << "OPEN PORT: ManipulatorOrientationJacobian::connectToInputs() -- opening input port for configuration values..." << endl;
     ok &= inputPort.open(configInputName.c_str());
     if(!ok) {
       cout << "ManipulatorOrientationJacobian::connectToInputs() -- could not open port for configuration values!!" << endl;
       return ok;
     }

     cout << "ManipulatorOrientationJacobian::connectToInputs() -- connecting ports for configuration values..." << endl;
     ok &= Network::connect(configOutputName.c_str(),configInputName.c_str(),"tcp");
     if(!ok) {
       cout << "ManipulatorOrientationJacobian::connectToInputs() -- could not connect ports to read configuration values!!" << endl;
       return ok;
     }

     connectedToInputs = true;   
     return ok;

}

Vector CB::ManipulatorOrientationJacobian::rotationToXYZEuler(const yarp::sig::Matrix &R) {
    double roll, pitch, yaw;
    double cosPitch;

    Vector V(3);
    
    pitch = atan2(-R[2][0], sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0]));
    cosPitch = cos(pitch);
    
    yaw = atan2(R[1][0] / cosPitch, R[0][0] / cosPitch);
    roll = atan2(R[2][1] / cosPitch, R[2][2] / cosPitch);
    
    V[0] = roll;
    V[1] = pitch;
    V[2] = yaw;

    return V;
}


Matrix CB::ManipulatorOrientationJacobian::computeEulerJacobian(const yarp::sig::Vector &V) {

    double alpha,gamma,beta;
    Matrix J(3,3);

    if(V.size() != 3) {
        cout << "ManipulatorOrientationJacobian::computeEulerJacobian() -- invalid size of input Vector!" << endl;
        exit(-1);
    }

    alpha = V[0];
    beta  = V[1];
    gamma = V[2];

    J[0][0] = cos(gamma) * cos(beta);
    J[1][0] = sin(gamma) * cos(beta);
    J[2][0] = -sin(beta);

    J[0][1] = -sin(gamma);
    J[1][1] = cos(gamma);
    J[2][1] = 0.0;

    J[0][2] = 0.0;
    J[1][2] = 0.0;
    J[2][2] = 1.0;

    return J;
}
