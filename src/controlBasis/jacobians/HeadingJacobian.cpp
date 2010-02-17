// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "HeadingJacobian.h"

using namespace std;
using namespace yarp::sig;

void CB::HeadingJacobian::startJacobian() {

    // if not conencted to inputs, do that now
  if(!connectedToInputs) {
    if(!connectToInputs()) {
      cout << "HeadingJacobian couldn't connect to input ports in startJacobian()..." << endl;
      return;
    }
  }  
  start();     // mandatory start function
}

void CB::HeadingJacobian::stopJacobian() {
  stop();     // mandatory stop function
}

bool CB::HeadingJacobian::updateJacobian() {
    J[0][0] = 1.0;
    J[1][1] = 1.0;
    return true; 
}

bool CB::HeadingJacobian::connectToInputs() {
     connectedToInputs = true;   
     return true;
}
