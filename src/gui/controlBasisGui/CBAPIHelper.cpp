// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "CBAPIHelper.h"
#include <yarp/os/Time.h>

using namespace std;
using namespace yarp::os;

CB::CBAPIHelper::CBAPIHelper() {
  numControllersInLaw = 0;
  numControllersInSequence = 0;
  sequenceIndex = -1;
  useDerivativeTerm = false;
}

void CB::CBAPIHelper::addControllerToLaw(string sen, string ref, string pf, string eff, bool useTranspose, double gain) {

  if(ref != "") {
    sen = "/cb/" + sen;
    ref = "/cb/" + ref;
    pf = "/cb/" + pf;
    eff = "/cb/" + eff;
    controlLaw.addController(sen, ref, pf, eff, gain);
  } else {
    sen = "/cb/" + sen;
    pf = "/cb/" + pf;
    eff = "/cb/" + eff;
    controlLaw.addController(sen, pf, eff, gain);
  }

  cout << "ADDING CONTROLLER:" << endl;
  cout << "\tsen: " << sen << endl;
  cout << "\tref: " << ref << endl;
  cout << "\teff: " << eff << endl;
  cout << "\tpf: " << pf << endl;

  numControllersInLaw++;
  controlLaw.useTranspose(useTranspose);
  controlLaw.usePDControl(useDerivativeTerm);

}

void CB::CBAPIHelper::addControllerToSequence(string sen, string ref, string pf, string eff, bool useTranspose, double gain) {

  if(ref != "") {
    sen = "/cb/" + sen;
    ref = "/cb/" + ref;
    pf = "/cb/" + pf;
    eff = "/cb/" + eff;
    sequenceControllerParameters.push_back(new ControllerParameters(sen, ref, pf, eff, gain));
  } else {
    sen = "/cb/" + sen;
    pf = "/cb/" + pf;
    eff = "/cb/" + eff;
    sequenceControllerParameters.push_back(new ControllerParameters(sen, pf, eff, gain));
  }

  cout << "ADDING CONTROLLER TO SEQUENCE:" << endl;
  cout << "\tsen: " << sen << endl;
  cout << "\tref: " << ref << endl;
  cout << "\teff: " << eff << endl;
  cout << "\tpf: " << pf << endl;

  numControllersInSequence++;
  sequenceControlLaw.useTranspose(useTranspose);
  sequenceControlLaw.usePDControl(useDerivativeTerm); 
}

void CB::CBAPIHelper::clearControlLaw() { 
  controlLaw.deleteControllers();
  numControllersInLaw = 0;
}

void CB::CBAPIHelper::clearSequence() { 
  sequenceControlLaw.deleteControllers();
  numControllersInSequence = 0;
  sequenceIndex = -1;

  for(int i=0; i<sequenceControllerParameters.size(); i++) {
      delete sequenceControllerParameters[i];
  }
  sequenceControllerParameters.clear();

}

void CB::CBAPIHelper::stopControlLaw() {
    controlLaw.stopAction();
}

void CB::CBAPIHelper::stopSequence() {
    sequenceIndex=-1;
    sequenceControlLaw.stopAction();
}

void CB::CBAPIHelper::runControlLaw() {
    controlLaw.startAction();
}

void CB::CBAPIHelper::runSequence() {
    cout << "running sequence controller[" << sequenceIndex << "]" << endl;
    goToNextControllerInSequence();
}

double CB::CBAPIHelper::getPotential(int n, bool sequence=false) {
    if(sequence) {
        return sequenceControlLaw.getControllerPotential(n);
    } else {
        return controlLaw.getControllerPotential(n);
    }
}

double CB::CBAPIHelper::getPotentialDot(int n, bool sequence=false) {
    if(sequence) {
        return sequenceControlLaw.getControllerPotentialDot(n);
    } else {
        return controlLaw.getControllerPotentialDot(n);
    }
}

int CB::CBAPIHelper::getState(int n, bool sequence=false) {
    if(sequence) {
        return (int)sequenceControlLaw.getControllerState(n);
    } else {
        return (int)controlLaw.getControllerState(n);
    }
}

void CB::CBAPIHelper::useTranspose(bool b) {
    sequenceControlLaw.useTranspose(b);
    controlLaw.useTranspose(b); 
}

void CB::CBAPIHelper::usePDControl(bool b) {
    useDerivativeTerm = b;
    sequenceControlLaw.usePDControl(b);
    controlLaw.usePDControl(b); 
}

int CB::CBAPIHelper::getSequenceControllerID() {
    return sequenceIndex;
}

void CB::CBAPIHelper::goToNextControllerInSequence() {

    if(sequenceIndex==(numControllersInSequence-1)) {
        cout << "no more controllers in the sequence" << endl;
        return;
    }

    // check to see if there is a previous controller running
    // index is at -1 if sequence is not running
    if(sequenceIndex != -1) {
        sequenceControlLaw.stopAction();
        cout << "controller[" << sequenceIndex << "] finished" << endl;
        sequenceControlLaw.deleteControllers();
    }

    // move to the next controller in the store
    sequenceIndex++;

    // add it to the control law
    if(sequenceControllerParameters[sequenceIndex]->Reference=="") {
        sequenceControlLaw.addController(sequenceControllerParameters[sequenceIndex]->Sensor, 
                                         sequenceControllerParameters[sequenceIndex]->PotentialFunction,
                                         sequenceControllerParameters[sequenceIndex]->Effector, 
                                         sequenceControllerParameters[sequenceIndex]->Gain);
    } else {
        sequenceControlLaw.addController(sequenceControllerParameters[sequenceIndex]->Sensor, 
                                         sequenceControllerParameters[sequenceIndex]->Reference,
                                         sequenceControllerParameters[sequenceIndex]->PotentialFunction,
                                         sequenceControllerParameters[sequenceIndex]->Effector, 
                                         sequenceControllerParameters[sequenceIndex]->Gain);
    }  
    sequenceControlLaw.usePDControl(useDerivativeTerm);

    // start it
    sequenceControlLaw.startAction();

}

void CB::CBAPIHelper::goToPreviousControllerInSequence() {

    if(sequenceIndex==0) {
        cout << "no previous controllers in the sequence" << endl;
        return;
    }

    // check to see if there is a previous controller running
    if(sequenceIndex > -1) {
        sequenceControlLaw.stopAction();
        cout << "controller[" << sequenceIndex << "] finished" << endl;
        sequenceControlLaw.deleteControllers();
    }

    // move to the next controller in the store
    sequenceIndex--;

    // add it to the control law
    if(sequenceControllerParameters[sequenceIndex]->Reference=="") {
        sequenceControlLaw.addController(sequenceControllerParameters[sequenceIndex]->Sensor, 
                                         sequenceControllerParameters[sequenceIndex]->PotentialFunction,
                                         sequenceControllerParameters[sequenceIndex]->Effector, 
                                         sequenceControllerParameters[sequenceIndex]->Gain);
    } else {
        sequenceControlLaw.addController(sequenceControllerParameters[sequenceIndex]->Sensor, 
                                         sequenceControllerParameters[sequenceIndex]->Reference,
                                         sequenceControllerParameters[sequenceIndex]->PotentialFunction,
                                         sequenceControllerParameters[sequenceIndex]->Effector, 
                                         sequenceControllerParameters[sequenceIndex]->Gain);
    }  
    sequenceControlLaw.usePDControl(useDerivativeTerm);

    // start it
    sequenceControlLaw.startAction();
}
