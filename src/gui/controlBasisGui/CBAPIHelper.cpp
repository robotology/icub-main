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
    sequenceControllerParameters[0].push_back(new ControllerParameters(sen, ref, pf, eff, gain));
  } else {
    sen = "/cb/" + sen;
    pf = "/cb/" + pf;
    eff = "/cb/" + eff;
    sequenceControllerParameters[0].push_back(new ControllerParameters(sen, pf, eff, gain));
  }
  sequenceControllerParameters[1].push_back(NULL);

  cout << "ADDING CONTROLLER TO SEQUENCE:" << endl;
  cout << "\tsen: " << sen << endl;
  cout << "\tref: " << ref << endl;
  cout << "\teff: " << eff << endl;
  cout << "\tpf: " << pf << endl;

  numControllersInSequence++;
  sequenceControlLaw.useTranspose(useTranspose);
  sequenceControlLaw.usePDControl(useDerivativeTerm); 


}


bool CB::CBAPIHelper::addSecondaryControllerToSequence(string sen, string ref, string pf, string eff, bool useTranspose, double gain) {


    if(sequenceControllerParameters[1][sequenceControllerParameters[1].size()-1] != NULL) {
        cout << "can't add secondary controller to sequence, one already exists!!" << endl;
        return false;
    }

    if(ref != "") {
        sen = "/cb/" + sen;
        ref = "/cb/" + ref;
        pf = "/cb/" + pf;
        eff = "/cb/" + eff;
        sequenceControllerParameters[1][sequenceControllerParameters[1].size()-1] = new ControllerParameters(sen, ref, pf, eff, gain);
    } else {
        sen = "/cb/" + sen;
        pf = "/cb/" + pf;
        eff = "/cb/" + eff;
        sequenceControllerParameters[1][sequenceControllerParameters[1].size()-1] = new ControllerParameters(sen, pf, eff, gain);
    }
    
    cout << "ADDING SECONDARY CONTROLLER TO SEQUENCE:" << endl;
    cout << "\tsen: " << sen << endl;
    cout << "\tref: " << ref << endl;
    cout << "\teff: " << eff << endl;
    cout << "\tpf: " << pf << endl;
    
    sequenceControlLaw.useTranspose(useTranspose);
    sequenceControlLaw.usePDControl(useDerivativeTerm); 

    return true;
}

void CB::CBAPIHelper::clearControlLaw() { 
  controlLaw.deleteControllers();
  numControllersInLaw = 0;
}

void CB::CBAPIHelper::clearSequence() { 
  sequenceControlLaw.deleteControllers();
  numControllersInSequence = 0;
  sequenceIndex = -1;

  for(int k=0; k<2; k++) {
      for(int i=0; i<sequenceControllerParameters[k].size(); i++) {
          delete sequenceControllerParameters[k][i];
          sequenceControllerParameters[k][i] = NULL;
      }
      sequenceControllerParameters[k].clear();
  }
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

int CB::CBAPIHelper::getNumberOfControllersInSequenceStep() {   
    return sequenceControlLaw.getNumControllers();
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
    if(sequenceControllerParameters[0][sequenceIndex]->Reference=="") {
        sequenceControlLaw.addController(sequenceControllerParameters[0][sequenceIndex]->Sensor, 
                                         sequenceControllerParameters[0][sequenceIndex]->PotentialFunction,
                                         sequenceControllerParameters[0][sequenceIndex]->Effector, 
                                         sequenceControllerParameters[0][sequenceIndex]->Gain);
        if(sequenceControllerParameters[1][sequenceIndex] != NULL) {
            sequenceControlLaw.addController(sequenceControllerParameters[1][sequenceIndex]->Sensor, 
                                             sequenceControllerParameters[1][sequenceIndex]->PotentialFunction,
                                             sequenceControllerParameters[1][sequenceIndex]->Effector, 
                                             sequenceControllerParameters[1][sequenceIndex]->Gain);
        }
    } else {
        sequenceControlLaw.addController(sequenceControllerParameters[0][sequenceIndex]->Sensor, 
                                         sequenceControllerParameters[0][sequenceIndex]->Reference,
                                         sequenceControllerParameters[0][sequenceIndex]->PotentialFunction,
                                         sequenceControllerParameters[0][sequenceIndex]->Effector, 
                                         sequenceControllerParameters[0][sequenceIndex]->Gain);
        if(sequenceControllerParameters[1][sequenceIndex] != NULL) {
            sequenceControlLaw.addController(sequenceControllerParameters[1][sequenceIndex]->Sensor, 
                                             sequenceControllerParameters[1][sequenceIndex]->Reference,
                                             sequenceControllerParameters[1][sequenceIndex]->PotentialFunction,
                                             sequenceControllerParameters[1][sequenceIndex]->Effector, 
                                             sequenceControllerParameters[1][sequenceIndex]->Gain);
        }
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
    if(sequenceControllerParameters[0][sequenceIndex]->Reference=="") {
        sequenceControlLaw.addController(sequenceControllerParameters[0][sequenceIndex]->Sensor, 
                                         sequenceControllerParameters[0][sequenceIndex]->PotentialFunction,
                                         sequenceControllerParameters[0][sequenceIndex]->Effector, 
                                         sequenceControllerParameters[0][sequenceIndex]->Gain);
        if(sequenceControllerParameters[1][sequenceIndex] != NULL) {
            sequenceControlLaw.addController(sequenceControllerParameters[1][sequenceIndex]->Sensor, 
                                             sequenceControllerParameters[1][sequenceIndex]->PotentialFunction,
                                             sequenceControllerParameters[1][sequenceIndex]->Effector, 
                                             sequenceControllerParameters[1][sequenceIndex]->Gain);
        }
    } else {
        sequenceControlLaw.addController(sequenceControllerParameters[0][sequenceIndex]->Sensor, 
                                         sequenceControllerParameters[0][sequenceIndex]->Reference,
                                         sequenceControllerParameters[0][sequenceIndex]->PotentialFunction,
                                         sequenceControllerParameters[0][sequenceIndex]->Effector, 
                                         sequenceControllerParameters[0][sequenceIndex]->Gain);
        if(sequenceControllerParameters[1][sequenceIndex] != NULL) {
            sequenceControlLaw.addController(sequenceControllerParameters[1][sequenceIndex]->Sensor, 
                                             sequenceControllerParameters[1][sequenceIndex]->Reference,
                                             sequenceControllerParameters[1][sequenceIndex]->PotentialFunction,
                                             sequenceControllerParameters[1][sequenceIndex]->Effector, 
                                             sequenceControllerParameters[1][sequenceIndex]->Gain);
        }
    }  
    sequenceControlLaw.usePDControl(useDerivativeTerm);

    // start it
    sequenceControlLaw.startAction();
}
