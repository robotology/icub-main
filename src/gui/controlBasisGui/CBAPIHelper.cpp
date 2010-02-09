// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "CBAPIHelper.h"

using namespace std;

CB::CBAPIHelper::CBAPIHelper() {
  numControllers = 0;
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
  numControllers++;
  controlLaw.useTranspose(useTranspose);
 
}

void CB::CBAPIHelper::clearControlLaw() {
  controlLaw.stopAction();
  controlLaw.resetControlLaw();
  numControllers = 0;
}

void CB::CBAPIHelper::stopControlLaw() {
  controlLaw.stopAction();
}

void CB::CBAPIHelper::runControlLaw() {
  controlLaw.startAction();
}

double CB::CBAPIHelper::getPotential(int n) {
  return controlLaw.getControllerPotential(n);
}

double CB::CBAPIHelper::getPotentialDot(int n) {
  return controlLaw.getControllerPotentialDot(n);
}

int CB::CBAPIHelper::getState(int n) {
  return (int)controlLaw.getControllerState(n);
}

void CB::CBAPIHelper::useTranspose(bool b) {
    controlLaw.useTranspose(b);
}
