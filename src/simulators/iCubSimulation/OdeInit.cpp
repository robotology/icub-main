// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "OdeInit.h"

OdeInit *OdeInit::_odeinit = NULL;

OdeInit& OdeInit::get() {
    if (_odeinit==NULL) {
        _odeinit=new OdeInit;
    }
    return *_odeinit;
}

void OdeInit::destroy() {
  if (_odeinit!=NULL) {
    delete _odeinit;
    _odeinit = NULL;
  }
}

