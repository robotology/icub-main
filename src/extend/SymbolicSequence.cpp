// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006, 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#include "SymbolicSequence.h"

#include "fetch.h"


bool SymbolicSequence::extend(int len) {
  if (future.size()>=len) {
    return true;
  }

  //cerr << "symbolic extension of degree " << len << endl;
  future = extendSequence(past,len);
  return future.size()>=len;
}


