// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006, 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#include "Sequence.h"

#include <math.h>


bool Sequence::extend(int len) {
  if (future.size()>=len) {
    return true;
  }
  future.clear();
  if (past.size()>=2) {
    double v1 = past[past.size()-1];
    double dv1 = v1-past[past.size()-2];
    for (int i=0; i<len; i++) {
      double v0 = 0;
      double dv0 = 1e6;
      int idx = 0;
      for (int k=1; k<past.size()-1; k++) {
	double v2 = past[k];
	double dv2 = v2-past[k-1];
	if (fabs(v2-v1)<fabs(v0-v1)
	    && fabs(dv2-dv1)<fabs(dv0-dv1)) {
	  v0 = v2;
	  dv0 = dv2;
	  idx = k;
	}
      }
      idx++;
      v0 = past[idx];
      dv0 = v0-past[idx-1];
      future.push_back(v0);
      v1 = v0;
      dv1 = dv0;
      ///cerr << v1 << " " << dv1 << endl;
    }
  } else {
    for (int i=0; i<len; i++) {
      if (past.size()>=1) {
	future.push_back(past[past.size()-1]);
      } else {
	future.push_back(0);
      }
    }
  }
  return future.size()>=len;
}



