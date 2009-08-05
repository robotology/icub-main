// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006, 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#include "ConstantSequence.h"

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>

using namespace std;
using namespace boost;

double ConstantSequence::predict(int index) {
  extend(index+1);
  mt19937 rng;
  double mean = h.getMean();
  double dev = h.getDeviation();
  if (dev<0.0001) {
    return mean;
  }
  normal_distribution<double> dist(mean,dev);
  variate_generator<mt19937&, normal_distribution<double> > 
    normal_sampler(rng, dist);
  double v = normal_sampler();
  return v;
}
