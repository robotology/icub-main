// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-    
#include "NonParametricProbabilityDistribution.h"
#include <yarp/sig/Vector.h>
#include <stdio.h>

using namespace std;
using namespace yarp::sig;

int main(int argc, char *argv[]) {

  NonParametricProbabilityDistribution param(2);

  Vector range_min(2);
  Vector range_max(2);
  Vector bins(2);

  range_min[0] = 1.0;
  range_min[1] = 1.0;

  range_max[0] = 13.0;
  range_max[1] = 13.0;

  bins[0] = 48;
  bins[1] = 48;

  param.setDimensionParameters(range_min,range_max,bins);

  Vector new_sample(2);
  Vector test_sample(2);
  Vector drawn_sample(2);

  new_sample[0] = 4.2;
  new_sample[1] = 4.5;
  param.addSample(new_sample, 1.0);

  new_sample[0] = 8.2;
  new_sample[1] = 8.5;
  param.addSample(new_sample, 1.0);

  new_sample[0] = 8.2;
  new_sample[1] = 6.5;
  param.addSample(new_sample, 1.0);

  //  param.normalizeAndRescale();

  test_sample[0] = 8.2;
  test_sample[1] = 3.5;
  double p = param.getProbability(test_sample);
  printf("prob: %f\n", p);

  for(int i=0; i<10; i++) {
    param.drawSample(drawn_sample);
    printf("sample: [%f  %f]\n", drawn_sample[0], drawn_sample[1]);
    usleep(1000000);
  }


  param.saveDistribution("dist.dat");
  

  return 1;
}
