/*
 * Copyright (C) 2007-2009 Arjan Gijsberts 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Example code how to use the learningMachine library using the portable
 * wrapper.
 */
#include <iostream>
#include <cmath>
#include <stdio.h>
#include "iCub/learningMachine/MachinePortable.h"
#include "iCub/learningMachine/MachineCatalogue.h"
#include "iCub/learningMachine/TransformerPortable.h"
#include "iCub/learningMachine/TransformerCatalogue.h"

#include <yarp/sig/Vector.h>
#include <yarp/os/Property.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <yarp/os/Time.h>

#define MIN(a, b)   ((a < b) ? a : b)

#define NO_TRAIN    4000
#define NO_TEST     8000
#define NOISE_MIN  -0.05
#define NOISE_MAX   0.05


using namespace iCub::learningmachine;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

// taken from LWPR example code
double cross(double x1, double x2) {
  x1 *= x1;
  x2 *= x2;
  double a = std::exp(-10 * x1);
  double b = std::exp(-50 * x2);
  double c = 1.25 * std::exp(-5 * (x1 + x2));
  return (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c);
}

double sin2d(double x1, double x2) {
  return std::sin(x1 + x2);
}

void elementProd(const Vector& v1, Vector& v2) {
  for(int i = 0; i < MIN(v1.size(), v2.size()); i++) {
    v2[i] = v1[i] * v2[i];
  }
}

Vector elementDiv(const Vector& v, double d) {
  Vector ret(v.size());
  for(int i = 0; i < v.size(); i++) {
    ret[i] = (d == 0.) ? v[i] : v[i] / d;
  }
  return ret;
}


std::pair<Vector, Vector> createSample() {
  std::pair<Vector, Vector> sample;
  sample.first.resize(2);
  sample.second.resize(2);
  sample.first[0] = Rand::scalar(-1, +1);
  sample.first[1] = Rand::scalar(-1, +1);
  sample.second[0] = sin2d(sample.first[0], sample.first[1]);
  sample.second[1] = cross(sample.first[0], sample.first[1]);
  return sample;
}

/*
 * This example shows how LearningMachine classes can be used in a indirect
 * manner in your code. In this context, this means that the YARP configuration
 * mechanism is used and instances are of the abstract base type. This
 * facilitates easy migration to other learning methods. Please see all
 * direct/indirect/portable examples to have an idea which method suits your
 * application best.
 *
 * Please keep in mind that the purpose is to demonstrate how to interface with
 * the learningMachine library. The synthetic data used in this example is
 * utterly useless.
 */

int main(int argc, char** argv) {
  int nrf = 250;
  Vector trainMSE(2);
  Vector testMSE(2);
  Vector noise_min(2);
  Vector noise_max(2);
  // train + test time for transformer and machine
  double tic;
  double trtrtime = 0.0;
  double trtetime = 0.0;
  double mctrtime = 0.0;
  double mctetime = 0.0; 

  if(argc > 1) sscanf(argv[1], "%d", &nrf);

  // initialize catalogue of machine factory
  registerMachines();
  // initialize catalogue of transformers
  registerTransformers();

  std::cout << "LearningMachine library example (portable)" << std::endl;

  // create Regularized Least Squares learner
  std::string name("RLS");
  MachinePortable mp = MachinePortable("RLS");
  Property p;
  p.put("dom", Value(nrf));
  p.put("cod", Value(2));
  p.put("lambda", Value(0.5));
  mp.getWrapped().configure(p);
  std::cout << "Learner:" << std::endl << mp.getWrapped().getInfo() << std::endl;

  // create Random Feature transformer
  TransformerPortable tp = TransformerPortable("RandomFeature");
  p.clear();
  p.put("dom", Value(2));
  p.put("cod", Value(nrf));
  p.put("gamma", Value(16.0));
  tp.getWrapped().configure(p);
  std::cout << "Transformer:" << std::endl << tp.getWrapped().getInfo() << std::endl;


  // create and feed training samples
  noise_min = NOISE_MIN;
  noise_max = NOISE_MAX;
  
  
  trainMSE = 0.0;
  for(int i = 0; i < NO_TRAIN; i++) {
    // create a new training sample
    std::pair<Vector, Vector> sample = createSample();

    // add some noise to output for training
    Vector noisyOutput = sample.second + Rand::vector(noise_min, noise_max);

    // transform input using RF
    tic = yarp::os::Time::now();
    Vector transInput = tp.getWrapped().transform(sample.first);
    trtrtime += (yarp::os::Time::now() - tic);

    // make prediction before feeding full sample
    tic = yarp::os::Time::now();
    Prediction prediction = mp.getWrapped().predict(transInput);

    // train on complete sample with noisy output
    mp.getWrapped().feedSample(transInput, noisyOutput);
    mctrtime += (yarp::os::Time::now() - tic);

    Vector diff = prediction.getPrediction() - sample.second;
    elementProd(diff, diff);
    trainMSE = trainMSE + diff;
  }
  trainMSE = elementDiv(trainMSE, NO_TRAIN);
  std::cout << "Train MSE: " << trainMSE.toString() << std::endl;
  std::cout << "Train Transformer Time per Sample: " << (trtrtime / NO_TRAIN) << std::endl;
  std::cout << "Train Machine Time per Sample: " << (mctrtime / NO_TRAIN) << std::endl;
  std::cout << "Combined Time per Sample: " << ((trtrtime + mctrtime) / NO_TRAIN) << std::endl;

  std::cout << std::endl;
  std::cout << "Saving machine portable to file 'mp.txt'...";
  bool ok = mp.writeToFile("mp.txt");
  std::cout << ((ok) ? "ok!" : "failed :(") << std::endl;

  std::cout << "Saving transformer portable to file 'tp.txt'...";
  ok = tp.writeToFile("tp.txt");
  std::cout << ((ok) ? "ok!" : "failed :(") << std::endl;

  std::cout << "Loading machine portable from file 'mp.txt'...";
  ok = mp.readFromFile("mp.txt");
  std::cout << ((ok) ? "ok!" : "failed :(") << std::endl;

  std::cout << "Loading transformer portable from file 'tp.txt'...";
  ok = tp.readFromFile("tp.txt");
  std::cout << ((ok) ? "ok!" : "failed :(") << std::endl;
  std::cout << std::endl;

  // predict test samples
  testMSE = 0.;
  for(int i = 0; i < NO_TEST; i++) {
    // create a new testing sample
    std::pair<Vector, Vector> sample = createSample();

    // transform input using RF
    tic = yarp::os::Time::now();
    Vector transInput = tp.getWrapped().transform(sample.first);
    trtetime += (yarp::os::Time::now() - tic);

    // make prediction
    tic = yarp::os::Time::now();
    Prediction prediction = mp.getWrapped().predict(transInput);
    mctetime += (yarp::os::Time::now() - tic);
    Vector diff = prediction.getPrediction() - sample.second;
    elementProd(diff, diff);
    //std::cout << "Sample: " << sample.input <<
    testMSE = testMSE + diff;
  }
  testMSE = elementDiv(testMSE, NO_TEST);
  std::cout << "Test MSE: " << testMSE.toString() << std::endl;
  std::cout << "Test Transformer Time per Sample: " << (trtetime / NO_TEST) << std::endl;
  std::cout << "Test Machine Time per Sample: " << (mctetime / NO_TEST) << std::endl;
  std::cout << "Combined Time per Sample: " << ((trtetime + mctetime) / NO_TEST) << std::endl;

  return 0;
}
