/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Example code how to use the learningMachine library in a direct manner.
 */

#include <iostream>
#include <math.h>
#include <iCub/LSSVMLearner.h>
#include <iCub/FixedRangeScaler.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Rand.h>

#define INPUT_MIN -10
#define INPUT_MAX  10
#define NO_TRAIN  400
#define NO_TEST   400

using namespace iCub::learningmachine;
using namespace yarp::sig;

std::pair<Vector, Vector> createSample(double min_in, double max_in) {
  std::pair<Vector, Vector> sample;
  sample.first.resize(1);
  sample.second.resize(1);
  double input = yarp::math::Rand::scalar(min_in, max_in);
  sample.first[0] = input;
  sample.second[0] = sin(input);
  return sample;
}

/*
 * This example shows how LearningMachine classes can be used in a direct manner
 * in your code. Please see all direct/indirect/portable examples to have an
 * idea which method suits your application best.
 *
 * Please keep in mind that the purpose is to demonstrate how to interface with
 * the learningMachine library. The synthetic data used in this example is
 * utterly useless.
 */

int main(int argc, char** argv) {
  std::cout << "LearningMachine library example (direct)" << std::endl;

  // one dimensional input, one dimensional output, c = 1.0
  LSSVMLearner lssvm = LSSVMLearner(1, 1, 2.);
  lssvm.getKernel()->setGamma(32.);

  // normalizer that scales [-10,10] -> [-1,1]
  // IScalers only do R^1 -> R^1, for higher dimensions use ScaleTransformer
  FixedRangeScaler scaler = FixedRangeScaler(INPUT_MIN, INPUT_MAX);

  // create and feed training samples
  for(int i = 0; i < NO_TRAIN; i++) {
    std::pair<Vector, Vector> sample = createSample(INPUT_MIN, INPUT_MAX);
    sample.first[0] = scaler.transform(sample.first[0]);
    lssvm.feedSample(sample.first, sample.second);
  }

  // train the machine on the data (it's a batch machine!)
  lssvm.train();

  // feed test samples
  double MSE = 0.;
  for(int i = 0; i < NO_TEST; i++) {
    std::pair<Vector, Vector> sample = createSample(INPUT_MIN, INPUT_MAX);
    sample.first[0] = scaler.transform(sample.first[0]);
    Vector prediction = lssvm.predict(sample.first);
    double diff = sample.second[0] - prediction[0];
    MSE += diff * diff;
  }
  MSE /= NO_TEST;
  std::cout << "MSE on test data after " << NO_TEST << " samples: " << MSE << std::endl;

}
