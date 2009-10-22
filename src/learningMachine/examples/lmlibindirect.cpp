/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Example code how to use the learningMachine library in a indirect manner.
 */
#include <iostream>
#include <math.h>
#include <iCub/RLSLearner.h>
#include <iCub/RandomFeature.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Property.h>
#include <yarp/math/Rand.h>

#define MIN(a, b)  (a > b) ? a : b

#define NO_TRAIN   1000
#define NO_TEST    1000
#define NOISE_MIN -0.05
#define NOISE_MAX  0.05


using namespace iCub::contrib::learningmachine;
using namespace yarp::sig;
using namespace yarp::math;

// taken from LWPR example code
double cross(double x1, double x2) {
  x1 *= x1;
  x2 *= x2;
  double a = exp(-10 * x1);
  double b = exp(-50 * x2);
  double c = 1.25 * exp(-5 * (x1 + x2));
  return (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c);
}

double sin2d(double x1, double x2) {
  return sin(x1 + x2);
}

void elementProd(const Vector& v1, Vector& v2) {
  std::cout << "v1: " << v1.size() << std::endl;
 
  std::cout << "v2: " << v2.size() << std::endl;
  std::cout << "MIN(v1.size(), v2.size()): " << MIN(v1.size(), v2.size()) << std::endl;
  for(int i = 0; i < MIN(v1.size(), v2.size()); i++) {
    std::cout << "i: " << i << std::endl;
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
  std::cout << "LearningMachine library example (indirect)" << std::endl;

  IMachineLearner* rls = new RLSLearner();
  Property p("(dom 250) (cod 2) (lambda (2.0 3.0))");
  rls->configure(p);
  std::cout << "Learner:" << std::endl << rls->getInfo() << std::endl;
  
  ITransformer* rf = new RandomFeature();
  p.fromString("(dom 2) (cod 250) (gamma 1.0)", true);
  rf->configure(p);
  std::cout << "Transformer:" << std::endl << rf->getInfo() << std::endl;

  // create and feed training samples
  Vector noise_min(2);
  noise_min = NOISE_MIN;
  Vector noise_max(2);
  noise_max = NOISE_MAX;
  Vector MSE(2);
  MSE = 0.0;
  for(int i = 0; i < NO_TRAIN; i++) {
    std::pair<Vector, Vector> sample = createSample();

    // add some noise for training
    Vector input = sample.first + Rand::vector(noise_min, noise_max);
    rf->transform(input, sample.first);

    // training errors based just on input
    Vector prediction = rls->predict(sample.first);

    Vector diff = prediction - sample.second;
    elementProd(diff, diff);
    MSE = MSE + diff;

    // train on complete sample
    rls->feedSample(sample.first, sample.second);
  }
  MSE = elementDiv(MSE, NO_TRAIN);
  std::cout << "Train MSE: " << MSE.toString() << std::endl;
/*  
  // train the machine on the data (it's a batch machine!)
  lssvm->train();
  
  // feed test samples
  double MSE = 0.;
  for(int i = 0; i < NO_TEST; i++) {
    std::pair<Vector, Vector> sample = createSample(INPUT_MIN, INPUT_MAX);
    sample.first[0] = scaler->transform(sample.first[0]);
    Vector prediction = lssvm->predict(sample.first);
    double diff = sample.second[0] - prediction[0];
    MSE += diff * diff;
  }
  MSE /= NO_TEST;
  std::cout << "MSE on test data after " << NO_TEST << " samples: " << MSE << std::endl;
*/
  delete rls;
  delete rf;
}
