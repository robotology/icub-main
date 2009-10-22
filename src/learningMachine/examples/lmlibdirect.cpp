#include <iostream>
#include <math>
#include <iCub/LSSVMLearner.h>
#include <iCub/Normalizer.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Rand.h>

#define INPUT_MIN -10
#define INPUT_MAX  10
#define NO_TRAIN  100
#define NO_TEST    50

using namespace iCub::contrib::learningmachine;
using namespace yarp::sig;

/*
 * This example shows how LearningMachine classes can be used in a direct manner
 * in your code.
 */
std::pair<Vector, Vector> createSample(double min_in, double max_in) {
  std::pair<Vector, Vector> sample;
  sample.first.resize(1);
  sample.second.resize(1);
  sample.first[0] = yarp::math::Rand::scalar(min_in, max_in);
  sample.second[0] = 
  
}


int main(int argc, char** argv) {
  std::cout << "LearningMachine library example (direct)" << std::endl;

  // one dimensional input, one dimensional output, c = 1.0
  LSSVMLearner* lssvm = new LSSVMLearner(1, 1, 1.0);
  
  // normalizer that scales [-10,10] -> [-1,1]
  // IScalers only do R^1 -> R^1, for higher dimensions use ScaleTransformer
  Normalizer* normalizer = new Normalizer(INPUT_MIN, INPUT_MAX);
  
  // create and feed training samples
  for(int i = 0; i < NO_TRAIN; i++) {
    std::pair<Vector, Vector> sample = createSample(INPUT_MIN, INPUT_MAX);
  }
  
  machine->train();
  
  
  delete machine;
  delete transformer;
}
