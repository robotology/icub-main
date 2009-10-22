#include <iostream>
#include <iCub/LSSVMLearner.h>

using namespace iCub::contrib::learningmachine;

int main(int argc, char** argv) {
  std::cout << "LearningMachine library example (direct)" << std::endl;
  LSSVMLearner* machine = new LSSVMLearner(0.2);
  
}
