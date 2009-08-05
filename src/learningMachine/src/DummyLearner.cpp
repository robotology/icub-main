/*
 * Copyright (C) 2007-2008 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Generalized Interface for Learning Machines (both offline and online)
 *
 */

#include "iCub/DummyLearner.h"

namespace iCub {
namespace contrib {
namespace learningmachine {

// just here for debugging, since Vector.toString() cannot be applied to const Vector :(
std::string printVector(const Vector& v) {
  std::ostringstream output;
  output << "[";
  for(int i = 0; i < v.size(); i++) {
    if(i > 0) output << ",";
    output << v[i];
  }
  output << "]";
  return output.str();
}

} // learningmachine
} // contrib
} // iCub
