// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-    
#ifndef __PROBABILITYDISTRIBUTION_H__
#define __PROBABILITYDISTRIBUTION_H__

#include <yarp/sig/Vector.h>
#include <string>

/**
 * ProbabilityDistribution Class : This class is an abstract interface to a PDF sensor that provides distributions to sample from or to build new distributions based on experience.
 **/

class ProbabilityDistribution {
  
 private:
  
  int dimension_count;

 public:

  ProbabilityDistribution(int count) {
    dimension_count = count;
  }

  /**
   * Destructor
   **/
  ~ProbabilityDistribution() {};
  
  virtual int getDimensionCount() = 0;

  virtual int addSample(yarp::sig::Vector dimension, double val) = 0;

  virtual int drawSample(yarp::sig::Vector &sample) = 0;
  
  virtual int loadDistribution(std::string str) = 0;
  
  virtual int saveDistribution(std::string str) = 0;
};

#endif
