// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-    
/*
\author Shiraj Sen and Stephen Hart

Copyright (C) 2010 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.
**/
#ifndef __NONPARAMETRICDISTRIBUTION_H__
#define __NONPARAMETRICDISTRIBUTION_H__

#include <vector>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include "ProbabilityDistribution.h"

class ProbabilityDensityFunction {
    
public:

    double val;
    std::vector<int> index; 
    
    ProbabilityDensityFunction() {}
    ~ProbabilityDensityFunction() {}
    
    void clear() { index.clear(); }

};

class NonParametricProbabilityDistribution : public ProbabilityDistribution {

protected:

    int dim_count;
    int sorted;
    int first;
    
    std::vector<ProbabilityDensityFunction> distribution;
    std::vector<ProbabilityDensityFunction> sorted_distribution;
    std::vector<ProbabilityDensityFunction> cumulative_distribution;
    
    yarp::sig::Vector range_min;
    yarp::sig::Vector range_max;
    yarp::sig::Vector no_of_bins;
    yarp::sig::Vector bin_size;
    yarp::sig::Matrix covar;
    yarp::sig::Matrix covar_inv;

    int data_size;
    double _total_sample_count;
    double covar_det;
    double alpha;
    double max_val;
  
    bool _needsCleaning;
    int _count;

public:

    NonParametricProbabilityDistribution(int count);

    ~NonParametricProbabilityDistribution();

    int addSample(yarp::sig::Vector sample, double val);

    int drawSample(yarp::sig::Vector &sample);

    double getProbability(yarp::sig::Vector sample);

    void setDimensionParameters(yarp::sig::Vector min, yarp::sig::Vector max, yarp::sig::Vector bins);    

    int getDimensionCount();

    int loadDistribution(std::string str);

    int saveDistribution(std::string str);

    void normalizeAndRescale(int scale);

private:

    static const double THRESHOLD = 0.3;
    static const int NORMALIZE_RATE = 10;

    void sort();
    
    void getNewSample(yarp::sig::Vector &sample);
        
    void setIndexParameters();
    
    void setIndex(int pt, int *index);

    void getIndex(int pt, int *index);

    void getBinCenter(int pt, yarp::sig::Vector &center);

    void computeCovariance();

    double computeGaussianProbability(yarp::sig::Vector mean, yarp::sig::Vector sample);
    
};

#endif
