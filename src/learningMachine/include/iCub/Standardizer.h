/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Standardization transformer to preprocess data samples.
 *
 */

#ifndef __ICUB_STANDARDIZER__
#define __ICUB_STANDARDIZER__

#include "iCub/IScaler.h"

using namespace yarp::os;

namespace iCub {
namespace contrib {
namespace learningmachine {


/**
 *
 * A class that implements standardization as a preprocessing step. 
 * Standardization is the process of converting all samples such that the set 
 * has a zero mean and unit standard deviation. In this particular 
 * implementation both the mean and standard deviation are calculated using a 
 * 'running' computation.
 *
 * \see iCub::contrib::learningmachine::IScaler
 *
 * \author Arjan Gijsberts
 *
 */

class Standardizer : public IScaler {
protected:
    /**
     * The number of samples that have been received so far.
     */
    int noSamples;

    /**
     * Temporary variable that counts the sum of the squared errors.
     */
    double squaredErrors;
    
    /**
     * Desired mean for the output distribution.
     */
    double mean;
    
    /**
     * Desired standard deviation for the output distribution.
     */
    double std;
    
    /**
     * Running mean based on the samples seen so far.
     */
    double runningMean;

    /**
     * Running standard deviation based on the samples seen so far.
     */
    double runningStd;

    /*
     * Inherited from IScaler
     */
    virtual void writeBottle(Bottle& bot);

    /*
     * Inherited from IScaler
     */
    virtual void readBottle(Bottle& bot);

public:
    /**
     * Constructor.
     *
     * @param m desired output mean
     * @param s desired output standard deviation
     */
    Standardizer(double m = 0., double s = 1.);

    /*
     * Inherited from IScaler
     */
    virtual void update(double val);

    /*
     * Inherited from IScaler
     */
    virtual std::string getInfo();
    
    /*
     * Inherited from IConfig
     */
    virtual bool configure(Searchable& config);

    /*
     * Inherited from IScaler
     */
    IScaler* create() {
        return new Standardizer(*this);
    }
    
    /**
     * Accessor for the desired mean.
     */
    virtual double getDesiredMean() { return this->mean; }

    /**
     * Mutator for the desired mean.
     *
     * @param m the new mean
     */
    virtual void setDesiredMean(double m) { this->mean = m; }

    /**
     * Accessor for the desired standard deviation.
     */
    virtual double getDesiredStd() { return this->std; }

    /**
     * Mutator for the desired standard deviation.
     *
     * @param s the new standard deviation
     */
    virtual void setDesiredStd(double s) { this->std = s; }
};


} // learningmachine
} // contrib
} // iCub

#endif
