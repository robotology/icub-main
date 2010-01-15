/*
 * Copyright (C) 2007-2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * author:  Arjan Gijsberts
 * email:   arjan.gijsberts@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef LM_STANDARDIZER__
#define LM_STANDARDIZER__

#include "iCub/IScaler.h"

using namespace yarp::os;

namespace iCub {
namespace learningmachine {


/**
 * \ingroup icub_libLM_transformers
 *
 * A class that implements standardization as a preprocessing step.
 * Standardization is the process of converting all samples such that the set
 * has a zero mean and unit standard deviation. In this particular
 * implementation both the mean and standard deviation are calculated using a
 * 'running' computation.
 *
 * \see iCub::learningmachine::IScaler
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
    Standardizer* clone() {
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
} // iCub

#endif
