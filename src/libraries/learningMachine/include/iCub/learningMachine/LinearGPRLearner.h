/*
 * Copyright (C) 2007-2011 RobotCub Consortium, European Commission FP6 Project IST-004370
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

#ifndef LM_LINEARGPRLEARNER__
#define LM_LINEARGPRLEARNER__

#include <string>
#include <vector>

#include <yarp/sig/Matrix.h>

#include "iCub/learningMachine/IFixedSizeLearner.h"


namespace iCub {
namespace learningmachine {


/**
 * \ingroup icub_libLM_learning_machines
 *
 * Standard linear Bayesian regression or, equivalently, Gaussian Process
 * Regression with a linear covariance function. It uses a rank 1 update rule to
 * incrementally update the Cholesky factor of the covariance matrix.
 *
 * See:
 * Gaussian Processes for Machine Learning.
 *   Carl Edward Rasmussen and Christopher K. I. Williams.
 *   The MIT Press, 2005.
 *
 * Pattern Recognition and Machine Learning.
 *   Christopher M. Bishop.
 *   Springer-Verlag, 2006.
 *
 *
 * \see iCub::learningmachine::IMachineLearner
 * \see iCub::learningmachine::IFixedSizeLearner
 * \see iCub::learningmachine::RLSLearner
 *
 * \author Arjan Gijsberts
 *
 */

class LinearGPRLearner : public IFixedSizeLearner {
private:
    /**
     * Cholesky factor of the covariance matrix.
     */
    yarp::sig::Matrix R;

    /**
     * Matrix B.
     */
    yarp::sig::Matrix B;

    /**
     * Weight matrix for the linear predictor.
     */
    yarp::sig::Matrix W;

    /**
     * Signal noise.
     */
    double sigma;

    /**
     * Number of samples during last training routine
     */
    int sampleCount;

public:
    /**
     * Constructor.
     *
     * @param dom initial domain size
     * @param cod initial codomain size
     * @param sigma initial value for signal noise \sigma
     */
    LinearGPRLearner(unsigned int dom = 1, unsigned int cod = 1, double sigma = 1.0);

    /**
     * Copy constructor.
     */
    LinearGPRLearner(const LinearGPRLearner& other);

    /**
     * Destructor.
     */
    virtual ~LinearGPRLearner();

    /**
     * Assignment operator.
     */
    LinearGPRLearner& operator=(const LinearGPRLearner& other);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void feedSample(const yarp::sig::Vector& input, const yarp::sig::Vector& output);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void train();

    /*
     * Inherited from IMachineLearner.
     */
    virtual Prediction predict(const yarp::sig::Vector& input);

    /*
     * Inherited from IMachineLearner.
     */
    void reset();

    /*
     * Inherited from IMachineLearner.
     */
    LinearGPRLearner* clone() {
        return new LinearGPRLearner(*this);
    }

    /*
     * Inherited from IMachineLearner.
     */
    virtual std::string getInfo();

    /*
     * Inherited from IMachineLearner.
     */
    virtual std::string getConfigHelp();

    /*
     * Inherited from IMachineLearner.
     */
    virtual void writeBottle(yarp::os::Bottle& bot);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void readBottle(yarp::os::Bottle& bot);

    /*
     * Inherited from IFixedSizeLearner.
     */
    void setDomainSize(unsigned int size);

    /*
     * Inherited from IFixedSizeLearner.
     */
    void setCoDomainSize(unsigned int size);

    /**
     * Sets the signal noise \sigma to a specified value. This resets the
     * machine.
     *
     * @param s the desired value.
     */
    void setSigma(double s);

    /**
     * Accessor for the signal noise \sigma.
     *
     * @returns the value of the parameter
     */
    double getSigma();

    /*
     * Inherited from IConfig.
     */
    virtual bool configure(yarp::os::Searchable& config);

};

} // learningmachine
} // iCub
#endif
