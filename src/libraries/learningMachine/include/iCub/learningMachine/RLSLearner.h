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

#ifndef LM_RLSLEARNER__
#define LM_RLSLEARNER__

#include <yarp/sig/Matrix.h>

#include "iCub/learningMachine/IFixedSizeLearner.h"


namespace iCub {
namespace learningmachine {


/**
 * \ingroup icub_libLM_learning_machines
 *
 * Recursive Regularized Least Squares (a.k.a. ridge regression) learner. It
 * uses a rank 1 update rule to update the Cholesky factor of the covariance
 * matrix.
 *
 * \see iCub::learningmachine::IMachineLearner
 * \see iCub::learningmachine::IFixedSizeLearner
 * \see iCub::learningmachine::LinearGPRLearner
 *
 * \author Arjan Gijsberts
 *
 */

class RLSLearner : public IFixedSizeLearner {
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
     * Number of samples during last training routine
     */
    int sampleCount;

    /**
     * Regularization parameter.
     */
    double lambda;

public:
    /**
     * Constructor.
     *
     * @param dom initial domain size
     * @param cod initial codomain size
     * @param lambda initial value for regularization parameter \lambda
     */
    RLSLearner(unsigned int dom = 1, unsigned int cod = 1, double lambda = 1.0);

    /**
     * Copy constructor.
     */
    RLSLearner(const RLSLearner& other);

    /**
     * Destructor.
     */
    virtual ~RLSLearner();

    /**
     * Assignment operator.
     */
    RLSLearner& operator=(const RLSLearner& other);

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
    RLSLearner* clone() {
        return new RLSLearner(*this);
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
     * Sets the regularization parameter \lambda to a specified value. This
     * resets the machine.
     *
     * @param l the desired value.
     */
    void setLambda(double l);

    /**
     * Accessor for the regularization parameter \lambda.
     *
     * @returns the value of the parameter
     */
    double getLambda();

    /*
     * Inherited from IConfig.
     */
    virtual bool configure(yarp::os::Searchable& config);
};

} // learningmachine
} // iCub
#endif
