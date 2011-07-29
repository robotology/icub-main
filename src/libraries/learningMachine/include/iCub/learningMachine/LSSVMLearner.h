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

#ifndef LM_LSSVMLEARNER__
#define LM_LSSVMLEARNER__

#include <vector>
#include <sstream>

#include <yarp/os/IConfig.h>
#include <yarp/sig/Matrix.h>

#include "iCub/learningMachine/IFixedSizeLearner.h"


namespace iCub {
namespace learningmachine {

class Kernel : public yarp::os::IConfig {
private:
    std::string name;
public:
    Kernel(std::string n = "") : name(n) { }

    virtual double evaluate(const yarp::sig::Vector& v1, const yarp::sig::Vector& v2) = 0;

    virtual std::string getConfigHelp() {
        return std::string("Kernel configuration options for '") +
               this->getName() + "'\n";
    }

    virtual std::string getName() {
        return this->name;
    }

    virtual void setName(std::string n) {
        this->name = n;
    }

    virtual std::string getInfo() {
        return this->getName();
    }
};

class RBFKernel : public Kernel {
private:
    double gamma;
public:
    RBFKernel(double g = 1.0) : Kernel("RBF"), gamma(g) {
    }

    virtual ~RBFKernel() {}

    virtual double evaluate(const yarp::sig::Vector& v1, const yarp::sig::Vector& v2);

    virtual void setGamma(double g) {
        this->gamma = g;
    }

    virtual double getGamma() {
        return this->gamma;
    }

    /*
     * Inherited from IConfig.
     */
    virtual bool configure(yarp::os::Searchable& config) {
        bool success = false;
        // format: set c dbl
        if(config.find("gamma").isDouble() || config.find("gamma").isInt()) {
            this->setGamma(config.find("gamma").asDouble());
            success = true;
        }
        return success;
    }

    virtual std::string getConfigHelp() {
        std::ostringstream buffer;
        buffer << this->Kernel::getConfigHelp();
        buffer << "  gamma val             RBF parameter gamma" << std::endl;
        return buffer.str();
    }

    virtual std::string getInfo() {
        std::ostringstream buffer;
        buffer << this->Kernel::getInfo();
        buffer << " gamma: " << this->getGamma();
        return buffer.str();
    }

};

/**
 * \ingroup icub_libLM_learning_machines
 *
 * This is basic implementation of the LSSVM algorithms. Note that for
 * efficiency the hyperparameters are shared among all outputs. Only the RBF
 * kernel function is supported.
 *
 * \see iCub::contrib::IMachineLearner
 * \see iCub::contrib::IFixedSizeLearner
 *
 * \author Arjan Gijsberts
 *
 */
class LSSVMLearner : public IFixedSizeLearner {
private:
    /**
     * Storage for the input vectors.
     */
    std::vector<yarp::sig::Vector> inputs;

    /**
     * Storage for the output vectors.
     */
    std::vector<yarp::sig::Vector> outputs;

    /**
     * The matrix of Lagrange multipliers, i.e. the coefficients.
     */
    yarp::sig::Matrix alphas;

    /**
     * The vector of biases.
     */
    yarp::sig::Vector bias;

    /**
     * The exact Leave-One-Out error on the training data.
     */
    yarp::sig::Vector LOO;

    /**
     * Regularization parameter C.
     */
    double C;

    /**
     * The kernel function.
     */
    RBFKernel* kernel;


public:
    /**
     * Constructor.
     *
     * @param dom initial domain size
     * @param cod initial codomain size
     * @param c initial value for regularization parameter C
     */
    LSSVMLearner(unsigned int dom = 1, unsigned int cod = 1, double c = 1.0);

    /**
     * Copy Constructor.
     */
    LSSVMLearner(const LSSVMLearner& other);

    /**
     * Destructor.
     */
    virtual ~LSSVMLearner();

    /**
     * Assignment operator.
     */
    virtual LSSVMLearner& operator=(const LSSVMLearner& other);

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
    Prediction predict(const yarp::sig::Vector& input);

    /*
     * Inherited from IMachineLearner.
     */
    void reset();

    /*
     * Inherited from IMachineLearner.
     */
    LSSVMLearner* clone();

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

    /*
     * Inherited from IConfig.
     */
    virtual bool configure(yarp::os::Searchable& config);

    /**
     * Mutator for the regularization parameter C.
     *
     * @param C the new value
     */
    virtual void setC(double C) {
        this->C = C;
    }

    /**
     * Accessor for the regularization parameter C.
     *
     * @returns the value of the parameter
     */
    virtual double getC() {
        return this->C;
    }

    /**
     * Accessor for the kernel.
     *
     * @returns a pointer to the kernel
     */
    virtual RBFKernel* getKernel() {
        return this->kernel;
    }
};

} // learningmachine
} // iCub
#endif
