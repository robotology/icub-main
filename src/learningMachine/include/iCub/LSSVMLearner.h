/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Least Squares Support Vector Machine (LSSVM) based learner 
 *
 */

/**
 *
 * An implementation of the LSSVM learning machine for use with the IMachineLearner
 * interface.
 *
 * \see iCub::contrib::IMachineLearner
 * \see iCub::contrib::IFixedSizeLearner
 *
 * \author Arjan Gijsberts
 *
 */

#ifndef __ICUB_LSSVMLEARNER__
#define __ICUB_LSSVMLEARNER__

#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <cassert>
#include <math.h>

#include <yarp/os/Bottle.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include "iCub/IFixedSizeLearner.h"

using namespace yarp;
using namespace yarp::math;

namespace iCub {
namespace contrib {
namespace learningmachine {

class Kernel : public IConfig {
private:
    std::string name;
public:
    Kernel(std::string name) {
        this->name = name;
    }

    virtual double evaluate(const Vector& v1, const Vector& v2) = 0;

    virtual std::string getConfigHelp() {
        return std::string("Kernel configuration options for '") + 
               this->getName() + "'\n"; 
    }
    
    virtual std::string getName() {
        return this->name;
    }
    
    virtual std::string getInfo() {
        return this->getName();
    }
};

class RBFKernel : public Kernel {
private:
    double gamma;
public:
    RBFKernel(std::string name = "RBF", double gamma = 1.0) : Kernel(name), gamma(gamma) {
    }
    
    ~RBFKernel() {}

    virtual double evaluate(const Vector& v1, const Vector& v2) {
        assert(v1.size() == v2.size());
        double result = 0.0;
        double diff;
        
        for(int i = 0; i < v1.size(); i++) {
            diff = v1(i) - v2(i);
            result += diff * diff;
        }
        result *= -1 * this->gamma;
        return exp(result);
    }
    
    virtual void setGamma(double g) {
        this->gamma = g;
    }
    
    virtual double getGamma() {
        return this->gamma;
    }

    /*
     * Inherited from IConfig.
     */
    virtual bool configure(Searchable& config) {
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
    std::vector<Vector> inputs;

    /**
     * Storage for the output vectors.
     */
    std::vector<Vector> outputs;
    
    /**
     * The matrix of Lagrange multipliers, i.e. the coefficients.
     */
    Matrix alphas;
    
    /**
     * The vector of biases.
     */
    Vector bias;
    
    /**
     * The exact Leave-One-Out error on the training data.
     */
    Vector LOO;

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
     * @param name the name under which this type of learner will be registered
     */
    LSSVMLearner(std::string name = "LSSVM");

    /**
     * Destructor.
     */
    ~LSSVMLearner();

    /*
     * Inherited from IMachineLearner.
     */
    virtual void feedSample(const Vector& input, const Vector& output);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void train();

    /*
     * Inherited from IMachineLearner.
     */
    Vector predict(const Vector& input);

    /*
     * Inherited from IMachineLearner.
     */
    void reset();

    /*
     * Inherited from IMachineLearner.
     */
    IMachineLearner* create();

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
    virtual void writeBottle(Bottle& bot);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void readBottle(Bottle& bot);

    /*
     * Inherited from IFixedSizeLearner.
     */
    void setDomainSize(int size);

    /*
     * Inherited from IFixedSizeLearner.
     */
    void setCoDomainSize(int size);

    /*
     * Inherited from IConfig.
     */
    virtual bool configure(Searchable& config);

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
};

} // learningmachine
} // contrib
} // iCub
#endif
