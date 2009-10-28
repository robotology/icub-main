/*
 * Copyright (C) 2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * *TEMPORARY* Regularized Least Squares learner.
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

#ifndef __ICUB_RLSLEARNER__
#define __ICUB_RLSLEARNER__

#include <string>
#include <vector>
#include <sstream>

#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include "iCub/IFixedSizeLearner.h"


using namespace yarp::sig;
using namespace yarp::math;
//using namespace iCub::contrib::liblssvm;

namespace iCub {
namespace contrib {
namespace learningmachine {

/**
 *
 *
 * \see iCub::contrib::IMachineLearner
 * \see iCub::contrib::IFixedSizeLearner
 *
 * \author Arjan Gijsberts
 *
 */

class RLS {
private:
    /**
     * Inverse of matrix A.
     */
    Matrix Ai;
    /**
     * Vector b.
     */
    Vector b;
    /**
     * Weight vector for the linear predictor.
     */
    Vector w;
    /**
     * Regularization constant.
     */
    double lambda;
    /**
     * Number of inputs.
     */
    int n;

    void writeBottle(Bottle& bot) {
        // TO IMPLEMENT
    }

    void readBottle(Bottle& bot) {
        // TO IMPLEMENT
    }

public:
    RLS(int n = 1.) {
        this->lambda = 1.0;
        this->n = n;
        this->reset();
    }

    void reset() {
        this->b = zeros(this->n);
        this->w = zeros(this->n);
        this->Ai = zeros(this->n, this->n);
        Vector diagonal(this->n);
        diagonal = (1. / this->lambda);
        this->Ai.diagonal(diagonal);
    }
  
    double predict(const Vector& x) {
        return dot(x, this->w);
    }
  
    void update(const Vector& x, double y) {
        this->b = this->b + x * y;
        
        Vector Aix = this->Ai * x;
        Vector xAi = x * this->Ai;
        double s = 1.0 / (1.0 + dot(xAi, x));
        // in python: self.Ai -= (numpy.outer(s * Aix, xAi))
        // in C++, however... :(
        for(int i = 0; i < this->Ai.rows(); i++) {
            double sAixi =  s * Aix(i);
            for(int j = 0; j < this->Ai.cols(); j++) {
                this->Ai(i, j) -= sAixi * xAi(j);
            }
        }

        this->w = this->Ai * this->b;
    }
    
    void setLambda(double l) {
        this->lambda = l;
    }
      
    double getLambda() {
        return this->lambda;
    }

    /**
     * Asks the RLS to return a string serialization.
     *
     * @return a string serialization of the scaler
     */
    virtual std::string toString() {
        Bottle model;
        this->writeBottle(model);
        return model.toString().c_str();
    }

    /**
     * Asks the RLS to initialize from a string serialization.
     *
     * @return true on succes
     */
    virtual bool fromString(const std::string& str) {
        Bottle model(str.c_str()); 
        this->readBottle(model);
        return true;
    }
};

class RLSLearner : public IFixedSizeLearner {
private:

    /**
     * The vector of RLS machines; one for each output element.
     */
    std::vector<RLS*> machines;

    /**
     * number of samples during last training routine
     */
    int sampleCount;

    /**
     * Resets the vector of machines and deletes each element.
     */
    void deleteAll();

    /**
     * Resets the vector of machines to the given size and deletes each element.
     *
     * @param size the desired size of the vector after resetting.
     */
    void deleteAll(int size);

    /**
     * Deletes a machine at the given index.
     *
     * @param index the index of the element.
     */
    void deleteAt(int index);

    /**
     * Initiates the vector of machines and resets each element before doing that.
     */
    void initAll();

    /**
     * Initiates the vector of machines to the given size and resets each element before doing that.
     *
     * @param size the desired size of the vector after intiation.
     */
    void initAll(int size);

    /**
     * Creates a new machine according to the currently stored type specifier.
     *
     * @return a machine of the desired type.
     */
    RLS* createMachine();

public:
    /**
     * Constructor.
     *
     * @param dom initial domain size
     * @param cod initial codomain size
     * @param lambda initial value for regularization parameter \lambda
     */
    RLSLearner(int dom = 1, int cod = 1, double lambda = 1.0);

    /**
     * Destructor.
     */
    ~RLSLearner();

    /*
     * Inherited from IMachineLearner.
     */
    virtual void feedSample(const Vector& input, const Vector& output);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void train();

    /**
     * Returns a pointer to the RLS at a certain position.
     *
     * @param index the index of the scaler
     */
    RLS* getAt(int index);

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

    /**
     * Sets the regularization parameter lambda of all machines to a specified value.
     *
     * @param l the desired value.
     */
    void setLambdaAll(double l);

    /**
     * Sets the regularization parameter lambda of the machine at a given index to a specified value.
     *
     * @param index the index of the element.
     * @param l the desired value.
     */
    void setLambdaAt(int index, double l);

    /*
     * Inherited from IConfig.
     */
    virtual bool configure(Searchable& config);
};

} // learningmachine
} // contrib
} // iCub
#endif
