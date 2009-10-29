/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Linear scaling transformer.
 *
 */

#ifndef __ICUB_RANDOMFEATURE__
#define __ICUB_RANDOMFEATURE__

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Bottle.h>
#include <yarp/math/Math.h>

#include "iCub/IFixedSizeTransformer.h"

using namespace yarp::os;
using namespace yarp::sig;

namespace iCub {
namespace learningmachine {


/**
 * Implementation of Random Feature preprocessing.
 *
 * See:
 * Random Features for Large-Scale Kernel Machines. Ali Rahimi and Ben Recht. 
 *   In Neural Information Processing Systems (NIPS) 2007.
 *
 * \author Arjan Gijsberts
 *
 */

class RandomFeature : public IFixedSizeTransformer {
protected:
    /**
     * Gamma parameter, analoguous to same parameter in RBF kernel.
     */
    double gamma;
    /**
     * Projection matrix W.
     */
    Matrix W;
    /**
     * Bias vector b.
     */
    Vector b;

    /*
     * Inherited from ITransformer.
     */
    virtual void writeBottle(Bottle& bot);

    /*
     * Inherited from ITransformer.
     */
    virtual void readBottle(Bottle& bot);

public:
    /**
     * Constructor.
     *
     * @param dom initial domain size
     * @param cod initial codomain size
     * @param g initial value for \gamma
     */
    RandomFeature(int dom = 1, int cod = 1, double gamma = 1.);

    /**
     * Destructor.
     */
    ~RandomFeature();

    /**
     * Feeds a single sample into the scaler, so that it can use this sample
     * to learn the appropriate offset and scale.
     *
     * @param value the sample value
     */
    //virtual void feedSample(const Vector& in);

    /*
     * Inherited from ITransformer.
     */
    virtual ITransformer* create() {
        return new RandomFeature();
    }

    /*
     * Inherited from ITransformer.
     */
    virtual void transform(const Vector& input, Vector& output);

    /*
     * Inherited from ITransformer.
     */
    virtual std::string getInfo();

    /*
     * Inherited from ITransformer.
     */
    virtual std::string getConfigHelp();

    /*
     * Inherited from ITransformer.
     */
    //virtual void setDomainSize(int size);

    /*
     * Inherited from IFixedSizeTransformer.
     */
    virtual void setCoDomainSize(int size);

    /*
     * Inherited from ITransformer.
     */
    virtual void reset();

    /*
     * Inherited from IConfig.
     */
    virtual bool configure(Searchable &config);
    
    /**
     * Accessor for the gamma parameter.
     *
     * @return gamma.
     */
    virtual double getGamma() {
        return this->gamma;
    }
    
    /**
     * Mutator for the gamma paramter.
     *
     * @param g the desired gamma.
     */
    virtual void setGamma(double g) {
        this->gamma = g;
        // rebuild projection matrix
        this->reset();
    }

protected:

};


} // learningmachine
} // iCub

#endif
