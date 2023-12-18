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

#ifndef LM_RANDOMFEATURE__
#define LM_RANDOMFEATURE__

#include <yarp/sig/Matrix.h>

#include "iCub/learningMachine/IFixedSizeTransformer.h"

namespace iCub {
namespace learningmachine {


/**
 * \ingroup icub_libLM_transformers
 *
 * Implementation of Random Feature preprocessing.
 *
 * \see iCub::learningmachine::SparseSpectrumFeature
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
    yarp::sig::Matrix W;

    /**
     * Bias vector b.
     */
    yarp::sig::Vector b;

    /*
     * Inherited from ITransformer.
     */
    virtual void writeBottle(yarp::os::Bottle& bot) const;

    /*
     * Inherited from ITransformer.
     */
    virtual void readBottle(yarp::os::Bottle& bot);

public:
    /**
     * Constructor.
     *
     * @param dom initial domain size
     * @param cod initial codomain size
     * @param g initial value for \gamma
     */
    RandomFeature(unsigned int dom = 1, unsigned int cod = 1, double gamma = 1.);

    /**
     * Destructor (empty).
     */
    virtual ~RandomFeature() { }

    /*
     * Inherited from ITransformer.
     */
    virtual RandomFeature* clone() {
        return new RandomFeature(*this);
    }

    /*
     * Inherited from ITransformer.
     */
    virtual yarp::sig::Vector transform(const yarp::sig::Vector& input);

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
    virtual void setDomainSize(unsigned int size);

    /*
     * Inherited from IFixedSizeTransformer.
     */
    virtual void setCoDomainSize(unsigned int size);

    /*
     * Inherited from ITransformer.
     */
    virtual void reset();

    /*
     * Inherited from ITransformer.
     */
    virtual bool configure(yarp::os::Searchable &config);

    /**
     * Accessor for the gamma parameter.
     *
     * @return gamma.
     */
    virtual double getGamma() const {
        return this->gamma;
    }

    /**
     * Mutator for the gamma parameter.
     *
     * @param g the desired gamma.
     */
    virtual void setGamma(double g) {
        this->gamma = g;
        // rebuild projection matrix
        this->reset();
    }

};


} // learningmachine
} // iCub

#endif
