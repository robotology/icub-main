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

#ifndef LM_SPARSESPECTRUMFEATURE__
#define LM_SPARSESPECTRUMFEATURE__

#include <yarp/sig/Matrix.h>

#include "iCub/learningMachine/IFixedSizeTransformer.h"

namespace iCub {
namespace learningmachine {


/**
 * \ingroup icub_libLM_transformers
 *
 * Implementation of Sparse Spectrum preprocessing.
 *
 * \see iCub::learningmachine::RandomFeature
 *
 * See:
 * Sparse Spectrum Gaussian Process Regression. Miguel Lazaro-Gredilla,
 *   Joaquin Quinonero-Candela, Carl Edward Rasmussen, and
 *   Anibal R. Figueiras-Vidal. In Journal of Machine Learning Research (JMLR),
 *   2010.
 *
 * \author Arjan Gijsberts
 *
 */

class SparseSpectrumFeature : public IFixedSizeTransformer {
protected:
    /**
     * Noise parameter sigma.
     */
    double sigma;

    /**
     * Characteristic length-scales ell, analoguous to same parameter in
     * asymmetric RBF kernel.
     */
    yarp::sig::Vector ell;

    /**
     * Projection matrix W.
     */
    yarp::sig::Matrix W;

    /*
     * Inherited from ITransformer.
     */
    virtual void writeBottle(yarp::os::Bottle& bot);

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
     * @param sigma initial value for sigma
     * @param ell initial value for \ell
     */
    SparseSpectrumFeature(unsigned int dom = 1, unsigned int cod = 2, double sigma = 1.,
                          yarp::sig::Vector ell = yarp::sig::Vector(0));

    /**
     * Destructor (empty).
     */
    virtual ~SparseSpectrumFeature() { }

    /*
     * Inherited from ITransformer.
     */
    virtual SparseSpectrumFeature* clone() {
        return new SparseSpectrumFeature(*this);
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
     * Inherited from IConfig.
     */
    virtual bool configure(yarp::os::Searchable &config);

    /**
     * Accessor for the sigma parameter.
     *
     * @return sigma.
     */
    virtual double getSigma() {
        return this->sigma;
    }

    /**
     * Mutator for the sigma parameter.
     *
     * @param s the desired sigma.
     */
    virtual void setSigma(double s) {
        this->sigma = s;
    }

    /**
     * Accessor for the characteristic length-scales parameter ell.
     *
     * @return ell.
     */
    virtual yarp::sig::Vector& getEll() {
        return this->ell;
    }

    /**
     * Mutator for the characteristic length-scales parameter ell.
     *
     * @param ell the desired ell.
     */
    virtual void setEll(yarp::sig::Vector& ell);

};


} // learningmachine
} // iCub

#endif
