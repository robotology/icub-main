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

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Bottle.h>
#include <yarp/math/Math.h>

#include "iCub/learningMachine/IFixedSizeTransformer.h"

using namespace yarp::os;
using namespace yarp::sig;

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
 *   Anibal R. Figueiras-Vidal. In Journal of Machine Learning Research (JMLR)
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
    Vector ell;

    /**
     * Projection matrix W.
     */
    Matrix W;

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
     * @param sigma initial value for sigma
     * @param ell initial value for \ell
     */
    //SparseSpectrumFeature(unsigned int dom = 1, unsigned int cod = 1, Vector* ell = (Vector*) 0x0);
    SparseSpectrumFeature(unsigned int dom = 1, unsigned int cod = 1, double sigma = 1., Vector ell = Vector(0));

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
    virtual Vector transform(const Vector& input);

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
    virtual bool configure(Searchable &config);

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
    virtual double getEll() {
        return this->ell;
    }

    /**
     * Mutator for the characteristic length-scales parameter ell.
     *
     * @param ell the desired ell.
     */
    virtual void setEll(Vector& ell) {
        this->ell = ell;
        // rebuild projection matrix
        this->reset();
    }

};


} // learningmachine
} // iCub

#endif
