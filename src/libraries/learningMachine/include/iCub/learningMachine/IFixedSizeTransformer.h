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

#ifndef LM_IFIXEDSIZETRANSFORMER__
#define LM_IFIXEDSIZETRANSFORMER__

#include "iCub/learningMachine/ITransformer.h"


namespace iCub {
namespace learningmachine {

/**
 * \ingroup icub_libLM_transformers
 *
 * An generalized interface for an ITransformer with a fixed domain
 * and codomain size. Many simple preprocessing methods fall in this category.
 *
 * \see iCub::learningmachine::ITransformer
 *
 * \author Arjan Gijsberts
 *
 */
class IFixedSizeTransformer : public ITransformer {
protected:
    /**
     * The dimensionality of the input domain.
     */
    unsigned int domainSize;

    /**
     * The dimensionality of the output domain (codomain).
     */
    unsigned int coDomainSize;

    /**
     * Checks whether the input is of the desired dimensionality.
     *
     * @param input a sample input
     * @return true if the dimensionality is correct
     */
    virtual bool checkDomainSize(const yarp::sig::Vector& input) {
        return ((unsigned int) input.size() == this->getDomainSize());
    }

    /**
     * Checks whether the output is of the desired dimensionality.
     *
     * @param output a sample output
     * @return true if the dimensionality is correct
     */
    virtual bool checkCoDomainSize(const yarp::sig::Vector& output) {
        return ((unsigned int) output.size() == this->getCoDomainSize());
    }

    /**
     * Validates whether the input and output are of the desired dimensionality. An
     * exception will be thrown if this is not the case.
     *
     * @param input a sample input
     * @param output the corresponding output
     */
    void validateDomainSizes(const yarp::sig::Vector& input, const yarp::sig::Vector& output);

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
     * @param dom the initial domain size
     * @param cod the initial codomain size
     */
    IFixedSizeTransformer(unsigned int dom = 1, unsigned int cod = 1) : domainSize(dom), coDomainSize(cod) { }

    /*
     * Inherited from ITransformer.
     */
    virtual yarp::sig::Vector transform(const yarp::sig::Vector& input);

    /**
     * Returns the size (dimensionality) of the input domain.
     *
     * @return the size of the input domain
     */
    unsigned int getDomainSize() {
        return this->domainSize;
    }

    /**
     * Returns the size (dimensionality) of the output domain (codomain).
     *
     * @return the size of the codomain
     */
    unsigned int getCoDomainSize() {
        return this->coDomainSize;
    }

    /**
     * Mutator for the domain size.
     *
     * @param size the desired domain size
     */
    virtual void setDomainSize(unsigned int size) {
        this->domainSize = size;
    }

    /**
     * Mutator for the codomain size.
     *
     * @param size the desired codomain size
     */
    virtual void setCoDomainSize(unsigned int size) {
        this->coDomainSize = size;
    }

    /*
     * Inherited from ITransformer.
     */
    virtual std::string getInfo();

    /*
     * Inherited from ITransformer.
     */
    virtual std::string getConfigHelp();

    /*
     * Inherited from IConfig.
     */
    virtual bool configure(yarp::os::Searchable& config);

};

} // learningmachine
} // iCub

#endif
