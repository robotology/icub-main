/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Generalized interfaces for fixed size (domain, codomain) learning machines.
 *
 */

#ifndef __ICUB_IFIXEDSIZETRANSFORMER__
#define __ICUB_IFIXEDSIZETRANSFORMER__

#include "iCub/ITransformer.h"

using namespace yarp::os;
using namespace yarp::sig;

namespace iCub {
namespace contrib {
namespace learningmachine {

/**
 *
 * An generalized interface for an ITransformer with a fixed domain
 * and codomain size. Many simple preprocessing methods fall in this category.
 *
 * \see iCub::contrib::learningmachine::ITransformer
 *
 * \author Arjan Gijsberts
 *
 */
class IFixedSizeTransformer : public ITransformer {
protected:
    /**
     * The dimensionality of the input domain.
     */
    int domainSize;

    /**
     * The dimensionality of the output domain (codomain).
     */
    int coDomainSize;

    /**
     * Checks whether the input is of the desired dimensionality.
     *
     * @param input a sample input
     * @return true if the dimensionality is correct
     */
    virtual bool checkDomainSize(const Vector& input) {
        return (input.size() == this->getDomainSize());
    }

    /**
     * Checks whether the output is of the desired dimensionality.
     *
     * @param output a sample output
     * @return true if the dimensionality is correct
     */
    virtual bool checkCoDomainSize(const Vector& output) {
        return (output.size() == this->getCoDomainSize());
    }

    /**
     * Validates whether the input and output are of the desired dimensionality. An
     * exception will be thrown if this is not the case.
     *
     * @param input a sample input
     * @param output the corresponding output
     */
    void validateDomainSizes(const Vector& input, const Vector& output);

public:
    /**
     * Constructor.
     *
     * @param name the name under which this type of learner will be registered
     * @param dSize the initial domain size
     * @param cdSize the initial codomain size
     */
    IFixedSizeTransformer(std::string name, int dSize = 1, int cdSize = 1) 
        : ITransformer(name), domainSize(dSize), coDomainSize(cdSize) {
    }

    /*
     * Inherited from ITransformer.
     */
    virtual void transform(const Vector& input, Vector& output);

    /**
     * Returns the size (dimensionality) of the input domain.
     *
     * @return the size of the input domain
     */
    int getDomainSize() { return this->domainSize; }

    /**
     * Returns the size (dimensionality) of the output domain (codomain).
     *
     * @return the size of the codomain
     */
    int getCoDomainSize() { return this->coDomainSize; }

    /**
     * Mutator for the domain size.
     *
     * @param size the desired domain size
     */
    virtual void setDomainSize(int size) { this->domainSize = size; }

    /**
     * Mutator for the codomain size.
     *
     * @param size the desired codomain size
     */
    virtual void setCoDomainSize(int size) {this->coDomainSize = size; }

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
    virtual bool configure(Searchable& config);

};

} // learningmachine
} // contrib
} // iCub

#endif
