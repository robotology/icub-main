/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Generalized interfaces for fixed size (domain, codomain) learning machines.
 *
 */

#ifndef __ICUB_IFIXEDSIZELEARNER__
#define __ICUB_IFIXEDSIZELEARNER__

#include "iCub/IMachineLearner.h"

using namespace yarp::os;
using namespace yarp::sig;

namespace iCub {
namespace learningmachine {

/**
 *
 * An generalized interface for a learning machine with a fixed domain
 * and codomain size. Many machine learning techniques will fall in this 
 * category.
 *
 * The learning machine can be used for regression and classification
 * from R^M to R^N. M and N are here fixed sizes which should be respected
 * by the samples that are fed into the system.
 *
 * \see iCub::contrib::IMachineLearner
 *
 * \author Arjan Gijsberts
 *
 */
class IFixedSizeLearner : public IMachineLearner {
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
    virtual bool checkDomainSize(const Vector& input);

    /**
     * Checks whether the output is of the desired dimensionality.
     *
     * @param output a sample output
     * @return true if the dimensionality is correct
     */
    virtual bool checkCoDomainSize(const Vector& output);

    /**
     * Validates whether the input and output are of the desired dimensionality. An
     * exception will be thrown if this is not the case.
     *
     * @param input a sample input
     * @param output the corresponding output
     */
    void validateDomainSizes(const Vector& input, const Vector& output);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void writeBottle(Bottle& bot);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void readBottle(Bottle& bot);

public:
    /**
     * Constructor.
     *
     * @param dom the initial domain size
     * @param cod the initial codomain size
     */
    IFixedSizeLearner(int dom = 1, int cod = 1) : domainSize(dom), coDomainSize(cod) {
    }

    /*
     * Inherited from IMachineLearner.
     */
    virtual void feedSample(const Vector& input, const Vector& output);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void train();

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
     * Inherited from IMachineLearner.
     */
    virtual std::string getInfo();

    /*
     * Inherited from IMachineLearner.
     */
    virtual std::string getConfigHelp();

    /*
     * Inherited from IConfig.
     */
    virtual bool configure(Searchable& config);

};

} // learningmachine
} // iCub

#endif
