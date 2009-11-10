/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Least Squares Support Vector Machine (LSSVM) based learner 
 *
 */

/**
 *
 * An implementation of the LSSVM learning machine that uses the efficient Atlas
 * library for linear algebra.
 * interface.
 *
 * \see iCub::contrib::IMachineLearner
 * \see iCub::contrib::IFixedSizeLearner
 *
 * \author Arjan Gijsberts
 *
 */

#ifndef LM_LSSVMATLASLEARNER__
#define LM_LSSVMATLASLEARNER__

#include <iostream>
#include <string>
#include <vector>
#include <sstream>

#include <yarp/os/Bottle.h>

#include "iCub/IFixedSizeLearner.h"
#include "iCub/LSSVM.h"

using namespace yarp;
using namespace iCub::contrib::liblssvm;

namespace iCub {
namespace learningmachine {

/**
 *
 * This is a wrapper around the LibLSSVM learner for use with the 
 * IMachineLearner interface.
 *
 * \see iCub::contrib::IMachineLearner
 * \see iCub::contrib::IFixedSizeLearner
 *
 * \author Arjan Gijsberts
 *
 */
class LSSVMAtlasLearner : public IFixedSizeLearner {
private:
    /**
     * Indicator whether the kernel functions have been registered in a 
     * a factory. This should be done only once during runtime.
     */
    static bool kernelsRegistered;

    /**
     * The vector of LSSVM machines; one for each output element.
     */
    std::vector<LSSVM*> machines;

    /**
     * The default configuration for the kernel.
     */
    std::string defaultKernel;

    /**
     * The stored (default) type of LSSVM machines that are being used.
     */
    int defaultType;

    /**
     * Storage for the input vectors.
     */
    std::vector<Vector> inputs;

    /**
     * Storage for the output vectors.
     */
    std::vector<Vector> outputs;

    /**
     * Enumeration of the different types of LSSVM machines.
     */
    enum machine {FULL, REFERENCE, PARTIAL};

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
     * Sets the kernel of all machines to a specified kernel type.
     *
     * @param kernel the kernel.
     */
    void setKernelAll(std::string kernel);

    /**
     * Sets the kernel of the machine at a given index to a specified kernel type.
     *
     * @param index the index of the element.
     * @param kernel the kernel.
     */
    void setKernelAt(int index, std::string kernel);

    /**
     * Sets the regularization parameter C of all machines to a specified value.
     *
     * @param c the desired value.
     */
    void setCAll(double c);

    /**
     * Sets the regularization parameter C of the machine at a given index to a specified value.
     *
     * @param index the index of the element.
     * @param c the desired value.
     */
    void setCAt(int index, double c);

    /**
     * Creates a new machine according to the currently stored type specifier.
     *
     * @return a machine of the desired type.
     */
    LSSVM* createMachine(int type);

public:
    /**
     * Constructor.
     *
     * @param size the initial codomain size
     * @param type specifier for the type of LSSVM (Cholesky, Reference or Partial)
     * @param kernel configuration specifier for the initial type of kernel
     */
    LSSVMAtlasLearner(int size = 1, int type = LSSVMAtlasLearner::FULL, std::string kernel = "1:1");

    /**
     * Destructor.
     */
    virtual ~LSSVMAtlasLearner();

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
    LSSVMAtlasLearner* clone();

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
};

} // learningmachine
} // iCub
#endif
