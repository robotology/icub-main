/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Fixed range scaler to preprocess data samples.
 *
 */

#ifndef __ICUB_FIXEDRANGESCALER__
#define __ICUB_FIXEDRANGESCALER__

#include "iCub/IScaler.h"

using namespace yarp::os;

namespace iCub {
namespace contrib {
namespace learningmachine {


/**
 *
 * A class that implements preprocessing based on a fixed range of outputs to a 
 * fixed range of outputs. 
 *
 * \see iCub::contrib::learningmachine::IScaler
 *
 * \author Arjan Gijsberts
 *
 */
class FixedRangeScaler : public IScaler {
protected:
    /**
     * The desired lower bound for the output range.
     */
    double lowerBoundOut;

    /**
     * The desired upper bound for the output range.
     */
    double upperBoundOut;

    /**
     * The expected lower bound in the sample values.
     */
    double lowerBoundIn;

    /**
     * The expected upper bound in the sample values.
     */
    double upperBoundIn;
    
    /**
     * Updates the scale and offset according to the specified expected and 
     * desired ranges.
     */
    void updateScales();

    /*
     * Inherited from IScaler
     */
    virtual void writeBottle(Bottle& bot);

    /*
     * Inherited from IScaler
     */
    virtual void readBottle(Bottle& bot);

public:
    /**
     * Constructor.
     *
     * @param li the initial lower input bound
     * @param ui the initial upper input bound
     * @param lo the initial lower output bound
     * @param uo the initial upper output bound
     */
    FixedRangeScaler(double li = -1., double ui = 1., double lo = -1., double uo = 1.);

    /*
     * Inherited from IScaler
     */
    virtual std::string getInfo();

    /*
     * Inherited from IConfig.
     */
    virtual bool configure(Searchable& config);

    /*
     * Inherited from IScaler.
     */
    IScaler* create() {
        return new FixedRangeScaler();
    }

    /**
     * Accessor for the desired lower bound.
     */
    virtual double getLowerBoundOut() { return this->lowerBoundOut; }

    /**
     * Mutator for the desired lower bound.
     *
     * @param l the new lower bound
     */
    virtual void setLowerBoundOut(double lo) { 
        this->lowerBoundOut = lo;
        this->updateScales(); 
    }

    /**
     * Accessor for the desired upper bound.
     */
    virtual double getUpperBoundOut() { return this->upperBoundOut; }

    /**
     * Mutator for the desired upper bound.
     *
     * @param u the new upper bound
     */
    virtual void setUpperBoundOut(double uo) {
        this->upperBoundOut = uo;
        this->updateScales(); 
    }

    /**
     * Accessor for the expected lower bound.
     */
    virtual double getLowerBoundIn() { return this->lowerBoundIn; }

    /**
     * Mutator for the expected lower bound.
     *
     * @param l the new lower bound
     */
    virtual void setLowerBoundIn(double li) {
        this->lowerBoundIn = li;
        this->updateScales(); 
    }

    /**
     * Accessor for the expected upper bound.
     */
    virtual double getUpperBoundIn() { return this->upperBoundIn; }

    /**
     * Mutator for the expected upper bound.
     *
     * @param u the new upper bound
     */
    virtual void setUpperBoundIn(double ui) {
        this->upperBoundIn = ui;
        this->updateScales(); 
    }

};

} // learningmachine
} // contrib
} // iCub

#endif
