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

#ifndef LM_FIXEDRANGESCALER__
#define LM_FIXEDRANGESCALER__

#include "iCub/learningMachine/IScaler.h"


namespace iCub {
namespace learningmachine {

/**
 * \ingroup icub_libLM_transformers
 *
 * A class that implements preprocessing based on a fixed range of outputs to a
 * fixed range of outputs.
 *
 * \see iCub::learningmachine::IScaler
 *
 * \author Arjan Gijsberts
 *
 */
class FixedRangeScaler : public IScaler {
protected:
    /**
     * The expected lower bound in the sample values.
     */
    double lowerBoundIn;

    /**
     * The expected upper bound in the sample values.
     */
    double upperBoundIn;

    /**
     * The desired lower bound for the output range.
     */
    double lowerBoundOut;

    /**
     * The desired upper bound for the output range.
     */
    double upperBoundOut;

    /**
     * Updates the scale and offset according to the specified expected and
     * desired ranges.
     */
    void updateScales();

    /*
     * Inherited from IScaler
     */
    virtual void writeBottle(yarp::os::Bottle& bot);

    /*
     * Inherited from IScaler
     */
    virtual void readBottle(yarp::os::Bottle& bot);

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
     * Inherited from IScaler.
     */
    virtual bool configure(yarp::os::Searchable& config);

    /*
     * Inherited from IScaler.
     */
    FixedRangeScaler* clone() {
        return new FixedRangeScaler(*this);
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
} // iCub

#endif
