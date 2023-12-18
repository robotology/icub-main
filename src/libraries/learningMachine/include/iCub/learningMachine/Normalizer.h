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

#ifndef LM_NORMALIZER__
#define LM_NORMALIZER__

#include "iCub/learningMachine/IScaler.h"

namespace iCub {
namespace learningmachine {

/**
 * \ingroup icub_libLM_transformers
 *
 * A class that implements normalization as a preprocessing step. This
 * normalizer limits all values between a predesignated range. By default this
 * range is defined as [-1, 1].
 *
 * \see iCub::learningmachine::IScaler
 *
 * \author Arjan Gijsberts
 *
 */

class Normalizer : public IScaler {
protected:
    /**
     * The actual lowest value found in the sample values.
     */
    double lowest;

    /**
     * The actual highest value found in the sample values.
     */
    double highest;

    /**
     * The desired lower bound for the normalization range.
     */
    double lowerBound;

    /**
     * The desired upper bound for the normalization range.
     */
    double upperBound;

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
     * @param l the initial lower bound
     * @param u the initial upper bound
     */
    Normalizer(double l = -1, double u = 1);

    /*
     * Inherited from IScaler.
     */
    virtual void update(double val);

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
    Normalizer* clone() {
        return new Normalizer(*this);
    }

    /**
     * Accessor for the desired lower bound.
     */
    virtual double getLowerBound() { return this->lowerBound; }

    /**
     * Mutator for the desired lower bound.
     *
     * @param l the new lower bound
     */
    virtual void setLowerBound(double l) { this->lowerBound = l; }

    /**
     * Accessor for the desired upper bound.
     */
    virtual double getUpperBound() { return this->upperBound; }

    /**
     * Mutator for the desired upper bound.
     *
     * @param u the new upper bound
     */
    virtual void setUpperBound(double u) { this->upperBound = u; }

};

} // learningmachine
} // iCub

#endif
