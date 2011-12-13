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

#ifndef LM_LINEARSCALER__
#define LM_LINEARSCALER__

#include "iCub/learningMachine/IScaler.h"

namespace iCub {
namespace learningmachine {

/**
 * \ingroup icub_libLM_transformers
 *
 * A class that implements linear scaling as a preprocessing step.
 *
 * \see iCub::learningmachine::IScaler
 *
 * \author Arjan Gijsberts
 *
 */

class LinearScaler : public IScaler {
protected:
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
     * @param s the scale for the linear transformation
     * @param o the offset for the linear transformation
     */
    LinearScaler(double s = 1, double o = 0);

    /*
     * Inherited from IScaler
     */
    virtual std::string getInfo();

    /*
     * Inherited from IConfig.
     */
    virtual bool configure(yarp::os::Searchable& config);

    /*
     * Inherited from IScaler.
     */
    LinearScaler* clone() {
        return new LinearScaler(*this);
    }

    /**
     * Accessor for the scaling factor.
     */
    virtual double getScale() { return this->scale; }

    /**
     * Mutator for the scaling factor.
     *
     * @param s the new scaling factor
     */
    virtual void setScale(double s) { this->scale = s; }

    /**
     * Accessor for the offset.
     */
    virtual double getOffset() { return this->offset; }

    /**
     * Mutator for the offset.
     *
     * @param o the new offset
     */
    virtual void setOffset(double o) { this->offset = o; }

};

} // learningmachine
} // iCub

#endif
