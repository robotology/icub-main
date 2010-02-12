/*
 * Copyright (C) 2007-2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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

#ifndef LM_ISCALER__
#define LM_ISCALER__

#include <cmath>
#include <string>
#include <yarp/os/IConfig.h>

using namespace yarp::os;

namespace iCub {
namespace learningmachine {

/**
 * \ingroup icub_libLM_transformers
 *
 * The IScaler is a linear scaler based scaler. It is used in particularly as
 * part of the ScaleTransformer.
 *
 * \see iCub::learningmachine:ScaleTransformer
 *
 * \author Arjan Gijsberts
 *
 */

class IScaler : public IConfig {
protected:
    /**
     * The offset for the linear transformation.
     */
    double offset;

    /**
     * The scale for the linear transformation.
     */
    double scale;

    /**
     * The name of this type of scaler.
     */
    std::string name;

    /**
     * Boolean indicating whether the scaler has to update each sample.
     */
    bool updateEnabled;

    /**
     * Writes a serialization of the scaler into a bottle.
     *
     * @param bot the bottle
     */
    virtual void writeBottle(Bottle& bot);

    /**
     * Unserializes a scaler from a bottle.
     *
     * @param bot the bottle
     */
    virtual void readBottle(Bottle& bot);

public:
    /**
     * Constructor.
     *
     * @param s the scale for the linear transformation
     * @param o the offset for the linear transformation
     */
    IScaler(double s = 1., double o = 0.)
      : offset(o), scale(s), name(""), updateEnabled(true) { }

    /**
     * Destructor (empty).
     */
    virtual ~IScaler() { }

    /**
     * Feeds a single sample into the scaler, so that it can use this sample
     * to update the offset and scale.
     *
     * @param value the sample value
     */
    virtual void update(double val) { }

    /**
     * Transforms a single sample value according to the state of the scaler.
     * This state is usually made up out of a desired offset and scale.
     *
     * @param val the sample
     * @return the resulting, transformed sample
     */
    virtual double transform(double val);

    /**
     * Untransforms a single sample value according to the state of the
     * scaler. This operation is the inverse of the transform operation.
     *
     * @param val the sample
     * @return the resulting, transformed sample
     */
    virtual double unTransform(double val) {
        return (val * this->scale) + this->offset;
    }

    /**
     * Asks the learning machine to return a string containing statistics on
     * its operation so far.
     *
     * @return the statistics of the machine
     */
    virtual std::string getInfo();

    /**
     * Retrieve the name of this scaler.
     *
     * @return the name of this scaler
     */
    std::string getName() const { return this->name; }

    /**
     * Set the name of this machine learning technique.
     *
     * @param name the new name
     */
    void setName(std::string name) { this->name = name; }

    /**
     * Mutator for the update state.
     *
     * @param u the desired boolean state
     */
    virtual void setUpdateEnabled(bool u) { this->updateEnabled = u; }

    /**
     * Accessor for the update state.
     *
     * @return the current update state
     */
    virtual bool getUpdateEnabled() { return this->updateEnabled; }

    /**
     * Asks the scaler to return a new object of its type.
     *
     * @return a fresh instance of the specified type
     */
    virtual IScaler* clone() = 0;

    /**
     * Asks the scaler to return a string serialization.
     *
     * @return a string serialization of the scaler
     */
    virtual std::string toString();

    /**
     * Asks the scaler to initialize from a string serialization.
     *
     * @return true on succes
     */
    virtual bool fromString(const std::string& str);

    /*
     * Inherited from IConfig.
     */
    virtual bool configure(Searchable& config);
};

/**
 * \ingroup icub_libLM_transformers
 *
 * The NullScaler is a scaler that does nothing, the output of the transform
 * function is equal to its input. The use is so that the ScaleTransformer
 * can 'disable' transformation on certain inputs.
 *
 * \see iCub::learningmachine:IScaler
 * \see iCub::learningmachine:ScaleTransformer
 *
 * \author Arjan Gijsberts
 *
 */

class NullScaler : public IScaler {
private:
public:
    /**
     * Constructor.
     */
    NullScaler() {
        this->setName("null");
    }

    /*
     * Inherited from IScaler.
     */
    virtual double transform(double val) {
        return val;
    }

    /*
     * Inherited from IScaler.
     */
    virtual double unTransform(double val) {
        return val;
    }

    /*
     * Inherited from IScaler.
     */
    virtual NullScaler* clone() {
        return new NullScaler(*this);
    }
};

} // learningmachine
} // iCub

#endif
