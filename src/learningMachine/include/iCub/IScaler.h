/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Linear scaler (element-based).
 *
 */

#ifndef __ICUB_ISCALER__
#define __ICUB_ISCALER__

#include <cmath>
#include <string>
#include <yarp/os/IConfig.h>

using namespace yarp::os;

namespace iCub {
namespace contrib {
namespace learningmachine {

/**
 * The IScaler is a linear scaler based scaler.
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
    //virtual void writeBottle(Bottle& bot) = 0;

    /**
     * Unserializes a scaler from a bottle. 
     *
     * @param bot the bottle
     */
    //virtual void readBottle(Bottle& bot) = 0;

public:
    /**
     * Constructor.
     *
     * @param s the scale for the linear transformation
     * @param o the offset for the linear transformation
     */
    IScaler(double s = 1., double o = 0.) : scale(s), offset(o), updateEnabled(true) {
        this->setName("");
    }

    /**
     * Destructor.
     */
    ~IScaler() {}

    /**
     * Feeds a single sample into the scaler, so that it can use this sample
     * to update the offset and scale.
     *
     * @param value the sample value
     */
    virtual void update(double val) {}

    /**
     * Transforms a single sample value according to the state of the scaler. 
     * This state is usually made up out of a desired offset and scale.
     *
     * @param val the sample
     * @return the resulting, transformed sample
     */
    virtual double transform(double val) {
        // update scaler if enabled
        if(this->updateEnabled) {
            this->update(val);
        }
        // check for division by zero
        return (fabs(this->scale) < 1.e-20) ? (val - this->offset) : (val - this->offset) / this->scale;
    }

    /**
     * Untransforms a single sample value according to the state of the 
     * scaler. This operation is the inverse of the transform operation.
     *
     * @param val the sample
     * @return the resulting, transformed sample
     */
    virtual double unTransform(double val) {
        // check for division by zero
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
    virtual IScaler* create() = 0;

    /*
     * Inherited from IConfig.
     */
    virtual bool configure(Searchable& config);
};

} // learningmachine
} // contrib
} // iCub

#endif
