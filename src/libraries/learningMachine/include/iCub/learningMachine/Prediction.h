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

#ifndef LM_PREDICTION__
#define LM_PREDICTION__

#include <string>
#include <cassert>

#include <yarp/os/Portable.h>
#include <yarp/sig/Vector.h>

namespace iCub {
namespace learningmachine {

/**
 * \ingroup icub_libLM_learning_machines
 *
 * A class that represents a prediction result. The serialization of this class is
 * compatible with PortablePair<Vector,Vector>, such that standard Yarp classes can
 * be used to read a Prediction from the network.
 *
 * \author Arjan Gijsberts
 *
 */

class Prediction : public yarp::os::Portable {
protected:
    /**
     * Expected value of the prediction.
     */
    yarp::sig::Vector prediction;

    /**
     * Optional variance of the prediction, measured as a unit standard deviation.
     */
    yarp::sig::Vector variance;

    /**
     * Indicator whether prediction variance is available.
     */
    bool varianceSet;

public:
    /**
     * Empty constructor.
     */
    Prediction() {
        this->setPrediction(yarp::sig::Vector(0));
        this->setVariance(yarp::sig::Vector(0));
    }

    /**
     * Constructor for expected value prediction without predictive variance.
     *
     * @param prediction  the predicted expected value
     * @param variance  the predicted expected variance in terms of a unit
     *                  standard deviation
     */
    Prediction(const yarp::sig::Vector& prediction) {
        this->setPrediction(prediction);
        this->setVariance(yarp::sig::Vector(0));
    }

    /**
     * Constructor for expected value prediction with predictive variance.
     *
     * @param prediction  the predicted expected value
     */
    Prediction(const yarp::sig::Vector& prediction, const yarp::sig::Vector& variance) {
        this->setPrediction(prediction);
        this->setVariance(variance);
    }

    /**
     * Returns the size of the prediction.
     *
     * @return the size
     */
    unsigned int size() {
        return this->prediction.size();
    }

    /**
     * Accessor for the expected value of the prediction.
     *
     * @return the predicted expected value
     */
    yarp::sig::Vector getPrediction() {
        return this->prediction;
    }

    /**
     * Mutator for the expected value of the prediction.
     *
     * @param prediction  the predicted expected value
     */
    void setPrediction(const yarp::sig::Vector& prediction) {
        this->prediction = prediction;
    }

    /**
     * Accessor for the variance of the prediction. More precisely, this
     * returns a predicted unit standard deviation.
     *
     * @return the predicted variance
     */
    yarp::sig::Vector getVariance() {
        return this->variance;
    }

    /**
     * Mutator for the variance of the prediction.
     *
     * @param variance  the predicted expected variance in terms of a unit
     *        standard deviation
     */
    void setVariance(const yarp::sig::Vector& variance) {
        assert(prediction.size() == variance.size() || variance.size() == 0);
        this->variance = variance;
        this->varianceSet = (variance.size() > 0);
    }

    /**
     * Indicator whether the prediction contains a predicted variance.
     *
     * @return  true if variance is available
     */
    bool hasVariance() {
        return this->varianceSet;
    }

    /**
     * Returns a string represenation of the prediction.
     *
     * @return  a string represenation of the prediction
     */
    std::string toString() {
        std::string out = this->prediction.toString().c_str();
        if(this->hasVariance()) {
            out += std::string(" +/- ") + this->variance.toString().c_str();
        }
        return out;
    }

    /*
     * Inherited from Portable.
     */
    bool write(yarp::os::ConnectionWriter& connection) {
        // follows PortablePair implementation
        connection.appendInt(BOTTLE_TAG_LIST);
        connection.appendInt(2);

        bool ok = this->prediction.write(connection);
        if (ok) {
            ok = this->variance.write(connection);
        }

        if (ok) {
            connection.convertTextMode();
        }

        return ok;
    }

    /*
     * Inherited from Portable.
     */
    bool read(yarp::os::ConnectionReader& connection) {
        // follows PortablePair implementation
        connection.convertTextMode();

        int header = connection.expectInt();
        if(header != BOTTLE_TAG_LIST) {
            return false;
        }

        int len = connection.expectInt();
        if(len != 2) {
            return false;
        }

        bool ok = this->prediction.read(connection);
        if (ok) {
            ok = this->variance.read(connection);
        }
        this->varianceSet = (this->variance.size() > 0);
        return ok;
    }

    /*
     * Inherited from Portable.
     */
    virtual void onCompletion() {
        this->prediction.onCompletion();
        this->variance.onCompletion();
    }

};


} // learningmachine
} // iCub

#endif
