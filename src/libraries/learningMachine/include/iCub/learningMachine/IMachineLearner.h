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

#ifndef LM_IMACHINELEARNER__
#define LM_IMACHINELEARNER__

#include <string>
#include <sstream>

#include <yarp/sig/Vector.h>
#include <yarp/os/Portable.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>

#include "iCub/learningMachine/Prediction.h"

namespace iCub {
namespace learningmachine {

/**
 * \defgroup icub_libLearningMachine learningMachine
 *
 * \ingroup icub_libraries
 *
 * A library containing learning machines and data transformers (preprocessors).
 *
 * \section icub_lm_dep Dependencies
 * - YARP
 * - Gnu Scientific Library
 *
 * \author Arjan Gijsberts
 */

/**
 * \defgroup icub_libLM_learning_machines Learning Machines
 *
 * \ingroup icub_libLearningMachine
 *
 * A collection of machine learning algorithms.
 *
 * \author Arjan Gijsberts
 *
 */

/**
 * \ingroup icub_libLM_learning_machines
 *
 * A generalized interface for a learning machine for offline and
 * online learning machines (e.g. SVM, LSSVM, ANN). This interface
 * extends the Learner interface, which imposes certain limitations.
 * Learning machines that meet this interface can, if desired, be made
 * into two executable modules by iCub::contrib::LearnModule and
 * iCub::contrib::PredictModule.
 *
 * The learning machine can be used for regression and classification
 * from R^* to R^*.
 *
 * Be aware that objects of this class type cannot be written directly
 * on a port, although this interface implements Portable. The reason is
 * that this is an abstract base class, which conflicts with the template
 * port readers and writers. See MachinePortable for how to send
 * Learning Machines over ports.
 *
 * \see iCub::contrib::Learner
 * \see iCub::contrib::IFixedSizeLearner
 * \see iCub::contrib::MachinePortable
 * \see iCub::contrib::MachineFactory
 *
 * \author Arjan Gijsberts
 *
 */

class IMachineLearner : public yarp::os::Portable {
protected:
    /**
     * The name of this type of machine learner.
     */
    std::string name;

    /**
     * Writes a serialization of the machine into a bottle. This method is
     * internally referenced by the write method. Typically, subclasses should
     * override this method instead of overriding the write method directly.
     *
     * @param bot the bottle containing the machine serialization
     */
    virtual void writeBottle(yarp::os::Bottle& bot) const = 0;

    /**
     * Unserializes a machine from a bottle. This method is internally
     * referenced by the read method. Typically, subclasses should override this
     * method instead of overriding the read method directly.
     *
     * @param bot the bottle
     */
    virtual void readBottle(yarp::os::Bottle& bot) = 0;

public:
    /**
     * Constructor.
     */
    IMachineLearner() : name("") { }

    /**
     * Destructor (empty).
     */
    virtual ~IMachineLearner() { }


    /**
     * Initialize the object.
     */
    virtual bool open(yarp::os::Searchable& config) {return true;}

    /**
     * Shut the object down.
     */
    virtual bool close() {return true;}

    /**
     * Change parameters.
     */
    virtual bool configure(yarp::os::Searchable& config) {return false;}

    /**
     * Provide the learning machine with an example of the desired mapping.
     *
     * @param input a sample input
     * @param output the corresponding output
     */
    virtual void feedSample(const yarp::sig::Vector& input, const yarp::sig::Vector& output) = 0;

    /**
     * Train the learning machine on the examples that have been supplied so
     * far. This method is primarily intended to be used for offline/batch
     * learning machines. It explicitly initiates the training routine on those
     * machines for the samples that have been collected so far.
     */
    virtual void train() { }

    /**
     * Ask the learning machine to predict the output for a given input.
     *
     * @param input the input
     * @return the expected output
     */
    virtual Prediction predict(const yarp::sig::Vector& input) = 0;

    /**
     * Asks the learning machine to return a clone of its type.
     *
     * @return a clone of the current learner
     */
    virtual IMachineLearner* clone() = 0;

    /**
     * Forget everything and start over.
     */
    virtual void reset() = 0;

    /*
     * Inherited from Portable.
     */
    bool write(yarp::os::ConnectionWriter& connection) const {
        yarp::os::Bottle model;
        this->writeBottle(model);
        return model.write(connection);
    }

    /*
     * Inherited from Portable.
     */
    bool read(yarp::os::ConnectionReader& connection) {
        yarp::os::Bottle model;
        model.read(connection);
        this->readBottle(model);
        return true;
    }

    /**
     * Asks the learning machine to return a string containing information on
     * its operation so far.
     *
     * @return the information on the machine
     */
    virtual std::string getInfo() {
        return std::string("Type: ") + this->getName() + std::string("\n");
    }

    /**
     * Asks the learning machine to return a string containing the list of
     * configuration options that it supports.
     *
     * @return an informative description of the configuration options
     */
    virtual std::string getConfigHelp() {
        return std::string("Machine configuration options for '") +
               this->getName() + "'\n";
    }

    /**
     * Asks the learning machine to return a string serialization.
     *
     * @return a string serialization of the machine
     */
    virtual std::string toString() {
        yarp::os::Bottle model;
        this->writeBottle(model);
        return model.toString().c_str();
    }

    /**
     * Asks the learning machine to initialize from a string serialization.
     *
     * @return true on succes
     */
    virtual bool fromString(const std::string& str) {
        yarp::os::Bottle model(str.c_str());
        this->readBottle(model);
        return true;
    }

    /**
     * Retrieve the name of this machine learning technique.
     *
     * @return the name of this machine learner
     */
    std::string getName() const {
        return this->name;
    }

    /**
     * Set the name of this machine learning technique.
     *
     * @param name the new name
     */
    void setName(const std::string& name) {
        this->name = name;
    }
};

} // learningmachine
} // iCub

#endif
