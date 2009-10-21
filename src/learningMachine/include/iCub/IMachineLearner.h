/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Generalized interfaces for learning machines (both offline and online)
 *
 */

#ifndef __ICUB_IMACHINELEARNER__
#define __ICUB_IMACHINELEARNER__

#include <yarp/sig/Vector.h>
#include <yarp/os/IConfig.h>
#include <yarp/os/Portable.h>
#include <yarp/os/Value.h>

#include <string>
#include <stdexcept>

using namespace yarp::os;
using namespace yarp::sig;

namespace iCub {
namespace contrib {
namespace learningmachine {


/**
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
class IMachineLearner : public IConfig, public Portable {
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
    virtual void writeBottle(Bottle& bot) = 0;

    /**
     * Unserializes a machine from a bottle. This method is internally 
     * referenced by the read method. Typically, subclasses should override this 
     * method instead of overriding the read method directly.
     *
     * @param bot the bottle
     */
    virtual void readBottle(Bottle& bot) = 0;

public:
    /**
     * Constructor.
     */
    IMachineLearner() {
        this->setName("");
    }

    /**
     * Destructor.
     */
    ~IMachineLearner() { }

    /**
     * Provide the learning machine with an example of the desired mapping.
     *
     * @param input a sample input
     * @param output the corresponding output
     */
    virtual void feedSample(const Vector& input, const Vector& output) = 0;

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
    virtual Vector predict(const Vector& input) = 0;

    /**
     * Asks the learning machine to return a new object of its type.
     *
     * @return a fresh instance of the current class
     */
    virtual IMachineLearner* create() = 0;

    /**
     * Forget everything and start over.
     */
    virtual void reset() = 0;

    /*
     * Inherited from Portable.
     */
    bool write(ConnectionWriter& connection) {
        Bottle model;
        this->writeBottle(model);
        model.write(connection);
        return true;
    }

    /*
     * Inherited from Portable.
     */
    bool read(ConnectionReader& connection) {
        Bottle model;
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
        Bottle model;
        this->writeBottle(model);
        return model.toString().c_str();
    }

    /**
     * Asks the learning machine to initialize from a string serialization.
     *
     * @return true on succes
     */
    virtual bool fromString(const std::string& str) {
        Bottle model(str.c_str()); 
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
} // contrib
} // iCub

#endif
