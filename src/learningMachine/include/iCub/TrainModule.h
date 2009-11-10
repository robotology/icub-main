/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Training module class for wrapping an executable around IMachineLearner classes.
 *
 */

#ifndef LM_TRAINMODULE__
#define LM_TRAINMODULE__

#include "iCub/PredictModule.h"

using namespace yarp::os;
using namespace yarp::sig;

namespace iCub {
namespace learningmachine {


/**
 * Port processor helper class for incoming training samples.
 *
 * \see iCub::learningmachine::TrainModule
 * \see iCub::learningmachine::IMachineProcessor
 *
 * \author Arjan Gijsberts
 *
 */
class TrainProcessor : public IMachineProcessor, public TypedReaderCallback< PortablePair<Vector,Vector> > {
private:
    /**
     * Boolean switch to disable and enable the sample stream to the machine.
     */
    bool enabled;

public:
    /**
     * Constructor.
     *
     * @param mp a reference to a machine portable.
     */
    TrainProcessor(MachinePortable& mp) : IMachineProcessor(mp), enabled(true) { }

    /**
     * Enables or disables processing of training samples.
     *
     * @param val the desired state
     */
    virtual void setEnabled(bool val) {
        this->enabled = val;
    }

    /*
     * Inherited from TypedReaderCallback.
     */
    virtual void onRead(PortablePair<Vector,Vector>& sample);

};


/**
 * A module for training.
 * The module can contain any iCub::contrib::IMachineLearner
 * This module is inspired by the original Learner module by Francesco Orabona
 * and Paul Fitzpatrick.
 *
 * \see iCub::learningmachine::IMachineLearner
 * \see iCub::learningmachine::IMachineLearnerModule
 * \see iCub::learningmachine::PredictModule
 * \see iCub::learningmachine::Learner
 *
 * \author Arjan Gijsberts, Francesco Orabona, Paul Fitzpatrick
 *
 */

class TrainModule : public PredictModule {
private:
    /**
     * Buffered port for the incoming training samples.
     */
    BufferedPort<PortablePair<Vector,Vector> > train_in;

    /**
     * Port for the outgoing models to the predict module.
     */
    Port model_out;

    /**
     * The processor handling incoming training samples.
     */
    TrainProcessor trainProcessor;

    /**
     * Copy constructor (private and unimplemented on purpose).
     */
    TrainModule(const TrainModule& other);

    /**
     * Assignment operator (private and unimplemented on purpose).
     */
    TrainModule& operator=(const TrainModule& other);

    /*
     * Inherited from IMachineLearnerModule.
     */
    void registerAllPorts();

    /*
     * Inherited from IMachineLearnerModule.
     */
    void unregisterAllPorts();

    /*
     * Inherited from IMachineLearnerModule.
     */
    void exitWithHelp(std::string error = "");

public:
    /**
     * Constructor.
     *
     * @param pp the default prefix used for the ports.
     * @param support an instance of the Support class.
     */
    TrainModule(std::string pp = "/lm/train")
      : PredictModule(pp), trainProcessor(machinePortable) { }

    /**
     * Destructor.
     */
    /*virtual ~TrainModule() {
        delete(this->machinePortable);
    }*/

    /*
     * Inherited from IMachineLearnerModule.
     */
    virtual bool open(Searchable& opt);

    /*
     * Inherited from IMachineLearnerModule.
     */
    //bool updateModule();

    /*
     * Inherited from IMachineLearnerModule.
     */
    virtual bool interruptModule();

    /*
     * Inherited from IMachineLearnerModule.
     */
    virtual bool respond(const Bottle& cmd, Bottle& reply);
};

} // learningmachine
} // iCub

#endif
