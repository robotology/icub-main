/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Prediction module class for wrapping an executable around IMachineLearner classes.
 *
 */

#ifndef __ICUB_PREDICTMODULE__
#define __ICUB_PREDICTMODULE__

#include "iCub/IMachineLearnerModule.h"
#include "iCub/MachinePortable.h"

using namespace yarp::os;
using namespace yarp::sig;

namespace iCub {
namespace contrib {
namespace learningmachine {


/**
 * Generic abstract class for machine based processors.
 *
 * \see iCub::contrib::learningmachine::PredictProcessor
 * \see iCub::contrib::learningmachine::TrainProcessor
 *
 * \author Arjan Gijsberts
 *
 */
class IMachineProcessor {
protected:
    /**
     * A pointer to a concrete wrapper around a learning machine.
     */
    MachinePortable* machinePortable;

public:
    /**
     * Constructor.
     *
     * @param mp a pointer to a machine portable.
     */
    IMachineProcessor(MachinePortable* mp = (MachinePortable*) 0) : machinePortable(mp) { }
    
    /**
     * Mutator for the machine portable.
     *
     * @param mp a pointer to a machine portable.
     */
    virtual void setMachinePortable(MachinePortable* mp) {
        this->machinePortable = mp;
    }
    
    /**
     * Retrieve the machine portable machine wrapper.
     *
     * @return a pointer to the machine portable
     */
    virtual MachinePortable* getMachinePortable() {
        return this->machinePortable;
    }

    /**
     * Convenience function to quickly retrieve the machine that is wrapped in 
     * the portable machine wrapper.
     *
     * @return a pointer to the actual machine
     */
    virtual IMachineLearner* getMachine() {
        return this->getMachinePortable()->getWrapped();
    }
};



/**
 * Reply processor helper class for predictions. 
 *
 * \see iCub::contrib::learningmachine::PredictModule
 * \see iCub::contrib::learningmachine::IMachineProcessor
 *
 * \author Arjan Gijsberts
 *
 */
class PredictProcessor : public IMachineProcessor, public PortReader {
public:    
    /*
     * Inherited from PortReader.
     */
    virtual bool read(ConnectionReader& connection);
};


/**
 * A module for predictions.
 * The module can contain any iCub::contrib::learningmachine::IMachineLearner.
 * This module can be used in a combined system with a TrainModule.
 *
 * \see iCub::contrib::learningmachine::IMachineLearner
 * \see iCub::contrib::learningmachine::IMachineLearnerModule
 * \see iCub::contrib::learningmachine::TrainModule
 *
 * \author Arjan Gijsberts, Francesco Orabona, Paul Fitzpatrick
 *
 */
class PredictModule : public IMachineLearnerModule {
protected:

    /**
     * Buffered port for the incoming samples and corresponding replies.
     */
    BufferedPort<Vector> predict_inout;

    /**
     * A pointer to a concrete wrapper around a learning machine.
     */
    MachinePortable* machinePortable;
    
    /**
     * The processor handling prediction requests.
     */
    PredictProcessor predictProcessor;

    /**
     * Incoming port for the models from the train module.
     */
    Port model_in;

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
     */
    PredictModule(std::string pp = "/lm/predict") : IMachineLearnerModule(pp) {
        this->machinePortable = new MachinePortable();
    }

    /**
     * Destructor.
     */
    ~PredictModule() {
        delete(this->machinePortable);
    }

    /*
     * Inherited from IMachineLearnerModule.
     */
    virtual bool open(Searchable& opt);

    /*
     * Inherited from IMachineLearnerModule.
     */
    virtual bool interruptModule();

    /*
     * Inherited from IMachineLearnerModule.
     */
    virtual bool respond(const Bottle& cmd, Bottle& reply);

    /*
     * Inherited from IMachineLearnerModule.
     */
    virtual bool close() {
        IMachineLearnerModule::close();
        if(this->getMachinePortable()->hasWrapped()) {
            return this->getMachine()->close();
        } else {
            return true;
        }
    }

    /**
     * Retrieve the machine that is wrapped in the portable machine wrapper.
     *
     * @return a pointer to the actual machine
     */
    virtual IMachineLearner* getMachine() {
        return this->getMachinePortable()->getWrapped();
    }

    /**
     * Retrieve the machine portable.
     *
     * @return a pointer to the machine portable
     */
    virtual MachinePortable* getMachinePortable() {
        return this->machinePortable;
    }

};

} // learningmachine
} // contrib
} // iCub

#endif
