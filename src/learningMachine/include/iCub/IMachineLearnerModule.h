/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Abstract base class for all MachineLearner modules.
 *
 */


#ifndef __ICUB_IMACHINELEARNERMODULE__
#define __ICUB_IMACHINELEARNERMODULE__

#include <iostream>

#include <yarp/os/Module.h>
#include <yarp/os/Searchable.h>

#include "iCub/DispatcherManager.h"


using namespace yarp::os;

namespace iCub {
namespace contrib {
namespace learningmachine {

/**
 * An abstract base module for the machine learning YARP interface. This 
 * abstract class contains base functionality that is shared by 
 * PredictModule, TrainModule and TransformModule.
 *
 * \see iCub::contrib::learningmachine::IMachineLearner
 * \see iCub::contrib::learningmachine::TrainModule
 * \see iCub::contrib::learningmachine::PredictModule
 * \see iCub::contrib::learningmachine::TransformerModule
 *
 * \author Arjan Gijsberts
 */
class IMachineLearnerModule : public Module {
protected:
    /**
     * An input port for commands.
     */
    BufferedPort<Bottle> cmd_in;

    /**
     * A prefix path for the ports that will be registered.
     */
    std::string portPrefix;
    
    /**
     * An instance of the DispatchManager to configure event listeners.
     */
    DispatcherManager dmanager;

    /**
     * Register a port with a given name.
     *
     * @param port the port object
     * @param name the name for the port
     *
     */
    void registerPort(Contactable& port, std::string name);

    /**
     * Registers all ports used by this module.
     */
    virtual void registerAllPorts() = 0;

    /**
     * Unregisters all ports used by this module.
     */
    virtual void unregisterAllPorts() = 0;

    /**
     * Exits the module while printing a help message.
     */
    virtual void exitWithHelp(std::string error = "") = 0;

public:
    /**
     * Constructor.
     *
     * @param pp the default prefix used for the ports.
     */
    IMachineLearnerModule(std::string pp) : portPrefix(pp) {
    }

    /**
     * Destructor.
     */
    ~IMachineLearnerModule() {
    }

    /**
     * Close the module.
     *
     * @return true if the module was closed successfully
     */
    virtual bool close();

};

} // learningmachine
} // contrib
} // iCub

#endif

