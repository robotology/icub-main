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

#ifndef LM_TRAINMODULE__
#define LM_TRAINMODULE__

#include <yarp/os/PortablePair.h>

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
 * \ingroup icub_libLM_modules
 *
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
    void printOptions(std::string error = "");

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
     * Destructor (empty).
     */
    virtual ~TrainModule() { };

    /*
     * Inherited from IMachineLearnerModule.
     */
    virtual bool configure(ResourceFinder& opt);

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
