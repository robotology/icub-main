/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Machine support class for shared objects, such as factories.
 *
 */

#ifndef __ICUB_MACHINESUPPORT__
#define __ICUB_MACHINESUPPORT__

#include <string>

#include "iCub/Support.h"
#include "iCub/IMachineLearner.h"

// machines
#include "iCub/DummyLearner.h"
#include "iCub/RLSLearner.h"
#include "iCub/LSSVMLearner.h"
#include "iCub/DatasetRecorder.h"
#ifdef BUILD_LSSVMATLAS // build only if we explicitly want to
#include "iCub/LSSVMAtlasLearner.h"
#endif

namespace iCub {
namespace contrib {
namespace learningmachine {


/**
 * The MachineSupport class provides a unified access point to the 
 * MachineLearner FactoryT instances 
 * and other global type of objects. This avoids the need for a singleton 
 * pattern. 
 *
 * \author Arjan Gijsberts
 *
 */

typedef FactoryT<std::string, IMachineLearner> MachineFactory;

class MachineSupport : private Support {
protected:
    /**
     * The IMachineLearner factory.
     */
    MachineFactory machineFactory;

public:
    /**
     * Constructor.
     *
     * @param init indicates whether the factories should be filled with the
     * default list of objects
     */
    MachineSupport(bool init = true) {
        if(init) {
            this->initMachines();
        }
    }

    /**
     * Destructor.
     */
    ~MachineSupport() {}
    
    /**
     * Fills the IMachineLearner factory with a default set of objects.
     */
    void initMachines() {
        this->getMachineFactory().registerPrototype(new DummyLearner());
        this->getMachineFactory().registerPrototype(new RLSLearner());
        this->getMachineFactory().registerPrototype(new LSSVMLearner());
#ifdef BUILD_LSSVMATLAS
        this->getMachineFactory().registerPrototype(new LSSVMAtlasLearner());
#endif
        this->getMachineFactory().registerPrototype(new DatasetRecorder());
    }
    
    /**
     * Asks the Support class to return a reference to the machine factory.
     *
     * @return a reference to the machine factory.
     */
    MachineFactory& getMachineFactory() {
        return this->machineFactory;
    }
};

} // learningmachine
} // contrib
} // iCub

#endif
