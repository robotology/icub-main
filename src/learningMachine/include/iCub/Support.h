/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Support class for shared objects, such as factories.
 *
 */

#ifndef __ICUB_SUPPORT__
#define __ICUB_SUPPORT__

#include <string>

#include "iCub/IMachineLearner.h"
#include "iCub/ITransformer.h"
#include "iCub/IScaler.h"
//#include "iCub/IEventListener.h"

// machines
#include "iCub/DummyLearner.h"
#include "iCub/RLSLearner.h"
#include "iCub/LSSVMLearner.h"
#include "iCub/DatasetRecorder.h"
#ifdef BUILD_LSSVMATLAS // build only if we explicitly want to
#include "iCub/LSSVMAtlasLearner.h"
#endif

// transformers
#include "iCub/ScaleTransformer.h"
#include "iCub/RandomFeature.h"

// scalers
#include "iCub/Standardizer.h"
#include "iCub/Normalizer.h"
#include "iCub/FixedRangeScaler.h"

namespace iCub {
namespace contrib {
namespace learningmachine {


/**
 * The Support class provides a unified access point to all FactoryT instances 
 * and other global type of objects. This avoids the need for a singleton 
 * pattern. 
 *
 * \author Arjan Gijsberts
 *
 */

typedef FactoryT<std::string, IMachineLearner> MachineFactory;
typedef FactoryT<std::string, ITransformer> TransformerFactory;
typedef FactoryT<std::string, IScaler> ScalerFactory;

class Support {
protected:
    /**
     * The IMachineLearner factory.
     */
    MachineFactory machineFactory;

    /**
     * The ITransformer factory.
     */
    TransformerFactory transformerFactory;

    /**
     * The IScaler factory.
     */
    ScalerFactory scalerFactory;

public:
    /**
     * Constructor.
     *
     * @param init indicates whether the factories should be filled with the
     * default list of objects
     */
    Support(bool init = true) {
        if(init) {
            this->initMachines();
            this->initTransformers();
            this->initScalers();
        }
    }

    /**
     * Destructor.
     */
    ~Support() {}
    
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
     * Fills the ITransformer factory with a default set of objects.
     */
    void initTransformers() {
        // register transformers
        this->getTransformerFactory().registerPrototype(new ScaleTransformer);
        this->getTransformerFactory().registerPrototype(new RandomFeature);
    }
    
    /**
     * Fills the IScaler factory with a default set of objects.
     */
    void initScalers() {
        this->getScalerFactory().registerPrototype(new Standardizer);
        this->getScalerFactory().registerPrototype(new Normalizer);
        this->getScalerFactory().registerPrototype(new FixedRangeScaler);
    }
    
    /**
     * Asks the Support class to return a reference to the machine factory.
     *
     * @return a reference to the machine factory.
     */
    MachineFactory& getMachineFactory() {
        return this->machineFactory;
    }
    
    /**
     * Asks the Support class to return a reference to the transformer factory.
     *
     * @return a reference to the transformer factory.
     */
    TransformerFactory& getTransformerFactory() {
        return this->transformerFactory;
    }
    
    /**
     * Asks the Support class to return a reference to the scaler factory.
     *
     * @return a reference to the scaler factory.
     */
    ScalerFactory& getScalerFactory() {
        return this->scalerFactory;
    }

};

} // learningmachine
} // contrib
} // iCub

#endif
