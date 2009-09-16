/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Support class for shared objects, such as factories.
 *
 */

#ifndef __ICUB_TRANSFORMERSUPPORT__
#define __ICUB_TRANSFORMERSUPPORT__

#include <string>

#include "iCub/Support.h"
#include "iCub/ITransformer.h"
#include "iCub/IScaler.h"
#include "iCub/FactoryT.h"

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
 * The TransformerSupport class provides a unified access point to all 
 * transformer related FactoryT instances and shares the common base provided 
 * by the general Support class.
 *
 * \author Arjan Gijsberts
 *
 */

typedef FactoryT<std::string, ITransformer> TransformerFactory;
typedef FactoryT<std::string, IScaler> ScalerFactory;

class TransformerSupport : virtual public Support {
protected:
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
    TransformerSupport(bool init = true) {
        if(init) {
            this->initTransformers();
            this->initScalers();
        }
    }

    /**
     * Destructor.
     */
    ~TransformerSupport() {}
    
    /**
     * Fills the ITransformer factory with a default set of objects.
     */
    void initTransformers() {
        // register transformers
        this->getTransformerFactory().registerPrototype(new ScaleTransformer(this));
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
