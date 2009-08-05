/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Simple header file for including various known transformers easily.
 *
 */

#ifndef __ICUB_TRANSFORMERCATALOGUE__
#define __ICUB_TRANSFORMERCATALOGUE__

#include "iCub/TransformerFactory.h"
#include "iCub/ScaleTransformer.h"
#include "iCub/Standardizer.h"
#include "iCub/Normalizer.h"
#include "iCub/FixedRangeScaler.h"
#include "iCub/RandomFeature.h"


namespace iCub {
namespace contrib {
namespace learningmachine {

void registerTransformers() {
    // register scalers
    ScalerFactory::instance().registerPrototype(new Standardizer);
    ScalerFactory::instance().registerPrototype(new Normalizer);
    ScalerFactory::instance().registerPrototype(new FixedRangeScaler);

    // register proper transformers
    TransformerFactory::instance().registerPrototype(new ScaleTransformer);
    TransformerFactory::instance().registerPrototype(new RandomFeature);
    
    
}

} // learningmachine
} // contrib
} // iCub

#endif
