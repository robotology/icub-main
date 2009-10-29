/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Simple header file for including various known transformers easily.
 *
 */

#ifndef __ICUB_TRANSFORMERCATALOGUE__
#define __ICUB_TRANSFORMERCATALOGUE__

#include "iCub/ITransformer.h"
#include "iCub/IScaler.h"
#include "iCub/ScaleTransformer.h"
#include "iCub/Standardizer.h"
#include "iCub/Normalizer.h"
#include "iCub/FixedRangeScaler.h"
#include "iCub/RandomFeature.h"


namespace iCub {
namespace learningmachine {

void registerTransformers() {
    // register scalers
    FactoryT<std::string, IScaler>::instance().registerPrototype(new Standardizer);
    FactoryT<std::string, IScaler>::instance().registerPrototype(new Normalizer);
    FactoryT<std::string, IScaler>::instance().registerPrototype(new FixedRangeScaler);

    // register proper transformers
    FactoryT<std::string, ITransformer>::instance().registerPrototype(new ScaleTransformer);
    FactoryT<std::string, ITransformer>::instance().registerPrototype(new RandomFeature);
    
    
}

} // learningmachine
} // iCub

#endif
