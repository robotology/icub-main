/*
 * Copyright (C) 2007-2008 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Simple header file for including various known machines to the factory.
 *
 * Sorry for using conditional compilation for adding the different types; it's
 * hard to implement the factory pattern neatly in C++.
 */

#ifndef __ICUB_MACHINECATALOGUE__
#define __ICUB_MACHINECATALOGUE__

#include "iCub/IMachineLearner.h"
#include "iCub/DummyLearner.h"
#include "iCub/RLSLearner.h"
#include "iCub/LSSVMLearner.h"
#ifdef BUILD_LSSVMATLAS
#include "iCub/LSSVMAtlasLearner.h"
#endif
#include "iCub/DatasetRecorder.h"


namespace iCub {
namespace learningmachine {

void registerMachines() {
    FactoryT<std::string, IMachineLearner>::instance().registerPrototype(new DummyLearner());
    FactoryT<std::string, IMachineLearner>::instance().registerPrototype(new RLSLearner());
    FactoryT<std::string, IMachineLearner>::instance().registerPrototype(new LSSVMLearner());
#ifdef BUILD_LSSVMATLAS
    FactoryT<std::string, IMachineLearner>::instance().registerPrototype(new LSSVMAtlasLearner());
#endif
    FactoryT<std::string, IMachineLearner>::instance().registerPrototype(new DatasetRecorder());
}

} // learningmachine
} // iCub

#endif
