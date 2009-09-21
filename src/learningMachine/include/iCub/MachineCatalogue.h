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

#include "iCub/MachineFactory.h"

#include "iCub/DummyLearner.h"
#include "iCub/RLSLearner.h"
#include "iCub/LSSVMLearner.h"
#ifdef BUILD_LSSVMATLAS
#include "iCub/LSSVMAtlasLearner.h"
#endif
#include "iCub/DatasetRecorder.h"


namespace iCub {
namespace contrib {
namespace learningmachine {

void registerMachines() {
    MachineFactory::instance().registerPrototype(new DummyLearner());
    MachineFactory::instance().registerPrototype(new RLSLearner());
    MachineFactory::instance().registerPrototype(new LSSVMLearner());
#ifdef BUILD_LSSVMATLAS
    MachineFactory::instance().registerPrototype(new LSSVMAtlasLearner());
#endif
    MachineFactory::instance().registerPrototype(new DatasetRecorder());
}

} // learningmachine
} // contrib
} // iCub

#endif
