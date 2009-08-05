/*
 * Copyright (C) 2007-2008 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Machine factory class for creating types of IMachineLearner objects.
 *
 */

#ifndef __ICUB_MACHINEFACTORY__
#define __ICUB_MACHINEFACTORY__

#include "iCub/FactoryT.h"

namespace iCub {
namespace contrib {
namespace learningmachine {

typedef FactoryT<std::string, IMachineLearner> MachineFactory;

} // learningmachine
} // contrib
} // iCub

#endif
