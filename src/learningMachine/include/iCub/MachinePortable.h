/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Convenience header to define MachinePortable type.
 *
 */

#ifndef __ICUB_MACHINEPORTABLE__
#define __ICUB_MACHINEPORTABLE__

#include "iCub/PortableT.h"
#include "iCub/IMachineLearner.h"


namespace iCub {
namespace learningmachine {

typedef PortableT<IMachineLearner> MachinePortable;

} // learningmachine
} // iCub

#endif
