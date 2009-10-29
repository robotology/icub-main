/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Event listener factory class for creating types of IEventListener objects.
 *
 */

#ifndef __ICUB_EVENTLISTENERFACTORY__
#define __ICUB_EVENTLISTENERFACTORY__

#include "iCub/FactoryT.h"
#include "iCub/IEventListener.h"

namespace iCub {
namespace learningmachine {

typedef FactoryT<std::string, IEventListener> EventListenerFactory;

} // learningmachine
} // iCub

#endif
