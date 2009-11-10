/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Event listener factory class for creating types of IEventListener objects.
 *
 */

#ifndef LM_EVENTLISTENERFACTORY__
#define LM_EVENTLISTENERFACTORY__

#include "iCub/FactoryT.h"
#include "iCub/IEventListener.h"

namespace iCub {
namespace learningmachine {

typedef FactoryT<std::string, IEventListener> EventListenerFactory;

} // learningmachine
} // iCub

#endif
