/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation for the abstract event listener class.
 *
 */

#include "iCub/IEventListener.h"

namespace iCub {
namespace contrib {
namespace learningmachine {

IEventListener::IEventListener(std::string name) {
    this->setName(name);
}

} // learningmachine
} // contrib
} // iCub

