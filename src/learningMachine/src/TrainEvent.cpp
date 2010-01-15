/*
 * Copyright (C) 2007-2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * author:  Arjan Gijsberts
 * email:   arjan.gijsberts@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <sstream>

#include "iCub/TrainEvent.h"

namespace iCub {
namespace learningmachine {

TrainEvent::TrainEvent(const Vector& input, const Vector& desired, const Vector& predicted) {
    this->input = input;
    this->desired = desired;
    this->predicted = predicted;
}

void TrainEvent::visit(IEventListener& listener) {
    listener.handle(*this);
}

std::string TrainEvent::toString() {
    std::ostringstream buffer;
    buffer << "Input: [" << this->input.toString() << "; " << "]  Desired: [" << this->desired.toString() << "; " << "]  Predicted: [" << this->predicted.toString() << "]";
    return buffer.str();
}


} // learningmachine
} // iCub
