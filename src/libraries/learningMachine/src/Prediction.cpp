/*
 * Copyright (C) 2007-2011 RobotCub Consortium, European Commission FP6 Project IST-004370
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

#include <cassert>

//#include "iCub/learningMachine/Prediction.h"

namespace iCub {
namespace learningmachine {
// whole implementation is moved to header
// move back here in case of problems
/*
void Prediction::setVariance(const yarp::sig::Vector& variance) {
    assert(prediction.size() == variance.size() || variance.size() == 0);
    this->variance = variance;
    this->varianceSet = (variance.size() > 0);
}

std::string Prediction::toString() {
    std::string out = this->prediction.toString().c_str();
    if(this->hasVariance()) {
        out += std::string(" +/- ") + this->variance.toString().c_str();
    }
    return out;
}

bool Prediction::write(yarp::os::ConnectionWriter& connection) {
    // follows PortablePair implementation
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(2);

    bool ok = this->prediction.write(connection);
    if (ok) {
        ok = this->variance.write(connection);
    }

    if (ok) {
        connection.convertTextMode();
    }

    return ok;
}

bool Prediction::read(yarp::os::ConnectionReader& connection) {
    // follows PortablePair implementation
    connection.convertTextMode();

    int header = connection.expectInt();
    if(header != BOTTLE_TAG_LIST) {
        return false;
    }

    int len = connection.expectInt();
    if(len != 2) {
        return false;
    }

    bool ok = this->prediction.read(connection);
    if (ok) {
        ok = this->variance.read(connection);
    }
    this->varianceSet = (this->variance.size() > 0);
    return ok;
}

void Prediction::onCompletion() {
    this->prediction.onCompletion();
    this->variance.onCompletion();
}
*/

} // learningmachine
} // iCub
