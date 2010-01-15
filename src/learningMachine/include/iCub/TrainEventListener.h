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

#ifndef LM_TRAINEVENTLISTENER__
#define LM_TRAINEVENTLISTENER__

#include <string>

#include <yarp/sig/Vector.h>

#include "iCub/IPortEventListener.h"
#include "iCub/TrainEvent.h"

namespace iCub {
namespace learningmachine {

/**
 *
 * \see iCub::learningmachine::IEventListener
 *
 * \author Arjan Gijsberts
 */

class TrainEventListener : public IPortEventListener {
protected:
    void vectorToBottle(const Vector& vec, Bottle& bot) {
        for(int i = 0; i < vec.size(); i++) {
            bot.addDouble(vec[i]);
        }
    }

public:
    /**
     * Constructor
     *
     * @param pp default port prefix
     */
    TrainEventListener(std::string pp = "/lm/event/train") : IPortEventListener(pp) {
        this->setName("Train");
      }

    /*
     * Inherited from IEventListener.
     */
    void handle(TrainEvent& e);

    /*
     * Inherited from IEventListener.
     */
    TrainEventListener* clone() {
        return new TrainEventListener(*this);
    }

};

} // learningmachine
} // iCub

#endif
