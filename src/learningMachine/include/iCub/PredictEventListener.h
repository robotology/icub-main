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

#ifndef LM_PREDICTEVENTLISTENER__
#define LM_PREDICTEVENTLISTENER__

#include <string>

#include <yarp/sig/Vector.h>

#include "iCub/IPortEventListener.h"
#include "iCub/PredictEvent.h"

namespace iCub {
namespace learningmachine {

/**
 *
 * \see iCub::learningmachine::IPortEventListener
 *
 * \author Arjan Gijsberts
 */

class PredictEventListener : public IPortEventListener {
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
    PredictEventListener(std::string pp = "/lm/event/predict") : IPortEventListener(pp) {
        this->setName("Predict");
    }

    /*
     * Inherited from IEventListener.
     */
    void handle(PredictEvent& e);

    /*
     * Inherited from IEventListener.
     */
    PredictEventListener* clone() {
        return new PredictEventListener(*this);
    }

};

} // learningmachine
} // iCub

#endif
