/*
 * Copyright (C) 2019 iCub-Tech Facility - Istituto Italiano di Tecnologia
 * Author:  Valentina Gaggero
 * email:   Valentina.gaggero@iit.it
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

// - include guard ----------------------------------------------------------------------------------------------------


#ifndef _PERIODIC_EVENTS_VERIFIER_H_
#define _PERIODIC_EVENTS_VERIFIER_H_

// - namespace embot::tools belongs to the embot (emBEDDED RObot) libray of C++11 classes and functions which are 
// - designed to run in the icub/r1 robot's embedded environment.
// - the objects inside this namespace are tools used to validate behaviours in the microcontrollers inside the robot
// - but can also be used inside icub-main classes which runs on the PC104 platform
// - hence particular attention was put in avoiding any call to YARP or embot::sys (RTOS) or embot::hw (HW of the micro). 
// - we also don't use in here any embot::common funtions or types to guarantee maximum portability.





namespace Tools 
{ 
    class Emb_PeriodicEventVerifier;
    class Emb_RensponseTimingVerifier;
}

class Tools::Emb_PeriodicEventVerifier
{

    public:
                
        Emb_PeriodicEventVerifier();
        ~Emb_PeriodicEventVerifier();
    
        bool init(double period, double tolerance, double min, double max, double step, double reportPeriod);
        void tick(double currentTime);
    private:
        struct Impl;
        Impl *pImpl;
    
};


class Tools::Emb_RensponseTimingVerifier
{

    public:
                
        Emb_RensponseTimingVerifier();
        ~Emb_RensponseTimingVerifier();
    
        bool init(double desiredResponseTime, double tolerance, double min, double max, double step, double reportPeriod);
        void tick(double currentResponseTime, double requestTime);
    private:
        struct Impl;
        Impl *pImpl;
    
};



#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

