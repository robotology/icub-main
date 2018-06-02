/* 
 * Copyright (C) 2014 Francesco Giovannini, iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Francesco Giovannini
 * email:   francesco.giovannini@iit.it
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



#ifndef __SKINMANAGER_DIAGNOSTICSREAD_THREAD_H__
#define __SKINMANAGER_DIAGNOSTICSREAD_THREAD_H__

#include <string>
#include <vector>
#include <deque>

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>

namespace iCub {
    namespace skinManager {
        class SkinDiagnosticsReadThread : public yarp::os::PeriodicThread {
            private:
                /* ****** Module attributes                             ****** */
                /** The thread period. */
                int period;

                /* ******* Ports.                                       ******* */
                yarp::os::BufferedPort<yarp::sig::Vector> portSkinDiagnosticsErrorsIn;
                yarp::os::BufferedPort<yarp::os::Bottle> portSkinManagerErrorsOut;
                

                /* ****** Resource finder                                ****** */
                yarp::os::ResourceFinder rf;

                /* ****** Debug attributes                              ****** */
                std::string dbgTag;

            public:
                SkinDiagnosticsReadThread(const int aPeriod, const yarp::os::ResourceFinder &aRf);
                virtual ~SkinDiagnosticsReadThread();

                virtual bool threadInit(void);
                virtual void run(void);
                virtual void threadRelease(void);
        };
    }
}

#endif

