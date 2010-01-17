// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author:  Eric Sauser
 * email:   eric.sauser@a3.epfl.ch
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

#ifndef WiimoteServerTHREAD_H_
#define WiimoteServerTHREAD_H_

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::sig;
extern "C"{
#include "wiiuse.h"
}
#ifndef WIN32
  #include <unistd.h>
#endif

#define WIIMOTE_ID 1
#define MAX_WIIMOTES                1


class WiimoteServerThread: public RateThread
{
private:    
    Semaphore               mMutex;
    int                     mPeriod;
    char                    mBaseName[256];
    
    BufferedPort<Vector>    mInputPort;
    BufferedPort<Vector>    mOutputPort;
    
    typedef struct{
        char bA;
        char bB;
        char bUP;
        char bDOWN;

        char bLEFT;
        char bRIGHT;
        char bMINUS;
        char bPLUS;

        char bHOME;
        char bONE;
        char bTWO;
        char bNULL;

    } WiimoteState;

    bool                        bEventMode;
    
public:
    static wiimote**            sWiimotes;
    static WiimoteState         sWState;
    static WiimoteState         sWPrevState;
    static WiimoteState         sWEventState;
    static bool                 sDisconnected;
    static bool                 sGotEvent;

public:
    WiimoteServerThread(int period, const char* baseName);
    virtual ~WiimoteServerThread();

            void SetEventMode(bool event=true);
    
    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();

    static void handle_event(struct wiimote_t* wm);
    static void handle_read(struct wiimote_t* wm, byte* data, unsigned short len);
    static void handle_ctrl_status(struct wiimote_t* wm) ;
    static void handle_disconnect(wiimote* wm);
};

#endif

