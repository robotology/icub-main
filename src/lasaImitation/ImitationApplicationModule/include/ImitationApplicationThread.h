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

#ifndef ImitationApplicationTHREAD_H_
#define ImitationApplicationTHREAD_H_

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::sig;

#include <vector>
#include <string>

using namespace std;

class ImitationApplicationThread: public RateThread
{
private:    
    Semaphore               mMutex;
    int                     mPeriod;
    char                    mBaseName[256];
    
    BufferedPort<Bottle>    mVelocityControllerPort;
    BufferedPort<Bottle>    mRobotControllerPort;
    BufferedPort<Bottle>    m3DMouseControllerPort;
    BufferedPort<Bottle>    mTouchpadControllerPort;

    enum PortId{
        PID_Velocity = 0,
        PID_Robot,
        PID_3DMouse,
        PID_Touchpad,
        PID_SIZE,
    };
    
    
    enum SrcPortId{
        SPID_Test1 = 0,
        SPID_SIZE
    };
    enum DstPortId{
        DPID_Test1 = 0,
        DPID_SIZE
    };
    char                    mSrcPortName[SPID_SIZE][256];
    char                    mDstPortName[DPID_SIZE][256];
    
    vector<int>             mCommandsType;
    vector<SrcPortId>       mConnexionsSrcPort;
    vector<DstPortId>       mConnexionsDstPort;
    vector<string>          mCommands;
    vector<PortId>          mCommandsPort;
    
    BufferedPort<Bottle>   *mPorts[PID_SIZE];
    
    enum State{
        IA_IDLE=0,
        IA_INIT,
        IA_STOP,
        IA_REST,
        IA_RUN
    };
    
    State                   mState;
    State                   mPrevState;
    State                   mNextState;

public:
    ImitationApplicationThread(int period, const char* baseName);
    virtual ~ImitationApplicationThread();

            void    ConnectToNetwork(bool bConnect);

            void    ClearCommands();
            void    SendCommands();
            void    AddCommand(PortId port, const char *cmd);
            void    AddConnexion(SrcPortId src, DstPortId dst);
            void    RemConnexion(SrcPortId src, DstPortId dst);
    
            void    PrepareToStop();

            int     respond(const Bottle& command, Bottle& reply);

    virtual void    run();
    virtual bool    threadInit();
    virtual void    threadRelease();
};

#endif

