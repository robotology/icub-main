/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#ifndef __LOCALIZER_H__
#define __LOCALIZER_H__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>

#include <iCub/ctrl/pids.h>

#include <iCub/gazeNlp.hpp>
#include <iCub/utils.hpp>
                            
#define KP_EYE_X            0.0010
#define KP_EYE_Y            0.0010
#define KP_EYE_Z            0.0005
#define KI_EYE_X            0.0001
#define KI_EYE_Y            0.0001
#define KI_EYE_Z            0.0001
#define LIM_LOW             -0.1
#define LIM_HIGH            0.1

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


// The thread launched by the application which is
// in charge of localizing target 3D position from
// image coordinates.
class Localizer : public RateThread
{
protected:
    exchangeData         *commData;
    xdPort               *port_xd;
    BufferedPort<Bottle> *port_mono;
    BufferedPort<Bottle> *port_stereo;
    BufferedPort<Bottle> *port_anglesIn;
    BufferedPort<Vector> *port_anglesOut;
    
    string localName;
    string configFile;
    unsigned int period;
    double Ts;

    iCubHeadCenter  eyeC;
    iCubEye        *eyeL;
    iCubEye        *eyeR;

    Matrix eyeCAbsFrame;
    Matrix invEyeCAbsFrame;

    Matrix *PrjL, *invPrjL;
    Matrix *PrjR, *invPrjR;
    double  cx;
    double  cy;

    parallelPID *pid;

    void handleMonocularInput();
    void handleStereoInput();
    void handleAnglesInput();
    void handleAnglesOutput();

public:
    Localizer(exchangeData *_commData, const string &_localName,
              const string &_configFile, unsigned int _period);

    void set_xdport(xdPort *_port_xd) { port_xd=_port_xd; }

    virtual bool threadInit();
    virtual void afterStart(bool s);
    virtual void run();
    virtual void threadRelease();
};


#endif


