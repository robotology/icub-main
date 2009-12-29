// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
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

#include <iostream>
#include <vector>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <math.h>

#include <iostream>
#include <iomanip>


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

#define MAX_EST_VELOCITY        200
#define MAX_EST_POSITION        200
#define MAX_EST_ACCEL          1000
#define MAX_SETPOINT_POSITION   200
#define MAX_DEC_FACTOR          0.2
#define MAX_EST_COEFS0          120
#define MAX_EST_COEFS1           20


class ICubShoulderDecouplingBox
{
public:
    double              mDecLearningFactor;

    double              mDecVel[3];
    double              mDecAccum[3];
    double              mDecFactors[3];
    double              mDecTimeConstant;
    
    double              mVCEstSetPoint[3];
    double              mSetPointResetFactor;

    double              mEstLearningFactor0;
    double              mEstLearningFactor1;
    double              mEstCoefs[3][2];
    double              mEstPos[3];
    double              mEstVel[3];
    double              mEstAcc[3];

public:
    ICubShoulderDecouplingBox();
    ~ICubShoulderDecouplingBox();
    
    void    Init();
    
    void    Decouple(Vector &inputVel, Vector &outputVel, double dt);
    void    LearnDecoupling(Vector &targetVel, Vector &currPos, Vector &currVel, double dt);

    void    ResetSetPoint(Vector &currPos);
    void    UpdateSetPoint(Vector &targetVel, Vector &currPos, double dt);

    void    LearnModel(Vector &targetVel, Vector &currPos, Vector &currVel, Vector &currAcc, double dt, int index);

    void    ProcessModel(Vector &targetVel, double dt);

};

