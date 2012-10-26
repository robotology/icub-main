/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#include <yarp/math/Math.h>
#include <iCub/ctrl/tuning.h>

using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;


/**********************************************************************/
OnlineDCMotorParametersEstimator::OnlineDCMotorParametersEstimator()
{
    ekf=NULL;
    Vector state0(4,0.0);
    state0[2]=state0[3]=1.0;
    init(0.01,1.0,1.0,1e5,state0);
}


/**********************************************************************/
void OnlineDCMotorParametersEstimator::init(const double Ts, const double Q,
                                            const double R, const double P0,
                                            const Vector &state0)
{
    this->Ts=Ts;
    if (ekf!=NULL)
    {
        ekf->set_Q(Q*eye(4,4));
        ekf->set_R(R*eye(1,1));
    }
    else
        ekf=new Kalman(Matrix(4,4),Matrix(2,1),Matrix(1,1),Q*eye(4,4),R*eye(1,1));

    ekf->init(state0,P0*eye(4,4));
}


/**********************************************************************/
Vector OnlineDCMotorParametersEstimator::estimate(const double u, const double z)
{
    return Vector(4);
}


/**********************************************************************/
OnlineDCMotorParametersEstimator::~OnlineDCMotorParametersEstimator()
{
    delete ekf;
}

