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
#include <yarp/math/SVD.h>
#include <iCub/ctrl/tuning.h>

using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;


/**********************************************************************/
OnlineDCMotorParametersEstimator::OnlineDCMotorParametersEstimator()
{
    Vector x0(4,0.0);
    x0[2]=x0[3]=1.0;
    init(0.01,1.0,1.0,1e5,x0);
}


/**********************************************************************/
bool OnlineDCMotorParametersEstimator::init(const double Ts, const double Q,
                                            const double R, const double P0,
                                            const Vector &x0)
{
    if (x0.length()<4)
        return false;

    A=F=eye(4,4);
    B.resize(4,0.0);
    C.resize(1,4); C(0,0)=1.0;
    Ct=C.transposed();    

    P=P0*eye(4,4);
    this->Q=Q*eye(4,4);
    this->R=R;

    this->Ts=Ts;
    x=_x=x0.subVector(0,3);
    x[2]=1.0/_x[2];
    x[3]=_x[3]/_x[2];

    return true;
}


/**********************************************************************/
Vector OnlineDCMotorParametersEstimator::estimate(const double u, const double y)
{
    double &x1=x[0];
    double &x2=x[1];
    double &x3=x[2];
    double &x4=x[3];

    double _exp=exp(-Ts*x3);
    double _exp_1=1.0-_exp;
    double _x3_2=x3*x3;
    double _tmp_1=(Ts*x3-_exp_1)/_x3_2;

    A(0,1)=_exp_1/x3;
    A(1,1)=_exp;

    B[0]=x4*_tmp_1;
    B[1]=x4*A(0,1);

    F(0,1)=A(0,1);
    F(1,1)=A(1,1);

    F(0,2)=-(x2*_exp_1)/_x3_2   + (u*x4*Ts*_exp_1)/_x3_2 - (2.0*u*B[0])/x3 + (Ts*x2*_exp)/x3;
    F(1,2)=-(u*x4*_exp_1)/_x3_2 - Ts*x2*_exp             + (u*x4*Ts*_exp)/x3;

    F(0,3)=u*_tmp_1;
    F(1,3)=u*A(0,1);

    // prediction
    x=A*x+B*u;
    P=F*P*F.transposed()+Q;

    // Kalman gain
    Matrix tmpMat=C*P*Ct;
    Matrix K=P*Ct/(tmpMat(0,0)+R);

    // correction
    x+=K*(y-C*x);
    P=(eye(4,4)-K*C)*P;

    _x=x;
    _x[2]=1.0/x[2];
    _x[3]=x[3]/x[2];

    return _x;
}



