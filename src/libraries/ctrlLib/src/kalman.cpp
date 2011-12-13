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

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <iCub/ctrl/kalman.h>

using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


/**********************************************************************/
Kalman::Kalman(const Matrix &_A, const Matrix &_H, const Matrix &_Q,
               const Matrix &_R) : A(_A), H(_H), Q(_Q), R(_R)
{
    n=A.rows();
    m=H.rows();

    At=A.transposed();
    Ht=H.transposed();
    I=eye(n,n);

    x.resize(n,0.0);
    P.resize(n,n); P.zero();
    K.resize(n,m); K.zero();
}


/**********************************************************************/
void Kalman::init(const Vector &_z0, const Vector &_x0, const Matrix &_P0)
{ 
	x=_x0;
    P=_P0;
}


/**********************************************************************/
Vector Kalman::filt(const Vector &z)
{
    // prediction
    x=A*x;
    P=A*P*At+Q;

    // Kalman gain
    K=P*Ht*pinv(H*P*Ht+R);

    // correction
    x=x+K*(z-H*x);
    P=(I-K*H)*P;

    return x;
}



