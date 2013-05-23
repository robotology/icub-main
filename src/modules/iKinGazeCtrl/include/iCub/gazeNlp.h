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

#ifndef __GAZENLP_H__
#define __GAZENLP_H__

#include <string>

#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinInv.h>
#include <iCub/iKin/iKinIpOpt.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


// Compute fixation point position and Jacobian wrt eyes (tilt,pan,vergence)
// Return true if division by zero is detected
bool computeFixationPointData(iKinChain &eyeL, iKinChain &eyeR, Vector &fp, Matrix &J);


// Compute fixation point position wrt eyes (tilt,pan,vergence)
// Return true if division by zero is detected
bool computeFixationPointOnly(iKinChain &eyeL, iKinChain &eyeR, Vector &fp);


// Describe the kinematic of the straight line
// coming out from the point located between eyes.
class iCubHeadCenter : public iCubEye
{
protected:
    void allocate(const string &_type);

public:
    iCubHeadCenter()                           { allocate("right"); }
    iCubHeadCenter(const string &_type)        { allocate(_type);   }
    iCubHeadCenter(const iCubHeadCenter &head) { clone(head);       }
};


// Solve through IPOPT the nonlinear problem 
class GazeIpOptMin : public iKinIpOptMin
{
private:
    GazeIpOptMin();
    GazeIpOptMin(const GazeIpOptMin&);
    GazeIpOptMin &operator=(const GazeIpOptMin&);

public:
    GazeIpOptMin(iKinChain &_chain, const double tol, const int max_iter=IKINCTRL_DISABLED,
                 const unsigned int verbose=0) :
                 iKinIpOptMin(_chain,IKINCTRL_POSE_XYZ,tol,max_iter,verbose,false) { }

    void   set_ctrlPose(const unsigned int _ctrlPose) { }
    void   setHessianOpt(const bool useHessian)       { }   // Hessian not implemented
    Vector solve(const Vector &q0, Vector &xd, const Vector &gDir);
};


#endif


