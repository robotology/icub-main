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

#include <iCub/iKin/iKinHlp.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinInv.h>
#include <iCub/iKin/iKinIpOpt.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


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


