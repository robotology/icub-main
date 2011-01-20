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

#include <yarp/sig/Vector.h>

#include <iCub/iKin/iKinInv.h>
#include <iCub/iKin/iKinIpOpt.h>

using namespace std;
using namespace yarp;
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
    virtual void allocate(const string &_type);

public:
    iCubHeadCenter()                           { allocate("right"); }
    iCubHeadCenter(const iCubHeadCenter &head) { clone(head);       }
};


// Describe the nonlinear problem of aligning two vectors
// in counterphase for controlling neck movements.
class HeadCenter_NLP : public iKin_NLP
{
private:
    HeadCenter_NLP(const HeadCenter_NLP&);
    HeadCenter_NLP &operator=(const HeadCenter_NLP&);

protected:
    iKinChain dummyChain;
    Vector    dummyVector;

    Matrix Hxd;
    Matrix GeoJacobP;
    Matrix AnaJacobZ;

    Vector qRest;

    double mod;
    double cosAng;
    double fPitch;
    double dfPitch;

    virtual void computeQuantities(const Ipopt::Number *x);

public:
    HeadCenter_NLP(iKinChain &c, const Vector &_q0, Vector &_xd,
                   iKinLinIneqConstr &_LIC, bool *_exhalt=NULL) :
                   iKin_NLP(c,IKINCTRL_POSE_XYZ,_q0,_xd,
                            0.0,dummyChain,dummyVector,dummyVector,
                            0.0,dummyVector,dummyVector,
                            _LIC,_exhalt) { qRest.resize(dim,0.0); }

    virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                              Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style);
    virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                                 Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u);
    virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value);
    virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f);
    virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g);
    virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,Ipopt::Index m, Ipopt::Index nele_jac,
                            Ipopt::Index* iRow, Ipopt::Index *jCol, Ipopt::Number* values);

    void setGravityDirection(const Vector &gDir);
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

    void set_ctrlPose(const unsigned int _ctrlPose) { }
    void setHessianOpt(const bool useHessian) { }   // Hessian not implemented
    virtual Vector solve(const Vector &q0, Vector &xd, const Vector &gDir,
                         Ipopt::ApplicationReturnStatus *exit_code=NULL, bool *exhalt=NULL,
                         iKinIterateCallback *iterate=NULL);
};


#endif


