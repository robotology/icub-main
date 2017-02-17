/* 
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
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

#include <string>
#include <cmath>
#include <limits>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

#include <yarp/os/Property.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/pointing_far.h>

#define RAD2DEG     (180.0/M_PI)
#define DEG2RAD     (M_PI/180.0)

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;


/*************************************************************************/
class PointingFarNLP : public Ipopt::TNLP
{
    iCubArm *finger_tip;
    iCubArm *finger_base;
    iCubFinger *finger;
    Vector point;

public:
    /*********************************************************************/
    PointingFarNLP(ICartesianControl *iarm) :
        point(3,0.0)
    {
        Bottle info;
        iarm->getInfo(info);
        string hand=info.find("arm_type").asString();
        finger_tip=new iCubArm(hand);
        finger_base=new iCubArm(hand);

        size_t underscore=hand.find('_');
        if (underscore!=string::npos)
            hand=hand.substr(0,underscore);

        // update joints limits
        for (size_t i=0; i<finger_tip->getN(); i++)
        {
            double min,max;
            iarm->getLimits(i,&min,&max);
            (*finger_tip->asChain())[i].setMin(DEG2RAD*min);
            (*finger_tip->asChain())[i].setMax(DEG2RAD*max);
            (*finger_base->asChain())[i].setMin(DEG2RAD*min);
            (*finger_base->asChain())[i].setMax(DEG2RAD*max);
        }

        // finger_tip: block torso
        finger_tip->blockLink(0,0.0);
        finger_tip->blockLink(1,0.0);
        finger_tip->blockLink(2,0.0);

        // finger_tip: block wrist
        finger_tip->blockLink(8,0.0);
        finger_tip->blockLink(9,0.0);

        // finger_base: block torso
        finger_base->blockLink(0,0.0);
        finger_base->blockLink(1,0.0);
        finger_base->blockLink(2,0.0);

        // finger_base: block wrist
        finger_base->blockLink(8,0.0);
        finger_base->blockLink(9,0.0);

        // remove all constraints
        finger_tip->asChain()->setAllConstraints(false);
        finger_base->asChain()->setAllConstraints(false);

        finger=new iCubFinger(hand+"_index");
        finger->asChain()->setAllConstraints(false);
        setFingerJoints(zeros(16));
    }

    /*********************************************************************/
    virtual ~PointingFarNLP()
    {
        delete finger_tip;
        delete finger_base;
        delete finger;
    }

    /*********************************************************************/
    void setPoint(const Vector& point)
    {
        this->point=point;
    }

    /*********************************************************************/
    void setFingerJoints(const Vector& finger_joints)
    {        
        Vector chain_joints;
        finger->getChainJoints(finger_joints,chain_joints);
        finger->setAng(DEG2RAD*chain_joints);

        // add final transformations
        finger_tip->asChain()->setHN(finger->getH());
        finger_base->asChain()->setHN(finger->getH0());
    }

    /*********************************************************************/
    void getResult(Vector &q, Vector &x) const
    {
        q.resize(finger_tip->getN());
        for (size_t i=0; i<q.length(); i++)
            q[i]=RAD2DEG*finger_tip->getAng(i);
        x=finger_tip->EndEffPose();
    }

    /*********************************************************************/
    bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                      Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style)
    {
        n=finger_tip->getDOF();
        m=2;
        nnz_jac_g=m*n;
        nnz_h_lag=0;
        index_style=TNLP::C_STYLE;
        return true;
    }

    /*********************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                         Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u)
    {
        for (Ipopt::Index i=0; i<n; i++)
        {
            x_l[i]=(*finger_tip->asChain())(i).getMin();
            x_u[i]=(*finger_tip->asChain())(i).getMax();
        }

        g_l[0]=1.0;
        g_u[0]=std::numeric_limits<double>::max();

        g_l[1]=g_u[1]=0.0;
        return true;
    }

    /*********************************************************************/
    bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                            bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                            Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda)
    {
        for (Ipopt::Index i=0; i<n; i++)
            x[i]=0.5*((*finger_tip->asChain())(i).getMin()+
                      (*finger_tip->asChain())(i).getMax());
        return true;
    }

    /*********************************************************************/
    void setAng(const Ipopt::Number* x)
    {
        Vector q(finger_tip->getDOF());
        for (size_t i=0; i<q.length(); i++)
           q[i]=x[i];
        finger_tip->setAng(q);
        finger_base->setAng(q);
    }

    /*********************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                Ipopt::Number& obj_value)
    {
        obj_value=x[3]*x[3];
        return true;
    }

    /*********************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number* grad_f)
    {
        for (Ipopt::Index i=0; i<n; i++)
            grad_f[i]=(i==3?2.0*x[3]:0.0);
        return true;
    }

    /*********************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                Ipopt::Index m, Ipopt::Number* g)
    {
        setAng(x);

        Vector fingerBasePos=finger_base->EndEffPosition();
        Vector pb_dir=point-fingerBasePos;

        Matrix tip=finger_tip->getH();
        Vector tb_dir=tip.getCol(3).subVector(0,2)-fingerBasePos;
        g[0]=dot(pb_dir,tb_dir)/(norm(pb_dir)*norm(tb_dir));
        
        double e=-1.0-tip(2,2);
        g[1]=e*e;
        return true;
    }

    /*********************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow,
                    Ipopt::Index *jCol, Ipopt::Number* values)
    {
        if (!values)
        {
            Ipopt::Index idx=0;
            for (Ipopt::Index i=0; i<n; i++,idx++)
            {
                iRow[idx]=0;
                jCol[idx]=i;
            }
            for (Ipopt::Index i=0; i<n; i++,idx++)
            {
                iRow[idx]=1;
                jCol[idx]=i;
            }
        }
        else
        {
            setAng(x);
            Vector fingerBasePos=finger_base->EndEffPosition();
            Vector pb_dir=point-fingerBasePos;

            Matrix tip=finger_tip->getH();
            Vector tb_dir=tip.getCol(3).subVector(0,2)-fingerBasePos;

            Matrix dpb_dir=-1.0*finger_base->AnaJacobian().removeRows(3,3);
            Matrix dtb_dir=finger_tip->AnaJacobian().removeRows(3,3)+dpb_dir;

            double npb=norm(pb_dir);
            double ntb=norm(tb_dir);
            double nn=npb*ntb;
            double tmp1=dot(pb_dir,tb_dir);
            double tmp2=ntb/npb;
            double tmp3=nn*nn;

            for (Ipopt::Index i=0; i<n; i++)
                values[i]=(dot(dpb_dir.getCol(i),tb_dir)+dot(pb_dir,dtb_dir.getCol(i)))/nn-
                          tmp1*(dot(pb_dir,dpb_dir.getCol(i))*tmp2)/tmp3;
            
            double e=-1.0-tip(2,2);
            Matrix dtipZ=finger_tip->AnaJacobian(2);
            for (Ipopt::Index i=0; i<n; i++)
                values[n+i]=-2.0*e*dtipZ(2,i);
        }
        return true;
    }

    /*********************************************************************/
    bool eval_h(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number obj_factor, Ipopt::Index m,
                const Ipopt::Number *lambda, bool new_lambda,
                Ipopt::Index nele_hess, Ipopt::Index *iRow,
                Ipopt::Index *jCol, Ipopt::Number *values)
    {
        return true;
    }

    /*********************************************************************/
    void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                           const Ipopt::Number* x, const Ipopt::Number* z_L,
                           const Ipopt::Number* z_U, Ipopt::Index m,
                           const Ipopt::Number* g, const Ipopt::Number* lambda,
                           Ipopt::Number obj_value, const Ipopt::IpoptData* ip_data,
                           Ipopt::IpoptCalculatedQuantities* ip_cq)
    {
    }
};


/*************************************************************************/
bool PointingFar::compute(ICartesianControl *iarm, const Property& requirements,
                          Vector& q, Vector& x)
{
    if (iarm==NULL)
    {
        yError()<<"Cartesian controller not configured";
        return false;
    }

    Bottle *bPoint=requirements.find("point").asList();
    if (bPoint==NULL)
    {
        yError()<<"Target point not provided";
        return false;
    }
    
    Vector point(3,0.0);
    for (int i=0; i<bPoint->size(); i++)
        point[i]=bPoint->get(i).asDouble();

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",0.01);
    app->Options()->SetNumericValue("constr_viol_tol",0.01);
    app->Options()->SetIntegerValue("acceptable_iter",0);
    app->Options()->SetStringValue("mu_strategy","adaptive");
    app->Options()->SetIntegerValue("max_iter",100);
    app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
    app->Options()->SetStringValue("hessian_approximation","limited-memory");
    app->Options()->SetStringValue("derivative_test","none");
    app->Options()->SetIntegerValue("print_level",0);
    app->Initialize();

    Ipopt::SmartPtr<PointingFarNLP> nlp=new PointingFarNLP(iarm);

    // specify requirements
    nlp->setPoint(point);
    if (Bottle *bFingerJoints=requirements.find("finger-joints").asList())
    {
        Vector finger_joints(bFingerJoints->size());
        for (size_t i=0; i<finger_joints.length(); i++)
            finger_joints[i]=bFingerJoints->get(i).asDouble();
        nlp->setFingerJoints(finger_joints);
    }

    Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));    
    nlp->getResult(q,x);
    return true;
}


/*************************************************************************/
bool PointingFar::point(ICartesianControl *iarm, const Vector& q, const Vector& x)
{
    if ((q.length()>=10) && (x.length()>=7))
    {
        int context;
        iarm->storeContext(&context);        

        // specify all joints
        Vector dof(10,1.0);
        iarm->setDOF(dof,dof);
        for (size_t i=0; i<q.length(); i++)
            iarm->setLimits(i,q[i],q[i]);

        double traj;
        iarm->getTrajTime(&traj);
        iarm->setInTargetTol(0.02);
        iarm->goToPoseSync(x.subVector(0,2),x.subVector(3,6));
        iarm->waitMotionDone(0.2,4.0*traj);

        iarm->restoreContext(context);
        iarm->deleteContext(context);
        return true;
    }
    else
        return false;
}


