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
    string type;
    iCubArm *arm;
    iCubArm *elbow;
    Vector point;

public:
    /*********************************************************************/
    PointingFarNLP(ICartesianControl *iarm) :
        point(3,0.0)
    {
        Bottle info;
        iarm->getInfo(info);
        type=info.find("arm_type").asString();
        arm=new iCubArm(type);
        elbow=new iCubArm(type);

        size_t underscore=type.find('_');
        if (underscore!=string::npos)
            type=type.substr(0,underscore);

        // update joints limits
        for (size_t i=0; i<arm->getN(); i++)
        {
            double min,max;
            iarm->getLimits(i,&min,&max);
            (*arm->asChain())[i].setMin(DEG2RAD*min);
            (*arm->asChain())[i].setMax(DEG2RAD*max);
            (*elbow->asChain())[i].setMin(DEG2RAD*min);
            (*elbow->asChain())[i].setMax(DEG2RAD*max);
        }

        // arm: block torso
        arm->blockLink(0,0.0);
        arm->blockLink(1,0.0);
        arm->blockLink(2,0.0);

        // arm: block wrist
        arm->blockLink(8,0.0);
        arm->blockLink(9,0.0);

        // elbow: block torso
        elbow->blockLink(0,0.0);
        elbow->blockLink(1,0.0);
        elbow->blockLink(2,0.0);

        // elbow: remove wrist,
        // pronosupination and elbow
        elbow->asChain()->popLink();
        elbow->asChain()->popLink();
        elbow->asChain()->popLink();
        elbow->asChain()->popLink();

        arm->asChain()->setAllConstraints(false);
        elbow->asChain()->setAllConstraints(false);
    }

    /*********************************************************************/
    virtual ~PointingFarNLP()
    {
        delete arm;
        delete elbow;
    }

    /*********************************************************************/
    void setRequirements(const Vector& point)
    {
        this->point=point;
    }

    /*********************************************************************/
    void getResult(Vector &q, Vector &x) const
    {
        q.resize(arm->getN());
        for (size_t i=0; i<q.length(); i++)
            q[i]=RAD2DEG*arm->getAng(i);
        x=arm->EndEffPose();
    }

    /*********************************************************************/
    bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                      Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style)
    {
        n=arm->getDOF();
        m=1;
        nnz_jac_g=n;
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
            x_l[i]=(*arm->asChain())(i).getMin();
            x_u[i]=(*arm->asChain())(i).getMax();
        }

        g_l[0]=g_u[0]=1.0;
        return true;
    }

    /*********************************************************************/
    bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                            bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                            Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda)
    {
        for (Ipopt::Index i=0; i<n; i++)
            x[i]=0.5*((*arm->asChain())(i).getMin()+(*arm->asChain())(i).getMax());
        return true;
    }

    /*********************************************************************/
    void setAng(const Ipopt::Number* x)
    {
        Vector q(arm->getDOF());
        for (size_t i=0; i<q.length(); i++)
           q[i]=x[i];
        arm->setAng(q);
        elbow->setAng(q.subVector(0,elbow->getDOF()-1));
    }

    /*********************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                Ipopt::Number& obj_value)
    {
        setAng(x);
        Matrix handH=arm->getH();
        double e=(type=="left"?1.0:-1.0)-handH(2,2);
        obj_value=e*e;
        return true;
    }

    /*********************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number* grad_f)
    {
        setAng(x);
        Matrix handH=arm->getH();
        double e=(type=="left"?1.0:-1.0)-handH(2,2);
        Matrix dhandHZ=arm->AnaJacobian(2);        
        for (Ipopt::Index i=0; i<n; i++)
            grad_f[i]=-2.0*e*dhandHZ(2,i);
        return true;
    }

    /*********************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                Ipopt::Index m, Ipopt::Number* g)
    {
        setAng(x);
        Vector elbowPos=elbow->EndEffPosition();
        Vector pe_dir=point-elbowPos;
        Vector he_dir=arm->EndEffPosition()-elbowPos;
        g[0]=dot(pe_dir,he_dir)/(norm(pe_dir)*norm(he_dir));        
        return true;
    }

    /*********************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow,
                    Ipopt::Index *jCol, Ipopt::Number* values)
    {
        if (!values)
        {            
            for (Ipopt::Index i=0; i<n; i++)
            {
                iRow[i]=0;
                jCol[i]=i;
            }
        }
        else
        {
            setAng(x);
            Vector elbowPos=elbow->EndEffPosition();
            Vector pe_dir=point-elbowPos;
            Vector he_dir=arm->EndEffPosition()-elbowPos;

            double npe=norm(pe_dir);
            double nhe=norm(he_dir);
            double nn=npe*nhe;

            Matrix dpe_dir=-1.0*cat(elbow->AnaJacobian().removeRows(3,3),zeros(3,2));
            Matrix dhe_dir=arm->AnaJacobian().removeRows(3,3)+dpe_dir;

            for (Ipopt::Index i=0; i<n; i++)
                values[i]=(dot(dpe_dir.getCol(i),he_dir)+dot(pe_dir,dhe_dir.getCol(i)))/nn-
                          dot(pe_dir,he_dir)*(dot(pe_dir,dpe_dir.getCol(i))/npe)/(nn*nn);
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
    app->Options()->SetNumericValue("tol",0.001);
    app->Options()->SetNumericValue("constr_viol_tol",0.001);
    app->Options()->SetIntegerValue("acceptable_iter",0);
    app->Options()->SetStringValue("mu_strategy","adaptive");
    app->Options()->SetIntegerValue("max_iter",1000);
    app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
    app->Options()->SetStringValue("hessian_approximation","limited-memory");
    app->Options()->SetStringValue("derivative_test","none");
    app->Options()->SetIntegerValue("print_level",0);
    app->Initialize();

    Ipopt::SmartPtr<PointingFarNLP> nlp=new PointingFarNLP(iarm);
    nlp->setRequirements(point);
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

        iarm->setInTargetTol(0.02);
        iarm->goToPoseSync(x.subVector(0,2),x.subVector(3,6));
        iarm->waitMotionDone();

        iarm->restoreContext(context);
        iarm->deleteContext(context);
        return true;
    }
    else
        return false;
}


