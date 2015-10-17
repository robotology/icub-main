/* 
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
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

#include <algorithm>
#include <string>
#include <deque>

#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>

#include <IpTNLP.hpp>
#include <nlp.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


/****************************************************************/
Matrix computeH(const Vector &x)
{
    Matrix H=rpy2dcm(x.subVector(3,5));
    H(0,3)=x[0]; H(1,3)=x[1]; H(2,3)=x[2];

    return H;
}


/****************************************************************/
Matrix computeH(const Ipopt::Number *x)
{
    Vector _x(6);
    for (size_t i=0; i<_x.length(); i++)
        _x[i]=x[i];

    return computeH(_x);
}


/****************************************************************/
class EyeAlignerNLP : public Ipopt::TNLP
{
protected:
    const deque<Vector> &p2d;
    const deque<Vector> &p3d;
    const Matrix        &Prj;

    Vector min;
    Vector max;    
    Vector x0;
    Vector x;

public:
    /****************************************************************/
    EyeAlignerNLP(const deque<Vector> &_p2d,
                  const deque<Vector> &_p3d,
                  const Vector &_min, const Vector &_max,
                  const Matrix &_Prj) :
                  p2d(_p2d), p3d(_p3d), Prj(_Prj)
    {
        min=_min;
        max=_max;
        x0=0.5*(min+max);
    }

    /****************************************************************/
    void set_x0(const Vector &x0)
    {
        size_t len=std::min(this->x0.length(),x0.length());
        for (size_t i=0; i<len; i++)
            this->x0[i]=x0[i];
    }

    /****************************************************************/
    Vector get_result() const
    {
        return x;
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        n=6;
        m=nnz_jac_g=nnz_h_lag=0;
        index_style=TNLP::C_STYLE;

        return true;
    }

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
    {
        for (Ipopt::Index i=0; i<n; i++)
        {
            x_l[i]=min[i];
            x_u[i]=max[i];
        }

        return true;
    }
    
    /****************************************************************/
    bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                            bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
    {
        for (Ipopt::Index i=0; i<n; i++)
            x[i]=x0[i];

        return true;
    }
    
    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {
        Matrix PrjH=Prj*SE3inv(computeH(x));

        obj_value=0.0;
        if (p2d.size()>0)
        {
            for (size_t i=0; i<p2d.size(); i++)
            {
                Vector p2di=PrjH*p3d[i];
                p2di=p2di/p2di[2];
                p2di.pop_back();

                obj_value+=norm2(p2d[i]-p2di);
            }

            obj_value/=p2d.size();
        }

        return true;
    }
    
    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        double roll=x[3];  double cr=cos(roll);  double sr=sin(roll);
        double pitch=x[4]; double cp=cos(pitch); double sp=sin(pitch);
        double yaw=x[5];   double cy=cos(yaw);   double sy=sin(yaw);

        Matrix Rz=eye(4,4);
        Rz(0,0)=cy; Rz(1,1)=cy; Rz(0,1)=-sy; Rz(1,0)=sy;
        Matrix dRz=zeros(4,4);
        dRz(0,0)=-sy; dRz(1,1)=-sy; dRz(0,1)=-cy; dRz(1,0)=cy;

        Matrix Ry=eye(4,4);
        Ry(0,0)=cp; Ry(2,2)=cp; Ry(0,2)=sp; Ry(2,0)=-sp;
        Matrix dRy=zeros(4,4);
        dRy(0,0)=-sp; dRy(2,2)=-sp; dRy(0,2)=cp; dRy(2,0)=-cp;

        Matrix Rx=eye(4,4);
        Rx(1,1)=cr; Rx(2,2)=cr; Rx(1,2)=-sr; Rx(2,1)=sr;
        Matrix dRx=zeros(4,4);
        dRx(1,1)=-sr; dRx(2,2)=-sr; dRx(1,2)=-cr; dRx(2,1)=cr;

        Matrix invRz=Rz.transposed();
        Matrix invRy=Ry.transposed();
        Matrix invRx=Rx.transposed();

        Matrix invR=(-1.0)*invRx*invRy*invRz;
        Vector p(4,-1.0); p[0]=-x[0]; p[1]=-x[1]; p[2]=-x[2];

        Matrix dHdx0=zeros(4,4);                   dHdx0.setCol(3,invR.getCol(0));
        Matrix dHdx1=zeros(4,4);                   dHdx1.setCol(3,invR.getCol(1));
        Matrix dHdx2=zeros(4,4);                   dHdx2.setCol(3,invR.getCol(2));
        Matrix dHdx3=dRx.transposed()*invRy*invRz; dHdx3.setCol(3,dHdx3*p);
        Matrix dHdx4=invRx*dRy.transposed()*invRz; dHdx4.setCol(3,dHdx4*p);
        Matrix dHdx5=invRx*invRy*dRz.transposed(); dHdx5.setCol(3,dHdx5*p);
        
        Matrix dPrjHdx0=Prj*dHdx0;
        Matrix dPrjHdx1=Prj*dHdx1;
        Matrix dPrjHdx2=Prj*dHdx2;
        Matrix dPrjHdx3=Prj*dHdx3;
        Matrix dPrjHdx4=Prj*dHdx4;
        Matrix dPrjHdx5=Prj*dHdx5;

        Matrix PrjH=Prj*SE3inv(computeH(x));

        grad_f[0]=grad_f[1]=grad_f[2]=0.0;
        grad_f[3]=grad_f[4]=grad_f[5]=0.0;
        if (p2d.size()>0)
        {
            for (size_t i=0; i<p2d.size(); i++)
            {
                Vector p2di=PrjH*p3d[i];
                p2di=p2di/p2di[2];
                p2di.pop_back();

                Vector d=p2d[i]-p2di;

                double u_num=dot(PrjH.getRow(0),p3d[i]);
                double v_num=dot(PrjH.getRow(1),p3d[i]);
                double lambda=dot(PrjH.getRow(2),p3d[i]);
                double lambda2=lambda*lambda;
                double tmp_dot;

                Vector dp2d_dx0(2);
                tmp_dot=dot(dPrjHdx0.getRow(2),p3d[i]);
                dp2d_dx0[0]=(dot(dPrjHdx0.getRow(0),p3d[i])*lambda-tmp_dot*u_num)/lambda2;
                dp2d_dx0[1]=(dot(dPrjHdx0.getRow(1),p3d[i])*lambda-tmp_dot*v_num)/lambda2;

                Vector dp2d_dx1(2);
                tmp_dot=dot(dPrjHdx1.getRow(2),p3d[i]);
                dp2d_dx1[0]=(dot(dPrjHdx1.getRow(0),p3d[i])*lambda-tmp_dot*u_num)/lambda2;
                dp2d_dx1[1]=(dot(dPrjHdx1.getRow(1),p3d[i])*lambda-tmp_dot*v_num)/lambda2;

                Vector dp2d_dx2(2);
                tmp_dot=dot(dPrjHdx2.getRow(2),p3d[i]);
                dp2d_dx2[0]=(dot(dPrjHdx2.getRow(0),p3d[i])*lambda-tmp_dot*u_num)/lambda2;
                dp2d_dx2[1]=(dot(dPrjHdx2.getRow(1),p3d[i])*lambda-tmp_dot*v_num)/lambda2;

                Vector dp2d_dx3(2);
                tmp_dot=dot(dPrjHdx3.getRow(2),p3d[i]);
                dp2d_dx3[0]=(dot(dPrjHdx3.getRow(0),p3d[i])*lambda-tmp_dot*u_num)/lambda2;
                dp2d_dx3[1]=(dot(dPrjHdx3.getRow(1),p3d[i])*lambda-tmp_dot*v_num)/lambda2;

                Vector dp2d_dx4(2);
                tmp_dot=dot(dPrjHdx4.getRow(2),p3d[i]);
                dp2d_dx4[0]=(dot(dPrjHdx4.getRow(0),p3d[i])*lambda-tmp_dot*u_num)/lambda2;
                dp2d_dx4[1]=(dot(dPrjHdx4.getRow(1),p3d[i])*lambda-tmp_dot*v_num)/lambda2;

                Vector dp2d_dx5(2);
                tmp_dot=dot(dPrjHdx5.getRow(2),p3d[i]);
                dp2d_dx5[0]=(dot(dPrjHdx5.getRow(0),p3d[i])*lambda-tmp_dot*u_num)/lambda2;
                dp2d_dx5[1]=(dot(dPrjHdx5.getRow(1),p3d[i])*lambda-tmp_dot*v_num)/lambda2;

                grad_f[0]-=2.0*dot(d,dp2d_dx0);
                grad_f[1]-=2.0*dot(d,dp2d_dx1);
                grad_f[2]-=2.0*dot(d,dp2d_dx2);
                grad_f[3]-=2.0*dot(d,dp2d_dx3);
                grad_f[4]-=2.0*dot(d,dp2d_dx4);
                grad_f[5]-=2.0*dot(d,dp2d_dx5);
            }

            for (Ipopt::Index i=0; i<n; i++)
                grad_f[i]/=p2d.size();
        }

        return true;
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {
        return true;
    }

    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values)
    {
        return true;
    }


    /****************************************************************/
    bool eval_h(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number *lambda,
                bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index *iRow,
                Ipopt::Index *jCol, Ipopt::Number *values)
    {
        return true;
    }
    

    /****************************************************************/
    void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                           const Ipopt::Number *x, const Ipopt::Number *z_L,
                           const Ipopt::Number *z_U, Ipopt::Index m,
                           const Ipopt::Number *g, const Ipopt::Number *lambda,
                           Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                           Ipopt::IpoptCalculatedQuantities *ip_cq)
    {
        this->x.resize(n);
        for (Ipopt::Index i=0; i<n; i++)
            this->x[i]=x[i];
    }
};



/****************************************************************/
EyeAligner::EyeAligner() : Prj(eye(3,4))
{
    min.resize(6); max.resize(6);
    min[0]=-1.0;   max[0]=1.0;
    min[1]=-1.0;   max[1]=1.0;
    min[2]=-1.0;   max[2]=1.0;
    min[3]=-M_PI;  max[3]=M_PI;
    min[4]=-M_PI;  max[4]=M_PI;
    min[5]=-M_PI;  max[5]=M_PI;

    x0=0.5*(min+max);
    Prj.resize(3,4); Prj.zero();
}


/****************************************************************/
double EyeAligner::evalError(const Matrix &H)
{
    Matrix PrjH=Prj*SE3inv(H);

    double error=0.0;
    if (p2d.size()>0)
    {
        for (size_t i=0; i<p2d.size(); i++)
        {
            Vector p2di=PrjH*p3d[i];
            p2di=p2di/p2di[2];
            p2di.pop_back();

            error+=norm(p2d[i]-p2di);
        }

        error/=p2d.size();
    }

    return error;
}


/****************************************************************/
bool EyeAligner::setProjection(const Matrix &Prj)
{
    if ((Prj.rows()>=3) && (Prj.cols()>=4))
    {
        this->Prj=Prj.submatrix(0,2,0,3);
        return true;
    }
    else
        return false;
}


/****************************************************************/
Matrix EyeAligner::getProjection() const
{
    return Prj;
}


/****************************************************************/
void EyeAligner::setBounds(const Vector &min, const Vector &max)
{
    size_t len_min=std::min(this->min.length(),min.length());
    size_t len_max=std::min(this->max.length(),max.length());

    for (size_t i=0; i<len_min; i++)
        this->min[i]=min[i];

    for (size_t i=0; i<len_max; i++)
        this->max[i]=max[i];
}


/****************************************************************/
bool EyeAligner::addPoints(const Vector &p2di, const Vector &p3di)
{
    if ((p2di.length()>=2) && (p3di.length()>=4))
    {
        p2d.push_back(p2di.subVector(0,1));
        p3d.push_back(p3di.subVector(0,3));

        return true;
    }
    else
        return false;
}


/****************************************************************/
void EyeAligner::clearPoints()
{
    p2d.clear();
    p3d.clear();
}


/****************************************************************/
size_t EyeAligner::getNumPoints() const
{
    return p2d.size();
}


/****************************************************************/
bool EyeAligner::setInitialGuess(const Matrix &H)
{
    if ((H.rows()>=4) && (H.cols()>=4))
    {
        Vector rpy=dcm2rpy(H);
        x0[0]=H(0,3); x0[1]=H(1,3); x0[2]=H(2,3);
        x0[3]=rpy[0]; x0[4]=rpy[1]; x0[5]=rpy[2];

        for (size_t i=0; i<min.length(); i++)
            x0[i]=std::min(std::max(x0[i],min[i]),max[i]);

        return true;
    }
    else
        return false;
}


/****************************************************************/
bool EyeAligner::calibrate(Matrix &H, double &error, const int max_iter,
                           const int print_level, const string &derivative_test)
{
    if (p2d.size()>0)
    {
        Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
        app->Options()->SetNumericValue("tol",1e-8);
        app->Options()->SetIntegerValue("acceptable_iter",0);
        app->Options()->SetStringValue("mu_strategy","adaptive");
        app->Options()->SetIntegerValue("max_iter",max_iter);
        app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
        app->Options()->SetStringValue("hessian_approximation","limited-memory");
        app->Options()->SetIntegerValue("print_level",print_level);
        app->Options()->SetStringValue("derivative_test",derivative_test.c_str());
        app->Options()->SetStringValue("derivative_test_print_all","yes");
        app->Initialize();

        Ipopt::SmartPtr<EyeAlignerNLP> nlp=new EyeAlignerNLP(p2d,p3d,min,max,Prj);

        nlp->set_x0(x0);
        Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));

        Vector x=nlp->get_result();
        H=computeH(x);
        error=evalError(H);

        return (status==Ipopt::Solve_Succeeded);
    }
    else
        return false;
}


