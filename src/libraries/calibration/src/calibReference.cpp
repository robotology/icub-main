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

#include <algorithm>

#include <yarp/math/Math.h>
#include <iCub/ctrl/math.h>
#include <iCub/calibration/calibReference.h>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::calibration;


namespace iCub
{

namespace calibration
{

/****************************************************************/
inline Matrix computeH(const Vector &x)
{
    Matrix H=euler2dcm(x.subVector(3,5));
    H(0,3)=x[0]; H(1,3)=x[1]; H(2,3)=x[2];

    return H;
}


/****************************************************************/
inline Matrix computeH(const Ipopt::Number *x)
{
    Vector _x(6);
    for (size_t i=0; i<_x.length(); i++)
        _x[i]=x[i];

    return computeH(_x);
}


/****************************************************************/
class CalibReferenceWithMatchedPointsNLP : public Ipopt::TNLP
{
protected:
    const deque<Vector> &p0;
    const deque<Vector> &p1;
    const Vector        &min;
    const Vector        &max;

    Vector x0;
    Vector x;

public:
    /****************************************************************/
    CalibReferenceWithMatchedPointsNLP(const deque<Vector> &_p0,
                                       const deque<Vector> &_p1,
                                       const Vector &_min, const Vector &_max) :
                                       p0(_p0), p1(_p1), min(_min), max(_max)
    {
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
        Matrix H=computeH(x);

        obj_value=0.0;
        for (size_t i=0; i<p0.size(); i++)
            obj_value+=0.5*norm2(p1[i]-H*p0[i]);

        return true;
    }
    
    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        double ca=cos(x[3]);  double sa=sin(x[3]);
        double cb=cos(x[4]);  double sb=sin(x[4]);
        double cg=cos(x[5]);  double sg=sin(x[5]);

        Matrix Rza=eye(4,4);
        Rza(0,0)=ca;   Rza(1,1)=ca;   Rza(1,0)=sa;   Rza(0,1)=-sa;
        Matrix dRza=zeros(4,4);
        dRza(0,0)=-sa; dRza(1,1)=-sa; dRza(1,0)=ca;  dRza(0,1)=-ca;

        Matrix Rzg=eye(4,4);
        Rzg(0,0)=cg;   Rzg(1,1)=cg;   Rzg(1,0)=sg;   Rzg(0,1)=-sg;
        Matrix dRzg=zeros(4,4);
        dRzg(0,0)=-sg; dRzg(1,1)=-sg; dRzg(1,0)=cg;  dRzg(0,1)=-cg;

        Matrix Ryb=eye(4,4);
        Ryb(0,0)=cb;   Ryb(2,2)=cb;   Ryb(2,0)=-sb;  Ryb(0,2)=sb;
        Matrix dRyb=zeros(4,4);
        dRyb(0,0)=-sb; dRyb(2,2)=-sb; dRyb(2,0)=-cb; dRyb(0,2)=cb;

        Matrix dHdx0=zeros(4,4); dHdx0(0,3)=1.0;
        Matrix dHdx1=zeros(4,4); dHdx1(1,3)=1.0;
        Matrix dHdx2=zeros(4,4); dHdx2(2,3)=1.0;
        Matrix dHdx3=dRza*Ryb*Rzg;
        Matrix dHdx4=Rza*dRyb*Rzg;
        Matrix dHdx5=Rza*Ryb*dRzg;

        Matrix H=computeH(x);
        grad_f[0]=grad_f[1]=grad_f[2]=grad_f[3]=grad_f[4]=grad_f[5]=0.0;
        for (size_t i=0; i<p0.size(); i++)
        {
            Vector d=p1[i]-H*p0[i];
            grad_f[0]-=dot(d,(dHdx0*p0[i]));
            grad_f[1]-=dot(d,(dHdx1*p0[i]));
            grad_f[2]-=dot(d,(dHdx2*p0[i]));
            grad_f[3]-=dot(d,(dHdx3*p0[i]));
            grad_f[4]-=dot(d,(dHdx4*p0[i]));
            grad_f[5]-=dot(d,(dHdx5*p0[i]));
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

}

}


/****************************************************************/
CalibReferenceWithMatchedPoints::CalibReferenceWithMatchedPoints()
{
    min.resize(6); max.resize(6);
    min[0]=-1.0;   max[0]=1.0;
    min[1]=-1.0;   max[1]=1.0;
    min[2]=-1.0;   max[2]=1.0;
    min[3]=-M_PI;  max[3]=M_PI;
    min[4]=-M_PI;  max[4]=M_PI;
    min[5]=-M_PI;  max[5]=M_PI;

    x0=0.5*(min+max);
}


/****************************************************************/
void CalibReferenceWithMatchedPoints::setBounds(const Vector &min,
                                                const Vector &max)
{
    size_t len_min=std::min(this->min.length(),min.length());
    size_t len_max=std::min(this->max.length(),max.length());

    for (size_t i=0; i<len_min; i++)
        this->min[i]=min[i];

    for (size_t i=0; i<len_max; i++)
        this->max[i]=max[i];
}


/****************************************************************/
double CalibReferenceWithMatchedPoints::evalError(const Vector &x)
{
    Matrix H=computeH(x);

    double error=0.0;
    for (size_t i=0; i<p0.size(); i++)
        error+=norm(p1[i]-H*p0[i]);

    return error;
}


/****************************************************************/
bool CalibReferenceWithMatchedPoints::addPoints(const Vector &p0,
                                                const Vector &p1)
{
    if ((p0.length()>=3) && (p1.length()>=3))
    {
        Vector _p0=p0.subVector(0,2); _p0.push_back(1.0);
        Vector _p1=p1.subVector(0,2); _p1.push_back(1.0);

        this->p0.push_back(_p0);
        this->p1.push_back(_p1);

        return true;
    }
    else
        return false;
}


/****************************************************************/
void CalibReferenceWithMatchedPoints::clearPoints()
{
    p0.clear();
    p1.clear();
}


/****************************************************************/
bool CalibReferenceWithMatchedPoints::setInitialGuess(const Matrix &H)
{
    if ((H.rows()>=4) && (H.cols()>=4))
    {        
        Vector euler=dcm2euler(H.submatrix(0,2,0,2));
        x0[0]=H(0,3);   x0[1]=H(1,3);   x0[2]=H(2,3);
        x0[3]=euler[0]; x0[4]=euler[1]; x0[5]=euler[2];

        return true;
    }
    else
        return false;
}


/****************************************************************/
bool CalibReferenceWithMatchedPoints::calibrate(Matrix &H, double &error)
{
    if (p0.size()>0)
    {
        Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
        app->Options()->SetNumericValue("tol",1e-8);
        app->Options()->SetNumericValue("acceptable_tol",1e-8);
        app->Options()->SetIntegerValue("acceptable_iter",10);
        app->Options()->SetStringValue("mu_strategy","adaptive");
        app->Options()->SetIntegerValue("max_iter",300);
        app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
        app->Options()->SetStringValue("hessian_approximation","limited-memory");
        app->Options()->SetIntegerValue("print_level",0);
        app->Options()->SetStringValue("derivative_test","none");
        app->Initialize();

        Ipopt::SmartPtr<CalibReferenceWithMatchedPointsNLP> nlp=new CalibReferenceWithMatchedPointsNLP(p0,p1,min,max);

        nlp->set_x0(x0);
        Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));

        Vector x=nlp->get_result();
        H=computeH(x);
        error=evalError(x);

        return (status==Ipopt::Solve_Succeeded);
    }
    else
        return false;
}



