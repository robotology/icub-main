/*
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
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
#include <iCub/optimization/calibReference.h>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::optimization;


namespace iCub
{

namespace optimization
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

    Vector min;
    Vector max;
    Vector x0;
    Vector x;

public:
    /****************************************************************/
    CalibReferenceWithMatchedPointsNLP(const deque<Vector> &_p0,
                                       const deque<Vector> &_p1,
                                       const Vector &_min, const Vector &_max) :
                                       p0(_p0), p1(_p1)
    {
        min=_min;
        max=_max;
        x0=0.5*(min+max);
    }

    /****************************************************************/
    virtual void set_x0(const Vector &x0)
    {
        size_t len=std::min(this->x0.length(),x0.length());
        for (size_t i=0; i<len; i++)
            this->x0[i]=x0[i];
    }

    /****************************************************************/
    virtual Vector get_result() const
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
        if (p0.size()>0)
        {
            for (size_t i=0; i<p0.size(); i++)
                obj_value+=norm2(p1[i]-H*p0[i]);

            obj_value/=p0.size();
        }

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
        grad_f[0]=grad_f[1]=grad_f[2]=0.0;
        grad_f[3]=grad_f[4]=grad_f[5]=0.0;
        if (p0.size()>0)
        {
            for (size_t i=0; i<p0.size(); i++)
            {
                Vector d=p1[i]-H*p0[i];
                grad_f[0]-=2.0*dot(d,(dHdx0*p0[i]));
                grad_f[1]-=2.0*dot(d,(dHdx1*p0[i]));
                grad_f[2]-=2.0*dot(d,(dHdx2*p0[i]));
                grad_f[3]-=2.0*dot(d,(dHdx3*p0[i]));
                grad_f[4]-=2.0*dot(d,(dHdx4*p0[i]));
                grad_f[5]-=2.0*dot(d,(dHdx5*p0[i]));
            }

            for (Ipopt::Index i=0; i<n; i++)
                grad_f[i]/=p0.size();
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
class CalibReferenceWithScaledMatchedPointsNLP : public CalibReferenceWithMatchedPointsNLP
{
public:
    /****************************************************************/
    CalibReferenceWithScaledMatchedPointsNLP(const deque<Vector> &_p0,
                                             const deque<Vector> &_p1,
                                             const Vector &_min, const Vector &_max) :
                                             CalibReferenceWithMatchedPointsNLP(_p0,_p1,_min,_max) { }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        CalibReferenceWithMatchedPointsNLP::get_nlp_info(n,m,nnz_jac_g,nnz_h_lag,index_style);
        n=6+3;

        return true;
    }

    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {
        Matrix H=computeH(x);
        Vector s(4);
        s[0]=x[6];
        s[1]=x[7];
        s[2]=x[8];
        s[3]=1.0;

        obj_value=0.0;
        if (p0.size()>0)
        {
            for (size_t i=0; i<p0.size(); i++)
                obj_value+=norm2(p1[i]-s*(H*p0[i]));

            obj_value/=p0.size();
        }

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
        Matrix dHdx6=zeros(4,4); dHdx6(0,0)=1.0; dHdx6*=H;
        Matrix dHdx7=zeros(4,4); dHdx7(1,1)=1.0; dHdx7*=H;
        Matrix dHdx8=zeros(4,4); dHdx8(2,2)=1.0; dHdx8*=H;

        Vector s(4);
        s[0]=x[6];
        s[1]=x[7];
        s[2]=x[8];
        s[3]=1.0;

        grad_f[0]=grad_f[1]=grad_f[2]=0.0;
        grad_f[3]=grad_f[4]=grad_f[5]=0.0;
        grad_f[6]=grad_f[7]=grad_f[8]=0.0;
        if (p0.size()>0)
        {
            for (size_t i=0; i<p0.size(); i++)
            {
                Vector d=p1[i]-s*(H*p0[i]);
                grad_f[0]-=2.0*dot(d,(dHdx0*p0[i]));
                grad_f[1]-=2.0*dot(d,(dHdx1*p0[i]));
                grad_f[2]-=2.0*dot(d,(dHdx2*p0[i]));
                grad_f[3]-=2.0*dot(d,(dHdx3*p0[i]));
                grad_f[4]-=2.0*dot(d,(dHdx4*p0[i]));
                grad_f[5]-=2.0*dot(d,(dHdx5*p0[i]));
                grad_f[6]-=2.0*dot(d,(dHdx6*p0[i]));
                grad_f[7]-=2.0*dot(d,(dHdx7*p0[i]));
                grad_f[8]-=2.0*dot(d,(dHdx8*p0[i]));
            }

            for (Ipopt::Index i=0; i<n; i++)
                grad_f[i]/=p0.size();
        }

        return true;
    }
};


/****************************************************************/
class CalibReferenceWithScalarScaledMatchedPointsNLP : public CalibReferenceWithMatchedPointsNLP
{
public:
    /****************************************************************/
    CalibReferenceWithScalarScaledMatchedPointsNLP(const deque<Vector> &_p0,
                                                   const deque<Vector> &_p1,
                                                   const Vector &_min, const Vector &_max) :
                                                   CalibReferenceWithMatchedPointsNLP(_p0,_p1,_min,_max) { }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        CalibReferenceWithMatchedPointsNLP::get_nlp_info(n,m,nnz_jac_g,nnz_h_lag,index_style);
        n=6+1;

        return true;
    }

    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {
        Matrix H=computeH(x);
        Vector s(4);
        s[0]=s[1]=s[2]=x[6];
        s[3]=1.0;

        obj_value=0.0;
        if (p0.size()>0)
        {
            for (size_t i=0; i<p0.size(); i++)
                obj_value+=norm2(p1[i]-s*(H*p0[i]));

            obj_value/=p0.size();
        }

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
        Matrix dHdx6=zeros(4,4); dHdx6(0,0)=1.0; dHdx6*=H;

        Vector s(4);
        s[0]=s[1]=s[2]=x[6];
        s[3]=1.0;

        grad_f[0]=grad_f[1]=grad_f[2]=0.0;
        grad_f[3]=grad_f[4]=grad_f[5]=0.0;
        grad_f[6]=0.0;
        if (p0.size()>0)
        {
            for (size_t i=0; i<p0.size(); i++)
            {
                Vector d=p1[i]-s*(H*p0[i]);
                grad_f[0]-=2.0*dot(d,(dHdx0*p0[i]));
                grad_f[1]-=2.0*dot(d,(dHdx1*p0[i]));
                grad_f[2]-=2.0*dot(d,(dHdx2*p0[i]));
                grad_f[3]-=2.0*dot(d,(dHdx3*p0[i]));
                grad_f[4]-=2.0*dot(d,(dHdx4*p0[i]));
                grad_f[5]-=2.0*dot(d,(dHdx5*p0[i]));
                grad_f[6]-=2.0*dot(d,(dHdx6*p0[i]));
            }

            for (Ipopt::Index i=0; i<n; i++)
                grad_f[i]/=p0.size();
        }

        return true;
    }
};

}

}


/****************************************************************/
CalibReferenceWithMatchedPoints::CalibReferenceWithMatchedPoints()
{
    max_iter=300;
    tol=1e-8;

    min.resize(6); max.resize(6);
    min[0]=-1.0;   max[0]=1.0;
    min[1]=-1.0;   max[1]=1.0;
    min[2]=-1.0;   max[2]=1.0;
    min[3]=-M_PI;  max[3]=M_PI;
    min[4]=-M_PI;  max[4]=M_PI;
    min[5]=-M_PI;  max[5]=M_PI;

    min_s.resize(3); max_s.resize(3);
    min_s[0]=0.1;    max_s[0]=10.0;
    min_s[1]=0.1;    max_s[1]=10.0;
    min_s[2]=0.1;    max_s[2]=10.0;

    min_s_scalar=0.1; max_s_scalar=10.0;

    x0=0.5*(min+max);
    s0.resize(3,1.0);
    s0_scalar=1.0;
}


/****************************************************************/
void CalibReferenceWithMatchedPoints::setBounds(const Matrix &min,
                                                const Matrix &max)
{
    if ((min.rows()<3) || (min.cols()<3) ||
        (max.rows()<3) || (max.cols()<3))
        return;

    this->min[0]=min(0,3); this->max[0]=max(0,3); 
    this->min[1]=min(1,3); this->max[1]=max(1,3);
    this->min[2]=min(2,3); this->max[2]=max(2,3);

    this->min[3]=min(0,0); this->max[3]=max(0,0);
    this->min[4]=min(0,1); this->max[4]=max(0,1);
    this->min[5]=min(0,2); this->max[5]=max(0,2);
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
void CalibReferenceWithMatchedPoints::setScalingBounds(const Vector &min,
                                                       const Vector &max)
{
    size_t len_min=std::min(this->min_s.length(),min.length());
    size_t len_max=std::min(this->max_s.length(),max.length());

    for (size_t i=0; i<len_min; i++)
        this->min_s[i]=min[i];

    for (size_t i=0; i<len_max; i++)
        this->max_s[i]=max[i];
}


/****************************************************************/
void CalibReferenceWithMatchedPoints::setScalingBounds(const double min,
                                                       const double max)
{
    min_s_scalar=min;
    max_s_scalar=max;
}


/****************************************************************/
double CalibReferenceWithMatchedPoints::evalError(const Matrix &H)
{
    double error=0.0;
    if (p0.size()>0)
    {
        for (size_t i=0; i<p0.size(); i++)
            error+=norm(p1[i]-H*p0[i]);

        error/=p0.size();
    }

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
void CalibReferenceWithMatchedPoints::getPoints(deque<Vector> &p0,
                                                deque<Vector> &p1) const
{
    p0=this->p0;
    p1=this->p1;
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
        Vector euler=dcm2euler(H);
        x0[0]=H(0,3);   x0[1]=H(1,3);   x0[2]=H(2,3);
        x0[3]=euler[0]; x0[4]=euler[1]; x0[5]=euler[2];

        return true;
    }
    else
        return false;
}


/****************************************************************/
bool CalibReferenceWithMatchedPoints::setScalingInitialGuess(const Vector &s)
{
    if (s.length()>=s0.length())
    {
        s0=s.subVector(0,s0.length()-1);
        return true;
    }
    else
        return false;
}


/****************************************************************/
bool CalibReferenceWithMatchedPoints::setScalingInitialGuess(const double s)
{
    s0_scalar=s;
    return true;
}


/****************************************************************/
bool CalibReferenceWithMatchedPoints::setCalibrationOptions(const Property &options)
{
    if (options.check("max_iter"))
        max_iter=options.find("max_iter").asInt();

    if (options.check("tol"))
        tol=options.find("tol").asDouble();

    return true;
}


/****************************************************************/
bool CalibReferenceWithMatchedPoints::calibrate(Matrix &H, double &error)
{
    if (p0.size()>0)
    {
        Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
        app->Options()->SetNumericValue("tol",tol);
        app->Options()->SetNumericValue("acceptable_tol",tol);
        app->Options()->SetIntegerValue("acceptable_iter",10);
        app->Options()->SetStringValue("mu_strategy","adaptive");
        app->Options()->SetIntegerValue("max_iter",max_iter);
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
        error=evalError(H);

        return (status==Ipopt::Solve_Succeeded);
    }
    else
        return false;
}


/****************************************************************/
bool CalibReferenceWithMatchedPoints::calibrate(Matrix &H, Vector &s,
                                                double &error)
{
    if (p0.size()>0)
    {
        Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
        app->Options()->SetNumericValue("tol",tol);
        app->Options()->SetNumericValue("acceptable_tol",tol);
        app->Options()->SetIntegerValue("acceptable_iter",10);
        app->Options()->SetStringValue("mu_strategy","adaptive");
        app->Options()->SetIntegerValue("max_iter",max_iter);
        app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
        app->Options()->SetStringValue("hessian_approximation","limited-memory");
        app->Options()->SetIntegerValue("print_level",0);
        app->Options()->SetStringValue("derivative_test","none");
        app->Initialize();

        Ipopt::SmartPtr<CalibReferenceWithScaledMatchedPointsNLP> nlp=new CalibReferenceWithScaledMatchedPointsNLP(p0,p1,cat(min,min_s),cat(max,max_s));

        nlp->set_x0(cat(x0,s0));
        Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));

        Vector x=nlp->get_result();
        H=computeH(x);
        s=x.subVector(6,8);
        Matrix S=eye(4,4);
        S(0,0)=s[0]; S(1,1)=s[1]; S(2,2)=s[2];
        error=evalError(S*H);

        return (status==Ipopt::Solve_Succeeded);
    }
    else
        return false;
}


/****************************************************************/
bool CalibReferenceWithMatchedPoints::calibrate(Matrix &H, double &s,
                                                double &error)
{
    if (p0.size()>0)
    {
        Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
        app->Options()->SetNumericValue("tol",tol);
        app->Options()->SetNumericValue("acceptable_tol",tol);
        app->Options()->SetIntegerValue("acceptable_iter",10);
        app->Options()->SetStringValue("mu_strategy","adaptive");
        app->Options()->SetIntegerValue("max_iter",max_iter);
        app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
        app->Options()->SetStringValue("hessian_approximation","limited-memory");
        app->Options()->SetIntegerValue("print_level",0);
        app->Options()->SetStringValue("derivative_test","none");
        app->Initialize();

        Ipopt::SmartPtr<CalibReferenceWithScalarScaledMatchedPointsNLP> nlp=new CalibReferenceWithScalarScaledMatchedPointsNLP(p0,p1,cat(min,min_s_scalar),cat(max,max_s_scalar));

        nlp->set_x0(cat(x0,s0_scalar));
        Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));

        Vector x=nlp->get_result();
        H=computeH(x);
        s=x[6];
        Matrix S=eye(4,4);
        S(0,0)=S(1,1)=S(2,2)=s;
        error=evalError(S*H);

        return (status==Ipopt::Solve_Succeeded);
    }
    else
        return false;
}


