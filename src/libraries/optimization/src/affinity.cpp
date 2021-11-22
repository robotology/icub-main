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
#include <iCub/optimization/affinity.h>

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
inline Matrix computeA(const Ipopt::Number *x)
{
    Matrix A=eye(4,4);
    A(0,0)=x[0]; A(0,1)=x[3]; A(0,2)=x[6]; A(0,3)=x[9];
    A(1,0)=x[1]; A(1,1)=x[4]; A(1,2)=x[7]; A(1,3)=x[10];
    A(2,0)=x[2]; A(2,1)=x[5]; A(2,2)=x[8]; A(2,3)=x[11];

    return A;
}


/****************************************************************/
class AffinityWithMatchedPointsNLP : public Ipopt::TNLP
{
protected:    
    const deque<Vector> &p0;
    const deque<Vector> &p1;

    deque<Matrix> dA;
    Matrix min;
    Matrix max;
    Matrix A0;
    Matrix A;

public:
    /****************************************************************/
    AffinityWithMatchedPointsNLP(const deque<Vector> &_p0,
                                 const deque<Vector> &_p1,
                                 const Matrix &_min, const Matrix &_max) :
                                 p0(_p0), p1(_p1)
    {
        min=_min;
        max=_max;
        A0=0.5*(min+max);
        
        for (int c=0; c<A0.cols(); c++)
        {
            for (int r=0; r<A0.rows()-1; r++)
            {
                Matrix dA=zeros(4,4); dA(r,c)=1.0;
                this->dA.push_back(dA);
            }
        }
    }

    /****************************************************************/
    virtual void set_A0(const Matrix &A0)
    {
        int row_max=(int)std::min(this->A0.rows()-1,A0.rows()-1);
        int col_max=(int)std::min(this->A0.cols(),A0.cols());
        this->A0.setSubmatrix(A0.submatrix(0,row_max,0,col_max),0,0);
    }

    /****************************************************************/
    virtual Matrix get_result() const
    {
        return A;
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        n=12;
        m=nnz_jac_g=nnz_h_lag=0;
        index_style=TNLP::C_STYLE;

        return true;
    }

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
    {
        Ipopt::Index i=0;
        for (int c=0; c<A0.cols(); c++)
        {
            for (int r=0; r<A0.rows()-1; r++)
            {
                x_l[i]=min(r,c);
                x_u[i]=max(r,c);
                i++;
            }
        }

        return true;
    }
    
    /****************************************************************/
    bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                            bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
    {
        Ipopt::Index i=0;
        for (int c=0; c<A0.cols(); c++)
        {
            for (int r=0; r<A0.rows()-1; r++)
                x[i++]=A0(r,c);
        }

        return true;
    }
    
    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {
        Matrix A=computeA(x);

        obj_value=0.0;
        if (p0.size()>0)
        {
            for (size_t i=0; i<p0.size(); i++)
                obj_value+=norm2(p1[i]-A*p0[i]);

            obj_value/=p0.size();
        }

        return true;
    }
    
    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        Matrix A=computeA(x);
        for (Ipopt::Index i=0; i<n; i++)
            grad_f[i]=0.0;

        if (p0.size()>0)
        {
            for (size_t i=0; i<p0.size(); i++)
            {
                Vector d=p1[i]-A*p0[i];
                for (Ipopt::Index j=0; j<n; j++)
                    grad_f[j]-=2.0*dot(d,(dA[j]*p0[i]));
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
        A=computeA(x);
    }
};

}

}


/****************************************************************/
AffinityWithMatchedPoints::AffinityWithMatchedPoints()
{
    max_iter=300;
    tol=1e-8;

    min=max=eye(4,4);
    for (int c=0; c<min.cols(); c++)
    {
        for (int r=0; r<min.rows()-1; r++)
        {
            min(r,c)=-1.0;
            max(r,c)=+1.0;
        }
    }

    A0=0.5*(min+max);
}


/****************************************************************/
void AffinityWithMatchedPoints::setBounds(const Matrix &min,
                                          const Matrix &max)
{
    int row_max,col_max;

    row_max=(int)std::min(this->min.rows()-1,min.rows()-1);
    col_max=(int)std::min(this->min.cols(),min.cols());
    this->min.setSubmatrix(min.submatrix(0,row_max,0,col_max),0,0);

    row_max=(int)std::min(this->max.rows()-1,max.rows()-1);
    col_max=(int)std::min(this->max.cols(),max.cols());
    this->max.setSubmatrix(max.submatrix(0,row_max,0,col_max),0,0);
}


/****************************************************************/
double AffinityWithMatchedPoints::evalError(const Matrix &A)
{
    double error=0.0;
    if (p0.size()>0)
    {
        for (size_t i=0; i<p0.size(); i++)
            error+=norm(p1[i]-A*p0[i]);

        error/=p0.size();
    }

    return error;
}


/****************************************************************/
bool AffinityWithMatchedPoints::addPoints(const Vector &p0,
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
void AffinityWithMatchedPoints::getPoints(deque<Vector> &p0,
                                          deque<Vector> &p1) const
{
    p0=this->p0;
    p1=this->p1;
}


/****************************************************************/
void AffinityWithMatchedPoints::clearPoints()
{
    p0.clear();
    p1.clear();
}


/****************************************************************/
bool AffinityWithMatchedPoints::setInitialGuess(const Matrix &A)
{
    int row_max=(int)std::min(A0.rows()-1,A.rows()-1);
    int col_max=(int)std::min(A0.cols(),A.cols());
    A0.setSubmatrix(A.submatrix(0,row_max,0,col_max),0,0);

    return true;
}


/****************************************************************/
bool AffinityWithMatchedPoints::setCalibrationOptions(const Property &options)
{
    if (options.check("max_iter"))
        max_iter=options.find("max_iter").asInt32();

    if (options.check("tol"))
        tol=options.find("tol").asFloat64();

    return true;
}


/****************************************************************/
bool AffinityWithMatchedPoints::calibrate(Matrix &A, double &error)
{
    if (p0.size()>0)
    {
        Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
        app->Options()->SetNumericValue("tol",tol);
        app->Options()->SetIntegerValue("acceptable_iter",0);
        app->Options()->SetStringValue("mu_strategy","adaptive");
        app->Options()->SetIntegerValue("max_iter",max_iter);
        app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
        app->Options()->SetStringValue("jac_c_constant","yes");
        app->Options()->SetStringValue("jac_d_constant","yes");
        app->Options()->SetStringValue("hessian_constant","yes");
        app->Options()->SetStringValue("hessian_approximation","limited-memory");
        app->Options()->SetIntegerValue("print_level",0);
        app->Options()->SetStringValue("derivative_test","none");
        app->Initialize();

        Ipopt::SmartPtr<AffinityWithMatchedPointsNLP> nlp=new AffinityWithMatchedPointsNLP(p0,p1,min,max);

        nlp->set_A0(A0);
        Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));

        A=nlp->get_result();
        error=evalError(A);

        return (status==Ipopt::Solve_Succeeded);
    }
    else
        return false;
}


