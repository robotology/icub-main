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
#include <iCub/optimization/neuralNetworks.h>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::optimization;


//namespace iCub
//{
//
//namespace optimization
//{
//
///****************************************************************/
//class ff2LayNNTrainNLP : public Ipopt::TNLP
//{
//protected:
//    const deque<Vector> &p0;
//    const deque<Vector> &p1;
//
//    deque<Matrix> dA;
//    Matrix min;
//    Matrix max;
//    Matrix A0;
//    Matrix A;
//
//public:
//    /****************************************************************/
//    ff2LayNNTrainNLP(const deque<Vector> &_p0,
//                    const deque<Vector> &_p1,
//                    const Matrix &_min, const Matrix &_max) :
//                    p0(_p0), p1(_p1)
//    {
//        min=_min;
//        max=_max;
//        A0=0.5*(min+max);
//
//        for (int c=0; c<A0.cols(); c++)
//        {
//            for (int r=0; r<A0.rows()-1; r++)
//            {
//                Matrix dA=zeros(4,4); dA(r,c)=1.0;
//                this->dA.push_back(dA);
//            }
//        }
//    }
//
//    /****************************************************************/
//    virtual void set_A0(const Matrix &A0)
//    {
//        int row_max=std::min(this->A0.rows()-1,A0.rows()-1);
//        int col_max=std::min(this->A0.cols(),A0.cols());
//        this->A0.setSubmatrix(A0.submatrix(0,row_max,0,col_max),0,0);
//    }
//
//    /****************************************************************/
//    virtual Matrix get_result() const
//    {
//        return A;
//    }
//
//    /****************************************************************/
//    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
//                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
//    {
//        n=12;
//        m=nnz_jac_g=nnz_h_lag=0;
//        index_style=TNLP::C_STYLE;
//
//        return true;
//    }
//
//    /****************************************************************/
//    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
//                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
//    {
//        Ipopt::Index i=0;
//        for (int c=0; c<A0.cols(); c++)
//        {
//            for (int r=0; r<A0.rows()-1; r++)
//            {
//                x_l[i]=min(r,c);
//                x_u[i]=max(r,c);
//                i++;
//            }
//        }
//
//        return true;
//    }
//
//    /****************************************************************/
//    bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
//                            bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
//                            Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
//    {
//        Ipopt::Index i=0;
//        for (int c=0; c<A0.cols(); c++)
//        {
//            for (int r=0; r<A0.rows()-1; r++)
//                x[i++]=A0(r,c);
//        }
//
//        return true;
//    }
//
//    /****************************************************************/
//    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
//                Ipopt::Number &obj_value)
//    {
//        Matrix A=computeA(x);
//
//        obj_value=0.0;
//        if (p0.size()>0)
//        {
//            for (size_t i=0; i<p0.size(); i++)
//                obj_value+=0.5*norm2(p1[i]-A*p0[i]);
//
//            obj_value/=p0.size();
//        }
//
//        return true;
//    }
//
//    /****************************************************************/
//    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
//                     Ipopt::Number *grad_f)
//    {
//        Matrix A=computeA(x);
//        for (Ipopt::Index i=0; i<n; i++)
//            grad_f[i]=0.0;
//
//        if (p0.size()>0)
//        {
//            for (size_t i=0; i<p0.size(); i++)
//            {
//                Vector d=p1[i]-A*p0[i];
//                for (Ipopt::Index j=0; j<n; j++)
//                    grad_f[j]-=dot(d,(dA[j]*p0[i]));
//            }
//
//            for (Ipopt::Index i=0; i<n; i++)
//                grad_f[i]/=p0.size();
//        }
//
//        return true;
//    }
//
//    /****************************************************************/
//    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
//                Ipopt::Index m, Ipopt::Number *g)
//    {
//        return true;
//    }
//
//    /****************************************************************/
//    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
//                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
//                    Ipopt::Index *jCol, Ipopt::Number *values)
//    {
//        return true;
//    }
//
//    /****************************************************************/
//    bool eval_h(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
//                Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number *lambda,
//                bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index *iRow,
//                Ipopt::Index *jCol, Ipopt::Number *values)
//    {
//        return true;
//    }
//
//    /****************************************************************/
//    void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
//                           const Ipopt::Number *x, const Ipopt::Number *z_L,
//                           const Ipopt::Number *z_U, Ipopt::Index m,
//                           const Ipopt::Number *g, const Ipopt::Number *lambda,
//                           Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
//                           Ipopt::IpoptCalculatedQuantities *ip_cq)
//    {
//        A=computeA(x);
//    }
//};
//
//}
//
//}


/****************************************************************/
bool ff2LayNNTrain::train(const unsigned int numHiddenNodes,
                          const deque<Vector> &in, const deque<Vector> &out,
                          deque<Vector> &pred, double &error)
{
    if ((in.size()==0) || (out.size()==0) || (pred.size()==0))
        return false;

    const Vector &in_front=in.front();
    for (size_t i=0; i<in_front.length(); i++)
    {
        minmax range;
        range.min=range.max=in_front[i];
        inMinMaxX.push_back(range);
        range.min=-1.0; range.max=1.0;
        inMinMaxY.push_back(range);
    }
    for (size_t i=1; i<in.size(); i++)
    {
        for (size_t j=0; j<in_front.length(); j++)
        {
            inMinMaxX[j].min=std::min(inMinMaxX[j].min,in[i][j]);
            inMinMaxX[j].max=std::max(inMinMaxX[j].max,in[i][j]);
        }
    }

    const Vector &out_front=out.front();
    for (size_t i=0; i<out_front.length(); i++)
    {
        minmax range;
        range.min=range.max=out_front[i];
        outMinMaxY.push_back(range);
        range.min=-1.0; range.max=1.0;
        outMinMaxX.push_back(range);
    }
    for (size_t i=1; i<out.size(); i++)
    {
        for (size_t j=0; j<out_front.length(); j++)
        {
            outMinMaxX[j].min=std::min(outMinMaxX[j].min,out[i][j]);
            outMinMaxX[j].max=std::max(outMinMaxX[j].max,out[i][j]);
        }
    }

    prepare();

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

    //Ipopt::SmartPtr<ff2LayNNTrainNLP> nlp=new ff2LayNNTrainNLP(p0,p1,min,max);
    //Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));
    //A=nlp->get_result();
    //return (status==Ipopt::Solve_Succeeded);

    return true; // debug
}


