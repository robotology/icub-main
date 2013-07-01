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
#include <yarp/math/Rand.h>
#include <iCub/ctrl/math.h>
#include <iCub/optimization/neuralNetworks.h>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

#define CAST_IPOPTAPP(x)        (static_cast<Ipopt::IpoptApplication*>(x))

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
class ff2LayNNTrainNLP : public Ipopt::TNLP
{
protected:
    Property bounds;
    bool randomInit;

    deque<Vector> &IW;
    deque<Vector> &LW;
    Vector        &b1;
    Vector        &b2;

public:
    /****************************************************************/
    ff2LayNNTrainNLP(ff2LayNNTrain *net, const Property &bounds,
                     const bool _randomInit, const deque<Vector> &in,
                     const deque<Vector> &out) : randomInit(_randomInit),
                     IW(net->get_IW()), LW(net->get_LW()),
                     b1(net->get_b1()), b2(net->get_b2())
    {
        this->bounds=bounds;
    }

    /****************************************************************/
    virtual double get_prediction(deque<Vector> &pred) const
    {
        return 0.0;
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
//      Ipopt::Index i=0;
//      for (int c=0; c<A0.cols(); c++)
//      {
//          for (int r=0; r<A0.rows()-1; r++)
//          {
//              x_l[i]=min(r,c);
//              x_u[i]=max(r,c);
//              i++;
//          }
//      }

        return true;
    }

    /****************************************************************/
    bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                            bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
    {
//      Ipopt::Index i=0;
//      for (int c=0; c<A0.cols(); c++)
//      {
//          for (int r=0; r<A0.rows()-1; r++)
//              x[i++]=A0(r,c);
//      }

        return true;
    }

    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {
//      Matrix A=computeA(x);
//
//      obj_value=0.0;
//      if (p0.size()>0)
//      {
//          for (size_t i=0; i<p0.size(); i++)
//              obj_value+=0.5*norm2(p1[i]-A*p0[i]);
//
//          obj_value/=p0.size();
//      }

        return true;
    }

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
//      Matrix A=computeA(x);
//      for (Ipopt::Index i=0; i<n; i++)
//          grad_f[i]=0.0;
//
//      if (p0.size()>0)
//      {
//          for (size_t i=0; i<p0.size(); i++)
//          {
//              Vector d=p1[i]-A*p0[i];
//              for (Ipopt::Index j=0; j<n; j++)
//                  grad_f[j]-=dot(d,(dA[j]*p0[i]));
//          }
//
//          for (Ipopt::Index i=0; i<n; i++)
//              grad_f[i]/=p0.size();
//      }

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
        //A=computeA(x);
    }
};

}

}


/****************************************************************/
ff2LayNNTrain::ff2LayNNTrain()
{
    App=new Ipopt::IpoptApplication();
    CAST_IPOPTAPP(App)->Options()->SetNumericValue("tol",1e-8);
    CAST_IPOPTAPP(App)->Options()->SetNumericValue("acceptable_tol",1e-8);
    CAST_IPOPTAPP(App)->Options()->SetIntegerValue("acceptable_iter",10);
    CAST_IPOPTAPP(App)->Options()->SetStringValue("mu_strategy","adaptive");
    CAST_IPOPTAPP(App)->Options()->SetIntegerValue("max_iter",300);
    CAST_IPOPTAPP(App)->Options()->SetStringValue("nlp_scaling_method","gradient-based");
    CAST_IPOPTAPP(App)->Options()->SetStringValue("hessian_approximation","limited-memory");
    CAST_IPOPTAPP(App)->Options()->SetIntegerValue("print_level",0);
    CAST_IPOPTAPP(App)->Options()->SetStringValue("derivative_test","none");
    CAST_IPOPTAPP(App)->Initialize();
}


/****************************************************************/
void ff2LayNNTrain::setBounds(const Property &bounds)
{
    this->bounds=bounds;
}


/****************************************************************/
bool ff2LayNNTrain::train(const unsigned int numHiddenNodes,
                          const deque<Vector> &in, const deque<Vector> &out,
                          deque<Vector> &pred, double &error)
{
    if ((in.size()==0) || (in.size()!=out.size()) || (in.size()!=pred.size()))
        return false;

    IW.clear();
    LW.clear();

    inMinMaxX.clear();
    inMinMaxY.clear();

    outMinMaxX.clear();
    outMinMaxY.clear();

    // seek for min-max of input and scale it in [-1,1]
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

    // seek for min-max of output and scale it in [-1,1]
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
    configured=true;

    Ipopt::SmartPtr<ff2LayNNTrainNLP> nlp=new ff2LayNNTrainNLP(this,bounds,true,in,out);
    Ipopt::ApplicationReturnStatus status=CAST_IPOPTAPP(App)->OptimizeTNLP(GetRawPtr(nlp));    

    error=nlp->get_prediction(pred);
    return (status==Ipopt::Solve_Succeeded);    
}


/****************************************************************/
bool ff2LayNNTrain::retrain(const deque<Vector> &in, const deque<Vector> &out,
                            deque<Vector> &pred, double &error)
{
    if ((in.size()==0) || (in.size()!=out.size()) || (in.size()!=pred.size()) || !configured)
        return false;

    Ipopt::SmartPtr<ff2LayNNTrainNLP> nlp=new ff2LayNNTrainNLP(this,bounds,false,in,out);
    Ipopt::ApplicationReturnStatus status=CAST_IPOPTAPP(App)->OptimizeTNLP(GetRawPtr(nlp));    

    error=nlp->get_prediction(pred);
    return (status==Ipopt::Solve_Succeeded);    
}


/****************************************************************/
ff2LayNNTrain::~ff2LayNNTrain()
{
    delete CAST_IPOPTAPP(App);
}

