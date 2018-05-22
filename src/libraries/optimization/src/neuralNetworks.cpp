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
#include <string>

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

    ff2LayNNTrain &net;
    deque<Vector> &IW;
    deque<Vector> &LW;
    Vector        &b1;
    Vector        &b2;

    const deque<Vector> &in;
    const deque<Vector> &out;
    deque<Vector> &pred;
    double error;

    /****************************************************************/
    bool getBounds(const string &tag, double &min, double &max)
    {
        min=-1.0; max=1.0;
        if (Bottle *b=bounds.find(tag).asList())
        {
            if (b->size()>=2)
            {
                min=b->get(0).asDouble();
                max=b->get(1).asDouble();
                return true;
            }
        }

        return false;
    }

    /****************************************************************/
    void fillNet(const Ipopt::Number *x)
    {
        Ipopt::Index k=0;
        for (size_t i=0; i<IW.size(); i++)
            for (size_t j=0; j<IW.front().length(); j++, k++)
                IW[i][j]=x[k];

        for (size_t i=0; i<LW.size(); i++)
            for (size_t j=0; j<LW.front().length(); j++, k++)
                LW[i][j]=x[k];

        for (size_t i=0; i<b1.length(); i++, k++)
            b1[i]=x[k];

        for (size_t i=0; i<b2.length(); i++, k++)
            b2[i]=x[k];
    }

    /****************************************************************/
    void fillVar(Ipopt::Number *x, const Ipopt::Number *x_l, const Ipopt::Number *x_u)
    {
        Ipopt::Index k=0;
        for (size_t i=0; i<IW.size(); i++)
            for (size_t j=0; j<IW.front().length(); j++, k++)
                x[k]=std::min(x_u[k],std::max(IW[i][j],x_l[k]));

        for (size_t i=0; i<LW.size(); i++)
            for (size_t j=0; j<LW.front().length(); j++, k++)
                x[k]=std::min(x_u[k],std::max(LW[i][j],x_l[k]));

        for (size_t i=0; i<b1.length(); i++, k++)
            x[k]=std::min(x_u[k],std::max(b1[i],x_l[k]));

        for (size_t i=0; i<b2.length(); i++, k++)
            x[k]=std::min(x_u[k],std::max(b2[i],x_l[k]));
    }

public:
    /****************************************************************/
    ff2LayNNTrainNLP(ff2LayNNTrain &_net, const Property &_bounds,
                     const bool _randomInit, const deque<Vector> &_in,
                     const deque<Vector> &_out, deque<Vector> &_pred) :
                     net(_net), bounds(_bounds), randomInit(_randomInit),
                     in(_in), out(_out), pred(_pred),
                     IW(_net.get_IW()), LW(_net.get_LW()),
                     b1(_net.get_b1()), b2(_net.get_b2())
    {
        pred.clear();
        error=0.0;        
    }

    /****************************************************************/
    virtual double get_error() const
    {
        return error;
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        n=IW.size()*IW.front().length()+LW.size()*LW.front().length()+b1.length()+b2.length();
        m=nnz_jac_g=nnz_h_lag=0;
        index_style=TNLP::C_STYLE;

        return true;
    }

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
    {
        double min_IW,max_IW; getBounds("IW",min_IW,max_IW);
        double min_LW,max_LW; getBounds("LW",min_LW,max_LW);
        double min_b1,max_b1; getBounds("b1",min_b1,max_b1);
        double min_b2,max_b2; getBounds("b2",min_b2,max_b2);

        Ipopt::Index k=0;
        for (size_t i=0; i<IW.size(); i++)
        {
            for (size_t j=0; j<IW.front().length(); j++, k++)
            {
                x_l[k]=min_IW;
                x_u[k]=max_IW;
            }
        }
            
        for (size_t i=0; i<LW.size(); i++)
        {
            for (size_t j=0; j<LW.front().length(); j++, k++)
            {
                x_l[k]=min_LW;
                x_u[k]=max_LW;
            }
        }

        for (size_t i=0; i<b1.size(); i++, k++)
        {
            x_l[k]=min_b1;
            x_u[k]=max_b1;
        }

        for (size_t i=0; i<b2.size(); i++, k++)
        {
            x_l[k]=min_b2;
            x_u[k]=max_b2;
        }

        return true;
    }

    /****************************************************************/
    bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                            bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
    {
        Ipopt::Number *x_l=new Ipopt::Number[n];
        Ipopt::Number *x_u=new Ipopt::Number[n];
        Ipopt::Number *g_l=new Ipopt::Number[n];
        Ipopt::Number *g_u=new Ipopt::Number[n];

        get_bounds_info(n,x_l,x_u,m,g_l,g_u);
        if (randomInit)
        {
            Rand::init();
            for (Ipopt::Index i=0; i<n; i++)
                x[i]=Rand::scalar(x_l[i],x_u[i]);
        }
        else
            fillVar(x,x_l,x_u);

        delete[] x_l;
        delete[] x_u;
        delete[] g_l;
        delete[] g_u;

        return true;
    }

    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {
        fillNet(x);

        obj_value=0.0;
        for (size_t i=0; i<in.size(); i++)
        {
            Vector pred=net.predict(in[i]);
            obj_value+=norm2(out[i]-pred);
        }

        obj_value/=in.size();
        return true;
    }

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
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
        error=0.0;
        pred.clear();
        for (size_t i=0; i<in.size(); i++)
        {
            Vector pred=net.predict(in[i]);            
            error+=norm2(out[i]-pred);
            this->pred.push_back(pred);
        }
        error/=in.size();
    }
};

}

}


/****************************************************************/
ff2LayNNTrain::ff2LayNNTrain()
{
    App=new Ipopt::IpoptApplication();
    CAST_IPOPTAPP(App)->Options()->SetNumericValue("tol",1e-8);
    CAST_IPOPTAPP(App)->Options()->SetIntegerValue("acceptable_iter",0);
    CAST_IPOPTAPP(App)->Options()->SetStringValue("mu_strategy","adaptive");
    CAST_IPOPTAPP(App)->Options()->SetIntegerValue("max_iter",300);
    CAST_IPOPTAPP(App)->Options()->SetStringValue("nlp_scaling_method","gradient-based");
    CAST_IPOPTAPP(App)->Options()->SetStringValue("hessian_approximation","limited-memory");
    CAST_IPOPTAPP(App)->Options()->SetStringValue("jacobian_approximation","finite-difference-values");
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
    if ((in.size()==0) || (in.size()!=out.size()))
        return false;

    IW.clear();
    IW.assign(numHiddenNodes,Vector(in.front().length(),0.0));
    b1.resize(numHiddenNodes,0.0);

    LW.clear();
    LW.assign(out.front().length(),Vector(numHiddenNodes,0.0));
    b2.resize(out.front().length(),0.0);

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

    // enlarge of 10%
    for (size_t i=0; i<inMinMaxX.size(); i++)
    {   
        inMinMaxX[i].min-=0.1*fabs(inMinMaxX[i].min);
        inMinMaxX[i].max+=0.1*fabs(inMinMaxX[i].max);
    }

    // seek for min-max of output and scale it in [-1,1]
    const Vector &out_front=out.front();
    for (size_t i=0; i<out_front.length(); i++)
    {
        minmax range;
        range.min=range.max=out_front[i];
        outMinMaxX.push_back(range);
        range.min=-1.0; range.max=1.0;
        outMinMaxY.push_back(range);
    }
    for (size_t i=1; i<out.size(); i++)
    {
        for (size_t j=0; j<out_front.length(); j++)
        {
            outMinMaxX[j].min=std::min(outMinMaxX[j].min,out[i][j]);
            outMinMaxX[j].max=std::max(outMinMaxX[j].max,out[i][j]);
        }
    }

    // enlarge of 10%
    for (size_t i=0; i<outMinMaxX.size(); i++)
    {   
        outMinMaxX[i].min-=0.1*fabs(outMinMaxX[i].min);
        outMinMaxX[i].max+=0.1*fabs(outMinMaxX[i].max);
    }

    prepare();
    configured=true;

    Ipopt::SmartPtr<ff2LayNNTrainNLP> nlp=new ff2LayNNTrainNLP(*this,bounds,true,in,out,pred);
    Ipopt::ApplicationReturnStatus status=CAST_IPOPTAPP(App)->OptimizeTNLP(GetRawPtr(nlp));

    error=nlp->get_error();
    return (status==Ipopt::Solve_Succeeded);
}


/****************************************************************/
bool ff2LayNNTrain::retrain(const deque<Vector> &in, const deque<Vector> &out,
                            deque<Vector> &pred, double &error)
{
    if ((in.size()==0) || (in.size()!=out.size()) || !configured)
        return false;

    Ipopt::SmartPtr<ff2LayNNTrainNLP> nlp=new ff2LayNNTrainNLP(*this,bounds,false,in,out,pred);
    Ipopt::ApplicationReturnStatus status=CAST_IPOPTAPP(App)->OptimizeTNLP(GetRawPtr(nlp));

    error=nlp->get_error();
    return (status==Ipopt::Solve_Succeeded);    
}


/****************************************************************/
ff2LayNNTrain::~ff2LayNNTrain()
{
    delete CAST_IPOPTAPP(App);
}


