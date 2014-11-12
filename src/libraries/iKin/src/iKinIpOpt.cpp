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

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

#include <iCub/iKin/iKinIpOpt.h>

#define CAST_IPOPTAPP(x)                    (static_cast<IpoptApplication*>(x))
#define IKINIPOPT_SHOULDER_MAXABDUCTION     (100.0*CTRL_DEG2RAD)

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace Ipopt;


/************************************************************************/
iKinLinIneqConstr::iKinLinIneqConstr()
{
    lowerBoundInf=IKINIPOPT_DEFAULT_LWBOUNDINF;
    upperBoundInf=IKINIPOPT_DEFAULT_UPBOUNDINF;
    active=false;
}


/************************************************************************/
iKinLinIneqConstr::iKinLinIneqConstr(const double _lowerBoundInf,
                                     const double _upperBoundInf)
{
    lowerBoundInf=_lowerBoundInf;
    upperBoundInf=_upperBoundInf;
    active=false;
}


/************************************************************************/
void iKinLinIneqConstr::clone(const iKinLinIneqConstr *obj)
{
    C =obj->C;
    uB=obj->uB;
    lB=obj->lB;

    lowerBoundInf=obj->lowerBoundInf;
    upperBoundInf=obj->upperBoundInf;
    active=obj->active;
}


/************************************************************************/
iKinLinIneqConstr::iKinLinIneqConstr(const iKinLinIneqConstr &obj)
{
    clone(&obj);
}


/************************************************************************/
iKinLinIneqConstr &iKinLinIneqConstr::operator=(const iKinLinIneqConstr &obj)
{
    clone(&obj);

    return *this;
}


/************************************************************************/
void iCubShoulderConstr::clone(const iKinLinIneqConstr *obj)
{
    iKinLinIneqConstr::clone(obj);

    const iCubShoulderConstr *ptr=static_cast<const iCubShoulderConstr*>(obj);

    shou_m=ptr->shou_m;
    shou_n=ptr->shou_n;
    elb_m=ptr->elb_m;
    elb_n=ptr->elb_n;
    
    chain=ptr->chain;
}


/************************************************************************/
void iCubShoulderConstr::appendMatrixRow(yarp::sig::Matrix &dest,
                                         const yarp::sig::Vector &row)
{
    yarp::sig::Matrix tmp;

    // if dest is already filled with something
    if (dest.rows())
    {   
        // exit if lengths do not match     
        if (row.length()!=dest.cols())
            return;

        tmp.resize(dest.rows()+1,dest.cols());

        // copy the content of dest in temp
        for (int i=0; i<dest.rows(); i++)
            for (int j=0; j<dest.cols(); j++)
                tmp(i,j)=dest(i,j);

        // reassign dest
        dest=tmp;
    }
    else
        dest.resize(1,row.length());

    // append the last row
    for (int i=0; i<dest.cols(); i++)
        dest(dest.rows()-1,i)=row[i];
}


/************************************************************************/
void iCubShoulderConstr::appendVectorValue(yarp::sig::Vector &dest, double val)
{
    yarp::sig::Vector tmp(dest.length()+1);
    for (size_t i=0; i<dest.length(); i++)
        tmp[i]=dest[i];

    dest=tmp;
    dest[dest.length()-1]=val;
}


/************************************************************************/
iCubShoulderConstr::iCubShoulderConstr(iCubArm &arm) : iKinLinIneqConstr()
{      
    chain=arm.asChain();

    double joint1_0, joint1_1;
    double joint2_0, joint2_1;
    joint1_0= 28.0*CTRL_DEG2RAD;
    joint1_1= 23.0*CTRL_DEG2RAD;
    joint2_0=-37.0*CTRL_DEG2RAD;
    joint2_1= 80.0*CTRL_DEG2RAD;
    shou_m=(joint1_1-joint1_0)/(joint2_1-joint2_0);
    shou_n=joint1_0-shou_m*joint2_0;

    double joint3_0, joint3_1;
    double joint4_0, joint4_1;
    joint3_0= 85.0*CTRL_DEG2RAD;
    joint3_1=105.0*CTRL_DEG2RAD;
    joint4_0= 90.0*CTRL_DEG2RAD;
    joint4_1= 40.0*CTRL_DEG2RAD;
    elb_m=(joint4_1-joint4_0)/(joint3_1-joint3_0);
    elb_n=joint4_0-elb_m*joint3_0;

    update(NULL);
}


/************************************************************************/
void iCubShoulderConstr::update(void*)
{
    // optimization won't use LinIneqConstr by default
    setActive(false);

    yarp::sig::Matrix _C;
    yarp::sig::Vector _lB;
    yarp::sig::Vector _uB;

    yarp::sig::Vector row(chain->getDOF());

    // if shoulder's axes are controlled, constraint them
    if (!(*chain)[3].isBlocked() && !(*chain)[3+1].isBlocked() &&
        !(*chain)[3+2].isBlocked())
    {
        // compute offset to shoulder's axes
        // given the blocked/released status of
        // previous link
        int offs=0;
        for (int i=0; i<3; i++)
            if (!(*chain)[i].isBlocked())
                offs++;

        // constraints on the cables length
        row.zero();
        row[offs]=1.71; row[offs+1]=-1.71;
        appendMatrixRow(_C,row);
        appendVectorValue(_lB,-347.00*CTRL_DEG2RAD);
        appendVectorValue(_uB,upperBoundInf);

        row.zero();
        row[offs]=1.71; row[offs+1]=-1.71; row[offs+2]=-1.71;
        appendMatrixRow(_C,row);
        appendVectorValue(_lB,-366.57*CTRL_DEG2RAD);
        appendVectorValue(_uB,112.42*CTRL_DEG2RAD);

        row.zero();
        row[offs+1]=1.0; row[offs+2]=1.0;
        appendMatrixRow(_C,row);
        appendVectorValue(_lB,-66.600*CTRL_DEG2RAD);
        appendVectorValue(_uB,213.30*CTRL_DEG2RAD);

        // constraints to prevent arm from touching torso
        row.zero();
        row[offs+1]=1.0; row[offs+2]=-shou_m;
        appendMatrixRow(_C,row);
        appendVectorValue(_lB,shou_n);
        appendVectorValue(_uB,upperBoundInf);

        // constraints to limit shoulder abduction
        row.zero();
        row[offs+1]=1.0;
        appendMatrixRow(_C,row);
        appendVectorValue(_lB,lowerBoundInf);
        appendVectorValue(_uB,IKINIPOPT_SHOULDER_MAXABDUCTION);

        // optimization will use LinIneqConstr
        getC()=_C;
        getlB()=_lB;
        getuB()=_uB;
        setActive(true);
    }

    // if elbow and pronosupination axes are controlled, constraint them
    if (!(*chain)[3+3].isBlocked() && !(*chain)[3+3+1].isBlocked())
    {
        // compute offset to elbow's axis
        // given the blocked/released status of
        // previous link
        int offs=0;
        for (int i=0; i<(3+3); i++)
            if (!(*chain)[i].isBlocked())
                offs++;

        // constraints to prevent the forearm from hitting the arm
        row.zero();
        row[offs]=-elb_m; row[offs+1]=1.0;
        appendMatrixRow(_C,row);
        appendVectorValue(_lB,lowerBoundInf);
        appendVectorValue(_uB,elb_n);

        row.zero();
        row[offs]=elb_m; row[offs+1]=1.0;
        appendMatrixRow(_C,row);
        appendVectorValue(_lB,-elb_n);
        appendVectorValue(_uB,upperBoundInf);

        // optimization will use LinIneqConstr
        getC()=_C;
        getlB()=_lB;
        getuB()=_uB;
        setActive(true);
    }
}


/************************************************************************/
class iKin_NLP : public TNLP
{
private:
    // Copy constructor: not implemented.
    iKin_NLP(const iKin_NLP&);
    // Assignment operator: not implemented.
    iKin_NLP &operator=(const iKin_NLP&);

protected:
    iKinChain &chain;
    iKinChain &chain2ndTask;

    iKinLinIneqConstr &LIC;

    unsigned int dim;
    unsigned int dim_2nd;
    unsigned int ctrlPose;

    yarp::sig::Vector &xd;
    yarp::sig::Vector &xd_2nd;
    yarp::sig::Vector &w_2nd;
    yarp::sig::Vector &qd_3rd;
    yarp::sig::Vector &w_3rd;
    yarp::sig::Vector  qd;
    yarp::sig::Vector  q0;
    yarp::sig::Vector  q;
    bool              *exhalt;

    yarp::sig::Vector  e_zero;
    yarp::sig::Vector  e_xyz;
    yarp::sig::Vector  e_ang;
    yarp::sig::Vector  e_2nd;
    yarp::sig::Vector  e_3rd;

    yarp::sig::Matrix  J_zero;
    yarp::sig::Matrix  J_xyz;
    yarp::sig::Matrix  J_ang;
    yarp::sig::Matrix  J_2nd;

    yarp::sig::Vector *e_1st;
    yarp::sig::Matrix *J_1st;
    yarp::sig::Vector *e_cst;
    yarp::sig::Matrix *J_cst;

    yarp::sig::Vector linC;

    double __obj_scaling;
    double __x_scaling;
    double __g_scaling;
    double lowerBoundInf;
    double upperBoundInf;
    double translationalTol;

    iKinIterateCallback *callback;

    double weight2ndTask;
    double weight3rdTask;
    bool   firstGo;

    /************************************************************************/
    virtual void computeQuantities(const Number *x)
    {
        yarp::sig::Vector new_q(dim);
        for (Index i=0; i<(int)dim; i++)
            new_q[i]=x[i];

        if (!(q==new_q) || firstGo)
        {
            firstGo=false;
            q=new_q;

            yarp::sig::Vector v(4,0.0);
            if (xd.length()>=7)
            {
                v[0]=xd[3];
                v[1]=xd[4];
                v[2]=xd[5];
                v[3]=xd[6];
            }

            yarp::sig::Matrix Des=axis2dcm(v);
            Des(0,3)=xd[0];
            Des(1,3)=xd[1];
            Des(2,3)=xd[2];
        
            q=chain.setAng(q);
            yarp::sig::Matrix H=chain.getH();
            yarp::sig::Matrix E=Des*SE3inv(H);
            v=dcm2axis(E);
            
            e_xyz[0]=xd[0]-H(0,3);
            e_xyz[1]=xd[1]-H(1,3);
            e_xyz[2]=xd[2]-H(2,3);
            e_ang[0]=v[3]*v[0];
            e_ang[1]=v[3]*v[1];
            e_ang[2]=v[3]*v[2];

            yarp::sig::Matrix J1=chain.GeoJacobian();
            submatrix(J1,J_xyz,0,2,0,dim-1);
            submatrix(J1,J_ang,3,5,0,dim-1);

            if (weight2ndTask!=0.0)
            {
                yarp::sig::Matrix H_2nd=chain2ndTask.getH();
                e_2nd[0]=w_2nd[0]*(xd_2nd[0]-H_2nd(0,3));
                e_2nd[1]=w_2nd[1]*(xd_2nd[1]-H_2nd(1,3));
                e_2nd[2]=w_2nd[2]*(xd_2nd[2]-H_2nd(2,3));

                yarp::sig::Matrix J2=chain2ndTask.GeoJacobian();

                for (unsigned int i=0; i<dim_2nd; i++)
                {
                    J_2nd(0,i)=w_2nd[0]*J2(0,i);
                    J_2nd(1,i)=w_2nd[1]*J2(1,i);
                    J_2nd(2,i)=w_2nd[2]*J2(2,i);
                }
            }

            if (weight3rdTask!=0.0)
                for (unsigned int i=0; i<dim; i++)
                    e_3rd[i]=w_3rd[i]*(qd_3rd[i]-q[i]);

            if (LIC.isActive())
                linC=LIC.getC()*q;
        }
    }


public:
    /************************************************************************/
    iKin_NLP(iKinChain &c, unsigned int _ctrlPose, const yarp::sig::Vector &_q0,
             yarp::sig::Vector &_xd, double _weight2ndTask, iKinChain &_chain2ndTask,
             yarp::sig::Vector &_xd_2nd, yarp::sig::Vector &_w_2nd, double _weight3rdTask,
             yarp::sig::Vector &_qd_3rd, yarp::sig::Vector &_w_3rd, iKinLinIneqConstr &_LIC,
             bool *_exhalt=NULL) :
             chain(c), q0(_q0), xd(_xd),
             chain2ndTask(_chain2ndTask),   xd_2nd(_xd_2nd), w_2nd(_w_2nd),
             weight3rdTask(_weight3rdTask), qd_3rd(_qd_3rd), w_3rd(_w_3rd),
             LIC(_LIC), 
             exhalt(_exhalt)
    {
        dim=chain.getDOF();
        dim_2nd=chain2ndTask.getDOF();

        ctrlPose=_ctrlPose;

        if (ctrlPose>IKINCTRL_POSE_ANG)
            ctrlPose=IKINCTRL_POSE_ANG;

        weight2ndTask=dim_2nd>0 ? _weight2ndTask : 0.0;

        qd.resize(dim);

        unsigned int n=q0.length();
        n=n>dim ? dim : n;

        unsigned int i;
        for (i=0; i<n; i++)
            qd[i]=q0[i];

        for (; i<dim; i++)
            qd[i]=0.0;

        q=qd;

        e_zero.resize(3,0.0);
        e_xyz.resize(3,0.0);
        e_ang.resize(3,0.0);
        e_2nd.resize(3,0.0);
        e_3rd.resize(dim,0.0);

        J_zero.resize(3,dim); J_zero.zero();
        J_xyz.resize(3,dim);  J_xyz.zero();
        J_ang.resize(3,dim);  J_ang.zero();
        J_2nd.resize(3,dim);  J_2nd.zero();

        if (ctrlPose==IKINCTRL_POSE_FULL)
        {
            e_1st=&e_ang;
            J_1st=&J_ang;
        }
        else
        {
            e_1st=&e_zero;
            J_1st=&J_zero;
        }

        e_cst=&e_xyz;
        J_cst=&J_xyz;

        firstGo=true;

        __obj_scaling=1.0;
        __x_scaling  =1.0;
        __g_scaling  =1.0;

        lowerBoundInf=IKINIPOPT_DEFAULT_LWBOUNDINF;
        upperBoundInf=IKINIPOPT_DEFAULT_UPBOUNDINF;
        translationalTol=IKINIPOPT_DEFAULT_TRANSTOL;

        callback=NULL;
    }

    /************************************************************************/
    yarp::sig::Vector get_qd() { return qd; }

    /************************************************************************/
    void set_callback(iKinIterateCallback *_callback) { callback=_callback; }

    /************************************************************************/
    void set_scaling(double _obj_scaling, double _x_scaling, double _g_scaling)
    {
        __obj_scaling=_obj_scaling;
        __x_scaling  =_x_scaling;
        __g_scaling  =_g_scaling;
    }

    /************************************************************************/
    void set_bound_inf(double lower, double upper)
    {
        lowerBoundInf=lower;
        upperBoundInf=upper;
    }

    /************************************************************************/
    void set_translational_tol(double tol) { translationalTol=tol; }

    /************************************************************************/
    bool set_posePriority(const string &priority)
    {
        if (priority=="position")
        {
            e_1st=&e_ang;
            J_1st=&J_ang;

            e_cst=&e_xyz;
            J_cst=&J_xyz;
        }
        else if (priority=="orientation")
        {
            e_1st=&e_xyz;
            J_1st=&J_xyz;

            e_cst=&e_ang;
            J_cst=&J_ang;
        }
        else 
            return false;

        return true;
    }

    /************************************************************************/
    bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g, Index& nnz_h_lag,
                      IndexStyleEnum& index_style)
    {
        n=dim;
        m=1;
        nnz_jac_g=dim;

        if (LIC.isActive())
        {
            int lenLower=LIC.getlB().length();
            int lenUpper=LIC.getuB().length();

            if (lenLower && (lenLower==lenUpper) && (LIC.getC().cols()==dim))
            {
                m+=lenLower;
                nnz_jac_g+=lenLower*dim;
            }
            else
                LIC.setActive(false);
        }
        
        nnz_h_lag=(dim*(dim+1))>>1;
        index_style=TNLP::C_STYLE;
        
        return true;
    }

    /************************************************************************/
    bool get_bounds_info(Index n, Number* x_l, Number* x_u, Index m, Number* g_l,
                         Number* g_u)
    {
        for (Index i=0; i<n; i++)
        {
            x_l[i]=chain(i).getMin();
            x_u[i]=chain(i).getMax();
        }
        
        Index offs=0;

        for (Index i=0; i<m; i++)
        {
            if (i==0)
            {
                g_l[0]=lowerBoundInf;
                g_u[0]=translationalTol;
                offs=1;
            }
            else
            {
                g_l[i]=LIC.getlB()[i-offs];
                g_u[i]=LIC.getuB()[i-offs];
            }
        }

        return true;
    }
    
    /************************************************************************/
    bool get_starting_point(Index n, bool init_x, Number* x, bool init_z,
                            Number* z_L, Number* z_U, Index m, bool init_lambda,
                            Number* lambda)
    {
        for (Index i=0; i<n; i++)
            x[i]=q0[i];

        return true;
    }
    
    /************************************************************************/
    bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
    {
        computeQuantities(x);

        obj_value=norm2(*e_1st);

        if (weight2ndTask!=0.0)
            obj_value+=weight2ndTask*norm2(e_2nd);

        if (weight3rdTask!=0.0)
            obj_value+=weight3rdTask*norm2(e_3rd);

        return true;
    }
    
    /************************************************************************/
    bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
    {
        computeQuantities(x);

        yarp::sig::Vector grad=-2.0*(J_1st->transposed() * *e_1st);

        if (weight2ndTask!=0.0)
            grad=grad-2.0*weight2ndTask*(J_2nd.transposed()*e_2nd);

        if (weight3rdTask!=0.0)
            grad=grad-2.0*weight3rdTask*(w_3rd*e_3rd);

        for (Index i=0; i<n; i++)
            grad_f[i]=grad[i];

        return true;
    }
    
    /************************************************************************/
    bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
    {
        computeQuantities(x);

        Index offs=0;

        for (Index i=0; i<m; i++)
        {
            if (i==0)
            {
                g[0]=norm2(*e_cst);
                offs=1;
            }
            else
                g[i]=linC[i-offs];
        }

        return true;
    }
    
    /************************************************************************/
    bool eval_jac_g(Index n, const Number* x, bool new_x, Index m, Index nele_jac,
                    Index* iRow, Index *jCol, Number* values)
    {
        if (m!=0)
        {
            if (values==NULL)
            {
                Index idx=0;
        
                for (Index row=0; row<m; row++)
                {
                    for (Index col=0; col<n; col++)
                    {
                        iRow[idx]=row;
                        jCol[idx]=col;
                        idx++;
                    }
                }
            }
            else
            {
                computeQuantities(x);
            
                yarp::sig::Vector grad=-2.0*(J_cst->transposed() * *e_cst);

                Index idx =0;
                Index offs=0;

                for (Index row=0; row<m; row++)
                {
                    for (Index col=0; col<n; col++)
                    {    
                        if (row==0)
                        {
                            values[idx]=grad[idx];                    
                            offs=1;
                        }
                        else
                            values[idx]=LIC.getC()(row-offs,col);
                    
                        idx++;
                    }
                }
            }
        }

        return true;
    }
    
    /************************************************************************/
    bool eval_h(Index n, const Number* x, bool new_x, Number obj_factor,
                Index m, const Number* lambda, bool new_lambda,
                Index nele_hess, Index* iRow, Index* jCol, Number* values)
    {
        if (values==NULL)
        {
            Index idx=0;
        
            for (Index row=0; row<n; row++)
            {
                for (Index col=0; col<=row; col++)
                {
                    iRow[idx]=row;
                    jCol[idx]=col;
                    idx++;
                }
            }
        }
        else
        {
            // Given the task: min f(q)=||xd-F(q)||^2
            // the Hessian Hij is: 2 * (<dF/dqi,dF/dqj> - <d2F/dqidqj,e>)
            computeQuantities(x);
            chain.prepareForHessian();

            if (weight2ndTask!=0.0)
                chain2ndTask.prepareForHessian();

            Index idx=0;
            for (Index row=0; row<n; row++)
            {
                for (Index col=0; col<=row; col++)
                {
                    // warning: row and col are swapped due to asymmetry
                    // of orientation part within the hessian 
                    yarp::sig::Vector h=chain.fastHessian_ij(col,row);
                    yarp::sig::Vector h_xyz(3), h_ang(3), h_zero(3,0.0);
                    h_xyz[0]=h[0];
                    h_xyz[1]=h[1];
                    h_xyz[2]=h[2];
                    h_ang[0]=h[3];
                    h_ang[1]=h[4];
                    h_ang[2]=h[5];
                
                    yarp::sig::Vector *h_1st;
                    if (ctrlPose==IKINCTRL_POSE_FULL)
                        h_1st=&h_ang;
                    else
                        h_1st=&h_zero;

                    yarp::sig::Vector *h_cst=(e_cst==&e_xyz)?&h_xyz:&h_ang;
                
                    values[idx]=2.0*(obj_factor*(dot(*J_1st,row,*J_1st,col)-dot(*h_1st,*e_1st))+
                                     lambda[0]*(dot(*J_cst,row,*J_cst,col)-dot(*h_cst,*e_cst)));
                
                    if ((weight2ndTask!=0.0) && (row<(int)dim_2nd) && (col<(int)dim_2nd))
                    {    
                        // warning: row and col are swapped due to asymmetry
                        // of orientation part within the hessian 
                        yarp::sig::Vector h2=chain2ndTask.fastHessian_ij(col,row);
                        yarp::sig::Vector h_2nd(3);
                        h_2nd[0]=(w_2nd[0]*w_2nd[0])*h2[0];
                        h_2nd[1]=(w_2nd[1]*w_2nd[1])*h2[1];
                        h_2nd[2]=(w_2nd[2]*w_2nd[2])*h2[2];
                
                        values[idx]+=2.0*obj_factor*weight2ndTask*(dot(J_2nd,row,J_2nd,col)-dot(h_2nd,e_2nd));
                    }
                
                    idx++;
                }
            }
        }
        
        return true;
    }

    /************************************************************************/
    bool intermediate_callback(AlgorithmMode mode, Index iter, Number obj_value,
                               Number inf_pr, Number inf_du, Number mu, Number d_norm,
                               Number regularization_size, Number alpha_du, Number alpha_pr,
                               Index ls_trials, const IpoptData* ip_data,
                               IpoptCalculatedQuantities* ip_cq)
    {
        if (callback!=NULL)
            callback->exec(xd,q);

        if (exhalt!=NULL)
            return !(*exhalt);
        else
            return true;
    }

    /************************************************************************/
    bool get_scaling_parameters(Number& obj_scaling, bool& use_x_scaling, Index n,
                                Number* x_scaling, bool& use_g_scaling, Index m,
                                Number* g_scaling)
    {
        obj_scaling=__obj_scaling;

        for (Index i=0; i<n; i++)
            x_scaling[i]=__x_scaling;

        for (Index j=0; j<m; j++)
            g_scaling[j]=__g_scaling;

        use_x_scaling=use_g_scaling=true;

        return true;
    }

    /************************************************************************/
    void finalize_solution(SolverReturn status, Index n, const Number* x,
                           const Number* z_L, const Number* z_U, Index m,
                           const Number* g, const Number* lambda, Number obj_value,
                           const IpoptData* ip_data, IpoptCalculatedQuantities* ip_cq)
    {
        for (Index i=0; i<n; i++)
            qd[i]=x[i];

        qd=chain.setAng(qd);
    }

    /************************************************************************/
    virtual ~iKin_NLP() { }
};


/************************************************************************/
iKinIpOptMin::iKinIpOptMin(iKinChain &c, const unsigned int _ctrlPose, const double tol,
                           const int max_iter, const unsigned int verbose, bool useHessian) :
                           chain(c)
{
    ctrlPose=_ctrlPose;
    posePriority="position";
    pLIC=&noLIC;

    if (ctrlPose>IKINCTRL_POSE_ANG)
        ctrlPose=IKINCTRL_POSE_ANG;

    chain.setAllConstraints(false); // this is required since IpOpt initially relaxes constraints

    App=new IpoptApplication();

    CAST_IPOPTAPP(App)->Options()->SetNumericValue("tol",tol);
    CAST_IPOPTAPP(App)->Options()->SetNumericValue("acceptable_tol",tol);
    CAST_IPOPTAPP(App)->Options()->SetIntegerValue("acceptable_iter",10);
    CAST_IPOPTAPP(App)->Options()->SetStringValue("mu_strategy","adaptive");
    CAST_IPOPTAPP(App)->Options()->SetIntegerValue("print_level",verbose);

    getBoundsInf(lowerBoundInf,upperBoundInf);

    translationalTol=IKINIPOPT_DEFAULT_TRANSTOL;

    if (max_iter>0)
        CAST_IPOPTAPP(App)->Options()->SetIntegerValue("max_iter",max_iter);
    else
        CAST_IPOPTAPP(App)->Options()->SetIntegerValue("max_iter",(Index)upperBoundInf);

    if (!useHessian)
        CAST_IPOPTAPP(App)->Options()->SetStringValue("hessian_approximation","limited-memory");

    CAST_IPOPTAPP(App)->Initialize();
}


/************************************************************************/
void iKinIpOptMin::set_ctrlPose(const unsigned int _ctrlPose)
{
    ctrlPose=_ctrlPose;

    if (ctrlPose>IKINCTRL_POSE_ANG)
        ctrlPose=IKINCTRL_POSE_ANG;
}


/************************************************************************/
bool iKinIpOptMin::set_posePriority(const string &priority)
{
    if ((priority=="position") || (priority=="orientation"))
    {
        posePriority=priority;
        return true;
    }
    else 
        return false;
}


/************************************************************************/
void iKinIpOptMin::specify2ndTaskEndEff(const unsigned int n)
{
    unsigned int _n=n;
    if (_n>chain.getN())
        _n=chain.getN();

    chain2ndTask.clear();
    for (unsigned int i=0; i<_n; i++)
        chain2ndTask<<chain[i];

    chain2ndTask.setH0(chain.getH0());
    if (_n==chain.getN())
        chain2ndTask.setHN(chain.getHN()); 
}


/************************************************************************/
iKinChain &iKinIpOptMin::get2ndTaskChain()
{
    return chain2ndTask;
}


/************************************************************************/
void iKinIpOptMin::setMaxIter(const int max_iter)
{
    if (max_iter>0)
        CAST_IPOPTAPP(App)->Options()->SetIntegerValue("max_iter",max_iter);
    else
        CAST_IPOPTAPP(App)->Options()->SetIntegerValue("max_iter",(Index)2e9);

    CAST_IPOPTAPP(App)->Initialize();
}


/************************************************************************/
int iKinIpOptMin::getMaxIter() const
{
    int max_iter;
    CAST_IPOPTAPP(App)->Options()->GetIntegerValue("max_iter",max_iter,"");
    return max_iter;
}


/************************************************************************/
void iKinIpOptMin::setTol(const double tol)
{
    CAST_IPOPTAPP(App)->Options()->SetNumericValue("tol",tol);
    CAST_IPOPTAPP(App)->Options()->SetNumericValue("acceptable_tol",tol);

    CAST_IPOPTAPP(App)->Initialize();
}


/************************************************************************/
double iKinIpOptMin::getTol() const
{
    double tol;
    CAST_IPOPTAPP(App)->Options()->GetNumericValue("tol",tol,"");
    return tol;
}


/************************************************************************/
void iKinIpOptMin::setVerbosity(const unsigned int verbose)
{
    CAST_IPOPTAPP(App)->Options()->SetIntegerValue("print_level",verbose);

    CAST_IPOPTAPP(App)->Initialize();
}


/************************************************************************/
void iKinIpOptMin::setHessianOpt(const bool useHessian)
{
    if (useHessian)
        CAST_IPOPTAPP(App)->Options()->SetStringValue("hessian_approximation","exact");
    else
        CAST_IPOPTAPP(App)->Options()->SetStringValue("hessian_approximation","limited-memory");

    CAST_IPOPTAPP(App)->Initialize();
}


/************************************************************************/
void iKinIpOptMin::setUserScaling(const bool useUserScaling, const double _obj_scaling,
                                  const double _x_scaling, const double _g_scaling)
{
    if (useUserScaling)
    {
        obj_scaling=_obj_scaling;
        x_scaling  =_x_scaling;
        g_scaling  =_g_scaling;

        CAST_IPOPTAPP(App)->Options()->SetStringValue("nlp_scaling_method","user-scaling");
    }
    else
        CAST_IPOPTAPP(App)->Options()->SetStringValue("nlp_scaling_method","gradient-based");

    CAST_IPOPTAPP(App)->Initialize();
}


/************************************************************************/
void iKinIpOptMin::setDerivativeTest(const bool enableTest, const bool enable2ndDer)
{
    if (enableTest)
    {
        if (enable2ndDer)
            CAST_IPOPTAPP(App)->Options()->SetStringValue("derivative_test","second-order");
        else
            CAST_IPOPTAPP(App)->Options()->SetStringValue("derivative_test","first-order");

        CAST_IPOPTAPP(App)->Options()->SetStringValue("derivative_test_print_all","yes");
    }
    else
        CAST_IPOPTAPP(App)->Options()->SetStringValue("derivative_test","none");

    CAST_IPOPTAPP(App)->Initialize();
}


/************************************************************************/
void iKinIpOptMin::getBoundsInf(double &lower, double &upper)
{
    CAST_IPOPTAPP(App)->Options()->GetNumericValue("nlp_lower_bound_inf",lower,"");
    CAST_IPOPTAPP(App)->Options()->GetNumericValue("nlp_upper_bound_inf",upper,"");
}


/************************************************************************/
void iKinIpOptMin::setBoundsInf(const double lower, const double upper)
{
    CAST_IPOPTAPP(App)->Options()->SetNumericValue("nlp_lower_bound_inf",lower);
    CAST_IPOPTAPP(App)->Options()->SetNumericValue("nlp_upper_bound_inf",upper);

    lowerBoundInf=lower;
    upperBoundInf=upper;
}


/************************************************************************/
yarp::sig::Vector iKinIpOptMin::solve(const yarp::sig::Vector &q0, yarp::sig::Vector &xd,
                                      double weight2ndTask, yarp::sig::Vector &xd_2nd,
                                      yarp::sig::Vector &w_2nd, double weight3rdTask,
                                      yarp::sig::Vector &qd_3rd, yarp::sig::Vector &w_3rd,
                                      int *exit_code, bool *exhalt, iKinIterateCallback *iterate)
{
    SmartPtr<iKin_NLP> nlp=new iKin_NLP(chain,ctrlPose,q0,xd,
                                        weight2ndTask,chain2ndTask,xd_2nd,w_2nd,
                                        weight3rdTask,qd_3rd,w_3rd,
                                        *pLIC,exhalt);
    
    nlp->set_scaling(obj_scaling,x_scaling,g_scaling);
    nlp->set_bound_inf(lowerBoundInf,upperBoundInf);
    nlp->set_translational_tol(translationalTol);
    nlp->set_posePriority(posePriority);
    nlp->set_callback(iterate);

    ApplicationReturnStatus status=CAST_IPOPTAPP(App)->OptimizeTNLP(GetRawPtr(nlp));

    if (exit_code!=NULL)
        *exit_code=status;

    return nlp->get_qd();
}


/************************************************************************/
yarp::sig::Vector iKinIpOptMin::solve(const yarp::sig::Vector &q0, yarp::sig::Vector &xd,
                                      double weight2ndTask, yarp::sig::Vector &xd_2nd,
                                      yarp::sig::Vector &w_2nd)
{
    yarp::sig::Vector dummy(1);
    return solve(q0,xd,weight2ndTask,xd_2nd,w_2nd,0.0,dummy,dummy);
}


/************************************************************************/
yarp::sig::Vector iKinIpOptMin::solve(const yarp::sig::Vector &q0, yarp::sig::Vector &xd)
{
    yarp::sig::Vector dummy(1);
    return solve(q0,xd,0.0,dummy,dummy,0.0,dummy,dummy);
}


/************************************************************************/
iKinIpOptMin::~iKinIpOptMin()
{
    delete CAST_IPOPTAPP(App);
}


