
#include <iostream>
#include <iomanip>

#include <iCub/iKinIpOpt.h>

#define IKINIPOPT_DEFAULT_TRANSTOL      1e-6

using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace ctrl;
using namespace iKin;
using namespace Ipopt;


/************************************************************************/
iKinLinIneqConstr::iKinLinIneqConstr()
{
    lowerBoundInf=-1e9;
    upperBoundInf=+1e9;
    active=false;
}


/************************************************************************/
iKinLinIneqConstr::iKinLinIneqConstr(Ipopt::Number _lowerBoundInf, Ipopt::Number _upperBoundInf)
{
    lowerBoundInf=_lowerBoundInf;
    upperBoundInf=_upperBoundInf;
    active=false;
}


/************************************************************************/
void iKinLinIneqConstr::_allocate(const iKinLinIneqConstr *obj)
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
    _allocate(&obj);
}


/************************************************************************/
iKinLinIneqConstr &iKinLinIneqConstr::operator=(const iKinLinIneqConstr &obj)
{
    _allocate(&obj);

    return *this;
}


/************************************************************************/
iKin_NLP::iKin_NLP(iKinChain &c, unsigned int _ctrlPose, const yarp::sig::Vector &_q0, yarp::sig::Vector &_xd,
                   double _weight2ndTask, iKinChain &_chain2ndTask, yarp::sig::Vector &_xd_2nd, yarp::sig::Vector &_w_2nd,
                   double _weight3rdTask, yarp::sig::Vector &_qd_3rd, yarp::sig::Vector &_w_3rd,
                   iKinLinIneqConstr &_LIC, bool *_exhalt) :
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

    size_t n=q0.length();
    n=n>dim ? dim : n;

    unsigned int i;
    for (i=0; i<n; i++)
        qd[i]=q0[i];

    for (; i<dim; i++)
        qd[i]=0.0;

    q=qd;

    e_xyz.resize(3,0.0);
    e_ang.resize(3,0.0);
    e_2nd.resize(3,0.0);
    e_3rd.resize(dim,0.0);

    J_xyz.resize(3,dim); J_xyz.zero();
    J_ang.resize(3,dim); J_ang.zero();
    J_2nd.resize(3,dim); J_2nd.zero();

    if (ctrlPose==IKINCTRL_POSE_XYZ)
    {
        e_1st=&e_xyz;
        J_1st=&J_xyz;
    }
    else
    {
        e_1st=&e_ang;
        J_1st=&J_ang;
    }

    firstGo=true;

    __obj_scaling=1.0;
    __x_scaling  =1.0;
    __g_scaling  =1.0;

    lowerBoundInf=-1e9;
    upperBoundInf=+1e9;

    translationalTol=IKINIPOPT_DEFAULT_TRANSTOL;

    callback=NULL;
}


/************************************************************************/
void iKin_NLP::computeQuantities(const Number *x)
{
    yarp::sig::Vector new_q(dim);

    for (Index i=0; i<(int)dim; i++)
        new_q[i]=x[i];

    if (!(q==new_q) || firstGo)
    {
        firstGo=false;
        q=new_q;

        yarp::sig::Vector v(4);
        v[0]=xd[3];
        v[1]=xd[4];
        v[2]=xd[5];
        v[3]=xd[6];
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

        if (weight2ndTask)
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

        if (weight3rdTask)
            for (unsigned int i=0; i<dim; i++)
                e_3rd[i]=w_3rd[i]*(qd_3rd[i]-q[i]);

        if (LIC.isActive())
            linC=LIC.getC()*q;
    }
}


/************************************************************************/
bool iKin_NLP::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                            Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    n=dim;
    m=0;
    nnz_jac_g=0;

    if (LIC.isActive())
    {
        size_t lenLower=LIC.getlB().length();
        size_t lenUpper=LIC.getuB().length();

        if (lenLower && lenLower==lenUpper && LIC.getC().cols()==dim)
        {
            m=lenLower;
            nnz_jac_g=lenLower*dim;
        }
        else
            LIC.setActive(false);
    }
    
    if (ctrlPose==IKINCTRL_POSE_FULL)
    {
        m+=1;
        nnz_jac_g+=dim;
    }

    nnz_h_lag=(dim*(dim+1))>>1;
    
    index_style=TNLP::C_STYLE;
    
    return true;
}


/************************************************************************/
bool iKin_NLP::get_bounds_info(Index n, Number* x_l, Number* x_u,
                               Index m, Number* g_l, Number* g_u)
{
    for (Index i=0; i<n; i++)
    {
        x_l[i]=chain(i).getMin();
        x_u[i]=chain(i).getMax();
    }
    
    Index offs=0;

    for (Index i=0; i<m; i++)
        if (i==0 && ctrlPose==IKINCTRL_POSE_FULL)
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

    return true;
}


/************************************************************************/
bool iKin_NLP::get_starting_point(Index n, bool init_x, Number* x,
                                  bool init_z, Number* z_L, Number* z_U,
                                  Index m, bool init_lambda,
                                  Number* lambda)
{
    for (Index i=0; i<n; i++)
        x[i]=q0[i];

    return true;
}


/************************************************************************/
bool iKin_NLP::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
    computeQuantities(x);

    obj_value=0.5*norm2(*e_1st);

    if (weight2ndTask)
        obj_value+=weight2ndTask*0.5*norm2(e_2nd);

    if (weight3rdTask)
        obj_value+=weight3rdTask*0.5*norm2(e_3rd);

    return true;
}


/************************************************************************/
bool iKin_NLP::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
    computeQuantities(x);

    yarp::sig::Vector grad=-1.0*(J_1st->transposed() * *e_1st);

    if (weight2ndTask)
        grad=grad-weight2ndTask*(J_2nd.transposed()*e_2nd);

    if (weight3rdTask)
        grad=grad-weight3rdTask*(w_3rd*e_3rd);

    for (Index i=0; i<n; i++)
        grad_f[i]=grad[i];

    return true;
}


/************************************************************************/
bool iKin_NLP::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
    computeQuantities(x);

    Index offs=0;

    for (Index i=0; i<m; i++)
        if (i==0 && ctrlPose==IKINCTRL_POSE_FULL)
        {
            g[0]=0.5*norm2(e_xyz);

            offs=1;
        }
        else
            g[i]=linC[i-offs];

    return true;
}


/************************************************************************/
bool iKin_NLP::eval_jac_g(Index n, const Number* x, bool new_x,
                          Index m, Index nele_jac, Index* iRow, Index *jCol,
                          Number* values)
{
    if (m)
    {
        if (!values)
        {
            Index idx=0;
    
            for (Index row=0; row<m; row++)
                for (Index col=0; col<n; col++)
                {
                    iRow[idx]=row;
                    jCol[idx]=col;
                    idx++;
                }
        }
        else
        {
            computeQuantities(x);
        
            yarp::sig::Vector grad=-1.0*(J_xyz.transposed()*e_xyz);

            Index idx =0;
            Index offs=0;

            for (Index row=0; row<m; row++)
                for (Index col=0; col<n; col++)
                {    
                    if (row==0 && ctrlPose==IKINCTRL_POSE_FULL)
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

    return true;
}


/************************************************************************/
bool iKin_NLP::eval_h(Index n, const Number* x, bool new_x, Number obj_factor,
                      Index m, const Number* lambda, bool new_lambda,
                      Index nele_hess, Index* iRow, Index* jCol, Number* values)
{
    if (!values)
    {
        Index idx=0;
    
        for (Index row=0; row<n; row++)
            for (Index col=0; col<=row; col++)
            {
                iRow[idx]=row;
                jCol[idx]=col;
                idx++;
            }
    }
    else
    {
        // Given the task: min f(q)=1/2*||xd-F(q)||^2
        // the Hessian Hij is: <dF/dqi,dF/dqj> - <d2F/dqidqj,e>

        computeQuantities(x);

        chain.prepareForHessian();

        if (weight2ndTask)
            chain2ndTask.prepareForHessian();

        Index idx=0;

        for (Index row=0; row<n; row++)
            for (Index col=0; col<=row; col++)
            {
                yarp::sig::Vector h=chain.fastHessian_ij(row,col);
                yarp::sig::Vector h_xyz(3), h_ang(3);
                h_xyz[0]=h[0];
                h_xyz[1]=h[1];
                h_xyz[2]=h[2];
                h_ang[0]=h[3];
                h_ang[1]=h[4];
                h_ang[2]=h[5];

                yarp::sig::Vector *h_1st;
                if (ctrlPose==IKINCTRL_POSE_XYZ)
                    h_1st=&h_xyz;
                else
                    h_1st=&h_ang;

                values[idx]=obj_factor*(dot(*J_1st,row,*J_1st,col)-dot(*h_1st,*e_1st));

                if (m)
                    values[idx]+=lambda[0]*(dot(J_xyz,row,J_xyz,col)-dot(h_xyz,e_xyz));

                if (weight2ndTask && row<(int)dim_2nd && col<(int)dim_2nd)
                {    
                    yarp::sig::Vector h2=chain2ndTask.fastHessian_ij(row,col);
                    yarp::sig::Vector h_2nd(3);
                    h_2nd[0]=(w_2nd[0]*w_2nd[0])*h2[0];
                    h_2nd[1]=(w_2nd[1]*w_2nd[1])*h2[1];
                    h_2nd[2]=(w_2nd[2]*w_2nd[2])*h2[2];

                    values[idx]+=obj_factor*weight2ndTask*(dot(J_2nd,row,J_2nd,col)-dot(h_2nd,e_2nd));
                }

                idx++;
            }
    }
    
    return true;
}


/************************************************************************/
bool iKin_NLP::get_scaling_parameters(Number& obj_scaling,
                                      bool& use_x_scaling, Index n, Number* x_scaling,
                                      bool& use_g_scaling, Index m, Number* g_scaling)
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
bool iKin_NLP::intermediate_callback(AlgorithmMode mode, Index iter, Number obj_value,
                                     Number inf_pr, Number inf_du, Number mu, Number d_norm,
                                     Number regularization_size, Number alpha_du, Number alpha_pr,
                                     Index ls_trials, const IpoptData* ip_data,
                                     IpoptCalculatedQuantities* ip_cq)
{
    if (callback)
        callback->exec(xd,q);

    if (exhalt)
        return !(*exhalt);
    else
        return true;
}


/************************************************************************/
void iKin_NLP::finalize_solution(SolverReturn status,
                                 Index n, const Number* x, const Number* z_L, const Number* z_U,
                                 Index m, const Number* g, const Number* lambda,
                                 Number obj_value,
                                 const IpoptData* ip_data,
                                 IpoptCalculatedQuantities* ip_cq)
{
    for (Index i=0; i<n; i++)
        qd[i]=x[i];

    qd=chain.setAng(qd);
}


/************************************************************************/
iKinIpOptMin::iKinIpOptMin(iKinChain &c, unsigned int _ctrlPose, const double tol, const int max_iter,
                           const unsigned int verbose, bool useHessian) : chain(c)
{
    ctrlPose=_ctrlPose;
    pLIC=&noLIC;

    if (ctrlPose>IKINCTRL_POSE_ANG)
        ctrlPose=IKINCTRL_POSE_ANG;

    chain.setAllConstraints(false); // this is required since IpOpt initially relaxes constraints

    App=new IpoptApplication();

    App->Options()->SetNumericValue("tol",tol);
    App->Options()->SetNumericValue("acceptable_tol",tol);
    App->Options()->SetIntegerValue("acceptable_iter",10);
    App->Options()->SetStringValue("mu_strategy","adaptive");
    App->Options()->SetIntegerValue("print_level",verbose);

    getBoundsInf(lowerBoundInf,upperBoundInf);

    translationalTol=IKINIPOPT_DEFAULT_TRANSTOL;

    if (max_iter>0)
        App->Options()->SetIntegerValue("max_iter",max_iter);
    else
        App->Options()->SetIntegerValue("max_iter",(Index)upperBoundInf);

    if (!useHessian)
        App->Options()->SetStringValue("hessian_approximation","limited-memory");

    App->Initialize();
}


/************************************************************************/
void iKinIpOptMin::set_ctrlPose(unsigned int _ctrlPose)
{
    ctrlPose=_ctrlPose;

    if (ctrlPose>IKINCTRL_POSE_ANG)
        ctrlPose=IKINCTRL_POSE_ANG;
}


/************************************************************************/
iKinChain &iKinIpOptMin::specify2ndTaskEndEff(unsigned int n)
{
    chain2ndTask.clear();
    chain2ndTask.setH0(chain.getH0());

    for (unsigned int i=0; i<n; i++)
        chain2ndTask << chain[i];

    return chain2ndTask;
}


/************************************************************************/
void iKinIpOptMin::setTol(const Number tol)
{
    App->Options()->SetNumericValue("tol",tol);
    App->Options()->SetNumericValue("acceptable_tol",tol);

    App->Initialize();
}


/************************************************************************/
void iKinIpOptMin::setMaxIter(const Index max_iter)
{
    if (max_iter>0)
        App->Options()->SetIntegerValue("max_iter",max_iter);
    else
        App->Options()->SetIntegerValue("max_iter",(Index)2e9);

    App->Initialize();
}


/************************************************************************/
void iKinIpOptMin::setVerbosity(const unsigned int verbose)
{
    App->Options()->SetIntegerValue("print_level",verbose);

    App->Initialize();
}


/************************************************************************/
void iKinIpOptMin::setHessianOpt(const bool useHessian)
{
    if (useHessian)
        App->Options()->SetStringValue("hessian_approximation","exact");
    else
        App->Options()->SetStringValue("hessian_approximation","limited-memory");

    App->Initialize();
}


/************************************************************************/
void iKinIpOptMin::setUserScaling(const bool useUserScaling, Number _obj_scaling,
                                  Number _x_scaling, Number _g_scaling)
{
    if (useUserScaling)
    {
        obj_scaling=_obj_scaling;
        x_scaling  =_x_scaling;
        g_scaling  =_g_scaling;

        App->Options()->SetStringValue("nlp_scaling_method","user-scaling");
    }
    else
        App->Options()->SetStringValue("nlp_scaling_method","gradient-based");

    App->Initialize();
}


/************************************************************************/
void iKinIpOptMin::setDerivativeTest(const bool enableTest, const bool enable2ndDer)
{
    if (enableTest)
    {
        if (enable2ndDer)
            App->Options()->SetStringValue("derivative_test","second-order");
        else
            App->Options()->SetStringValue("derivative_test","first-order");

        App->Options()->SetStringValue("derivative_test_print_all","yes");
    }
    else
        App->Options()->SetStringValue("derivative_test","none");

    App->Initialize();
}


/************************************************************************/
void iKinIpOptMin::getBoundsInf(Number &lower, Number &upper)
{
    App->Options()->GetNumericValue("nlp_lower_bound_inf",lower,"");
    App->Options()->GetNumericValue("nlp_upper_bound_inf",upper,"");
}


/************************************************************************/
void iKinIpOptMin::setBoundsInf(Number lower, Number upper)
{
    App->Options()->SetNumericValue("nlp_lower_bound_inf",lower);
    App->Options()->SetNumericValue("nlp_upper_bound_inf",upper);

    lowerBoundInf=lower;
    upperBoundInf=upper;
}


/************************************************************************/
yarp::sig::Vector iKinIpOptMin::solve(const yarp::sig::Vector &q0, yarp::sig::Vector &xd,
                                      double weight2ndTask, yarp::sig::Vector &xd_2nd, yarp::sig::Vector &w_2nd,
                                      double weight3rdTask, yarp::sig::Vector &qd_3rd, yarp::sig::Vector &w_3rd,
                                      ApplicationReturnStatus *exit_code, bool *exhalt,
                                      iKinIterateCallback *iterate)
{
    SmartPtr<iKin_NLP> nlp=new iKin_NLP(chain,ctrlPose,q0,xd,
                                        weight2ndTask,chain2ndTask,xd_2nd,w_2nd,
                                        weight3rdTask,qd_3rd,w_3rd,
                                        *pLIC,exhalt);

    nlp->set_scaling(obj_scaling,x_scaling,g_scaling);
    nlp->set_bound_inf(lowerBoundInf,upperBoundInf);
    nlp->set_translational_tol(translationalTol);
    nlp->set_callback(iterate);

    ApplicationReturnStatus status=App->OptimizeTNLP(GetRawPtr(nlp));

    if (exit_code)
        *exit_code=status;

    return nlp->get_qd();
}


/************************************************************************/
ApplicationReturnStatus iKinIpOptMin::optimize(const SmartPtr<TNLP>& tnlp)
{
    return App->OptimizeTNLP(tnlp);
}


/************************************************************************/
ApplicationReturnStatus iKinIpOptMin::reoptimize(const SmartPtr<TNLP>& tnlp)
{
    return App->ReOptimizeTNLP(tnlp);
}


/************************************************************************/
iKinIpOptMin::~iKinIpOptMin()
{
    delete App;
}


