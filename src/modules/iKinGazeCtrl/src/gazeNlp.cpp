
#include <iCub/gazeNlp.hpp>


/************************************************************************/
void iCubHeadCenter::allocate(const string &_type)
{
    // change DH parameters
    linkList[getN()-2]->setD(0.0);

    // block last two links
    blockLink(getN()-2,0.0);
    blockLink(getN()-1,0.0);

    // disable neck roll
    blockLink(4,0.0);
}


/************************************************************************/
void HeadCenter_NLP::computeQuantities(const Ipopt::Number *x)
{
    Vector new_q(dim);

    for (Ipopt::Index i=0; i<(int)dim; i++)
        new_q[i]=x[i];

    if (!(q==new_q) || firstGo)
    {
        firstGo=false;
        q=new_q;

        q=chain.setAng(q);
        Hxd=chain.getH();
        Hxd(0,3)-=xd[0];
        Hxd(1,3)-=xd[1];
        Hxd(2,3)-=xd[2];
        Hxd(3,3)=0.0;
        mod=norm(Hxd,3);
        cosAng=dot(Hxd,2,Hxd,3)/mod;
        
        GeoJacobP=chain.GeoJacobian();
        AnaJacobZ=chain.AnaJacobian(2);
    }
}


/************************************************************************/
bool HeadCenter_NLP::get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                                  Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    n=dim;
    m=nnz_jac_g=nnz_h_lag=0;        
    index_style=TNLP::C_STYLE;
    
    return true;
}


/************************************************************************/
bool HeadCenter_NLP::get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                                     Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u)
{
    for (Ipopt::Index i=0; i<n; i++)
    {
        x_l[i]=chain(i).getMin();
        x_u[i]=chain(i).getMax();
    }

    return true;
}


/************************************************************************/
bool HeadCenter_NLP::eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                            Ipopt::Number& obj_value)
{
    computeQuantities(x);

    obj_value=cosAng+1.0;

    return true;
}


/************************************************************************/
bool HeadCenter_NLP::eval_grad_f(Ipopt::Index n, const Ipopt::Number* x,
                                 bool new_x, Ipopt::Number* grad_f)
{
    computeQuantities(x);

    for (Ipopt::Index i=0; i<n; i++)
        grad_f[i]=(dot(AnaJacobZ,i,Hxd,3)+dot(Hxd,2,GeoJacobP,i))/mod
                  -(cosAng*dot(Hxd,3,GeoJacobP,i))/(mod*mod);

    return true;
}


/************************************************************************/
bool computeFixationPointData(iKinChain &eyeL, iKinChain &eyeR, Vector &fp, Matrix &J)
{
    Vector dfp1(4), dfp2(4);
    Vector dfpL1(4),dfpL2(4);
    Vector dfpR1(4),dfpR2(4);

    Matrix HL=eyeL.getH();
    Matrix HR=eyeR.getH();
    HL(3,3)=HR(3,3)=0.0;

    double qty1=dot(HR,2,HL,2);
    Matrix H1=HL-HR;
    Matrix H2L=HL-qty1*HR;
    Matrix H2R=qty1*HL-HR;
    Matrix H3(4,4); H3(3,2)=0.0;
    double qty2L=dot(H2L,2,H1,3);
    double qty2R=dot(H2R,2,H1,3);
    double qty3=qty1*qty1-1.0;

    if (!qty3)
        return true;

    double tL=qty2L/qty3;
    double tR=qty2R/qty3;

    Matrix GeoJacobP_L=eyeL.GeoJacobian();
    Matrix GeoJacobP_R=eyeR.GeoJacobian();
    Matrix AnaJacobZ_L=eyeL.AnaJacobian(2);
    Matrix AnaJacobZ_R=eyeR.AnaJacobian(2);

    // Left part
    {
        double dqty1, dqty1L, dqty1R;
        double dqty2, dqty2L, dqty2R;
        int    j;

        Vector Hz=HL.getCol(2);
        Matrix M=GeoJacobP_L.submatrix(0,3,0,1)+tL*AnaJacobZ_L.submatrix(0,3,0,1);

        // derivative wrt eye tilt
        j=0;
        dqty1=dot(AnaJacobZ_R,j,HL,2)+dot(HR,2,AnaJacobZ_L,j);
        for (unsigned int i=0; i<3; i++)
            H3(i,2)=AnaJacobZ_L(i,j)-dqty1*HR(i,2)-qty1*AnaJacobZ_R(i,j);
        dqty2=dot(H3,2,H1,3)+dot(H2L,2,GeoJacobP_L-GeoJacobP_R,j);
        dfp1=M.getCol(j)+Hz*((dqty2-2.0*qty1*qty2L*dqty1/qty3)/qty3);

        // derivative wrt pan left eye
        j=1;
        dqty1L=dot(HR,2,AnaJacobZ_L,j);
        for (unsigned int i=0; i<3; i++)
            H3(i,2)=AnaJacobZ_L(i,j)-dqty1*HR(i,2);
        dqty2L=dot(H3,2,H1,3)+dot(H2L,2,GeoJacobP_L,j);
        dfpL1=M.getCol(j)+Hz*((dqty2L-2.0*qty1*qty2L*dqty1L/qty3)/qty3);

        // derivative wrt pan right eye
        dqty1R=dot(AnaJacobZ_R,j,HL,2);
        for (unsigned int i=0; i<3; i++)
            H3(i,2)=-dqty1*HR(i,2)-qty1*AnaJacobZ_R(i,j);
        dqty2R=dot(H3,2,H1,3)+dot(H2L,2,-1.0*GeoJacobP_R,j);
        dfpR1=Hz*((dqty2R-2.0*qty1*qty2L*dqty1R/qty3)/qty3);
    }

    // Right part
    {
        double dqty1, dqty1L, dqty1R;
        double dqty2, dqty2L, dqty2R;
        int    j;

        Vector Hz=HR.getCol(2);
        Matrix M=GeoJacobP_R.submatrix(0,3,0,1)+tR*AnaJacobZ_R.submatrix(0,3,0,1);

        // derivative wrt eye tilt
        j=0;
        dqty1=dot(AnaJacobZ_R,j,HL,2)+dot(HR,2,AnaJacobZ_L,j);
        for (unsigned int i=0; i<3; i++)
            H3(i,2)=qty1*AnaJacobZ_L(i,j)+dqty1*HL(i,2)-AnaJacobZ_R(i,j);
        dqty2=dot(H3,2,H1,3)+dot(H2R,2,GeoJacobP_L-GeoJacobP_R,j);
        dfp2=M.getCol(j)+Hz*((dqty2-2.0*qty1*qty2R*dqty1/qty3)/qty3);

        // derivative wrt pan left eye
        j=1;
        dqty1L=dot(HR,2,AnaJacobZ_L,j);
        for (unsigned int i=0; i<3; i++)
            H3(i,2)=qty1*AnaJacobZ_L(i,j)+dqty1L*HL(i,2);
        dqty2L=dot(H3,2,H1,3)+dot(H2R,2,GeoJacobP_L,j);
        dfpL2=Hz*((dqty2L-2.0*qty1*qty2R*dqty1L/qty3)/qty3);

        // derivative wrt pan right eye
        dqty1R=dot(AnaJacobZ_R,j,HL,2);
        for (unsigned int i=0; i<3; i++)
            H3(i,2)=dqty1R*HL(i,2)-AnaJacobZ_R(i,j);
        dqty2R=dot(H3,2,H1,3)+dot(H2R,2,-1.0*GeoJacobP_R,j);
        dfpR2=M.getCol(j)+Hz*((dqty2R-2.0*qty1*qty2R*dqty1R/qty3)/qty3);
    }

    for (unsigned int i=0; i<3; i++)
    {
        // fixation point position
        fp[i]=0.5*(HL(i,3)+tL*HL(i,2)+HR(i,3)+tR*HR(i,2));

        // Jacobian
        // r=p-v/2, l=p+v/2;
        // dfp/dp=dfp/dl*dl/dp + dfp/dr*dr/dp = dfp/dl + dfp/dr;
        // dfp/dv=dfp/dl*dl/dv + dfp/dr*dr/dv = (dfp/dl - dfp/dr)/2;
        J(i,0)=0.50*(dfp1[i]           + dfp2[i]);              // tilt
        J(i,1)=0.50*(dfpL1[i]+dfpR1[i] + dfpL2[i]+dfpR2[i]);    // pan
        J(i,2)=0.25*(dfpL1[i]-dfpR1[i] + dfpL2[i]-dfpR2[i]);    // vergence
    }

    return false;
}


/************************************************************************/
bool computeFixationPointOnly(iKinChain &eyeL, iKinChain &eyeR, Vector &fp)
{
    Vector dfp1(4), dfp2(4);
    Vector dfpL1(4),dfpL2(4);
    Vector dfpR1(4),dfpR2(4);

    Matrix HL=eyeL.getH();
    Matrix HR=eyeR.getH();
    HL(3,3)=HR(3,3)=0.0;

    double qty1=dot(HR,2,HL,2);
    Matrix H1=HL-HR;
    Matrix H2L=HL-qty1*HR;
    Matrix H2R=qty1*HL-HR;
    Matrix H3(4,4); H3(3,2)=0.0;
    double qty2L=dot(H2L,2,H1,3);
    double qty2R=dot(H2R,2,H1,3);
    double qty3=qty1*qty1-1.0;

    if (!qty3)
        return true;

    double tL=qty2L/qty3;
    double tR=qty2R/qty3;

    for (unsigned int i=0; i<3; i++)
        fp[i]=0.5*(HL(i,3)+tL*HL(i,2)+HR(i,3)+tR*HR(i,2));

    return false;
}


/************************************************************************/
Eyes_NLP::Eyes_NLP(iKinChain &_eyeL, iKinChain &_eyeR, const Vector &_q0, Vector &_xd,
                   iKinLinIneqConstr &_LIC, bool *_exhalt) :
                   iKin_NLP(_eyeL,IKINCTRL_POSE_XYZ,_q0,_xd,
                   0.0,dummyChain,dummyVector,dummyVector,
                   0.0,dummyVector,dummyVector,
                   _LIC,_exhalt), eyeL(_eyeL), eyeR(_eyeR)
{
    // tilt+pan+vergence
    dim=3;

    // prevent from starting with verg==0.0
    subsStartVerg0=5.0;

    qd.resize(dim);
    size_t n=q0.length();
    n=n>dim ? dim : n;

    unsigned int i;
    for (i=0; i<n; i++)
        qd[i]=q0[i];

    for (; i<dim; i++)
        qd[i]=0.0;

    if (!qd[dim-1])
        qd[dim-1]=subsStartVerg0;

    q=qd;

    qL.resize(2);
    qR.resize(2);
    fp.resize(3);
    J.resize(3,3);
}


/************************************************************************/
void Eyes_NLP::reinforceBounds()
{
    qL[0]=qR[0]=q[0];
    qL[1]=q[1]+q[2]/2.0;
    qR[1]=q[1]-q[2]/2.0;

    qL=eyeL.setAng(qL);
    qR=eyeR.setAng(qR);

    q[0]=(qL[0]+qR[0]) / 2.0;   // tilt
    q[1]=(qL[1]+qR[1]) / 2.0;   // pan
    q[2]=qL[1]-qR[1];           // vergence
}


/************************************************************************/
void Eyes_NLP::computeQuantities(const Ipopt::Number *x)
{
    Vector new_q(dim);

    for (Ipopt::Index i=0; i<(int)dim; i++)
        new_q[i]=x[i];

    if (!(q==new_q) || firstGo)
    {
        firstGo=false;
        q=new_q;

        reinforceBounds();
        divByZero=computeFixationPointData(eyeL,eyeR,fp,J);

        e_xyz=xd-fp;
    }
}


/************************************************************************/
bool Eyes_NLP::get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                            Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    n=dim;
    m=2;
    nnz_jac_g=m*(n-1);
    nnz_h_lag=0;
    index_style=TNLP::C_STYLE;
    
    return true;
}


/************************************************************************/
bool Eyes_NLP::get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                               Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u)
{
    // tilt bounds
    x_l[0]=eyeL(0).getMin();
    x_u[0]=eyeL(0).getMax();

    // pan bounds
    x_l[1]=eyeL(1).getMin();
    x_u[1]=eyeL(1).getMax();

    // vergence bounds
    x_l[2]=0.0;
    x_u[2]=eyeL(1).getMax();

    g_l[0]=g_l[1]=x_l[1];   // minimum joint angle m
    g_u[0]=g_u[1]=x_u[1];   // Maximum joint angle M

    return true;
}


/************************************************************************/
bool Eyes_NLP::get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                  bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                  Ipopt::Index m, bool init_lambda,
                                  Ipopt::Number* lambda)
{
    for (Ipopt::Index i=0; i<n; i++)
        x[i]=q0[i];

    if (!x[n-1])
        x[n-1]=subsStartVerg0;

    return true;
}


/************************************************************************/
bool Eyes_NLP::eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                      Ipopt::Number& obj_value)
{
    computeQuantities(x);

    if (divByZero)
        return false;

    obj_value=0.5*norm2(e_xyz);

    return true;
}


/************************************************************************/
bool Eyes_NLP::eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                           Ipopt::Number* grad_f)
{
    computeQuantities(x);

    if (divByZero)
        return false;

    for (Ipopt::Index i=0; i<n; i++)
        grad_f[i]=-dot(e_xyz,J.getCol(i));

    return true;
}


/************************************************************************/
bool Eyes_NLP::eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                      Ipopt::Index m, Ipopt::Number* g)
{
    computeQuantities(x);

    if (divByZero)
        return false;

    g[0]=x[1]+x[2]/2.0; // l=pan+vergence/2 in [m,M]
    g[1]=x[1]-x[2]/2.0; // r=pan-vergence/2 in [m,M]

    return true;
}


/************************************************************************/
bool Eyes_NLP::eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                          Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow,
                          Ipopt::Index *jCol, Ipopt::Number* values)
{
    if (!values)
    {
        Ipopt::Index idx=0;

        for (Ipopt::Index row=0; row<m; row++)
            for (Ipopt::Index col=1; col<n; col++) // irrespective of tilt
            {
                iRow[idx]=row;
                jCol[idx]=col;
                idx++;
            }
    }
    else
    {
        computeQuantities(x);

        if (divByZero)
            return false;

        values[0]= 1.0;
        values[1]= 0.5;
        values[2]= 1.0;
        values[3]=-0.5;
    }

    return true;
}


/************************************************************************/
void Eyes_NLP::finalize_solution(Ipopt::SolverReturn status,
                                 Ipopt::Index n, const Ipopt::Number* x,
                                 const Ipopt::Number* z_L, const Ipopt::Number* z_U,
                                 Ipopt::Index m, const Ipopt::Number* g,
                                 const Ipopt::Number* lambda, Ipopt::Number obj_value,
                                 const Ipopt::IpoptData* ip_data,
                                 Ipopt::IpoptCalculatedQuantities* ip_cq)
{
    for (Ipopt::Index i=0; i<n; i++)
        q[i]=x[i];

    reinforceBounds();
    qd=q;
}


/************************************************************************/
GazeIpOptMin::GazeIpOptMin(const string &type, iKinChain &c1, iKinChain &c2,
                           const double tol, const int max_iter,
                           const unsigned int verbose) :
                           iKinIpOptMin(c1,IKINCTRL_POSE_XYZ,tol,max_iter,verbose,false),
                           chain2(c2)
{
    neckType=type=="neck" ? true : false;

    if (!neckType)
        chain2.setAllConstraints(false);
}


/************************************************************************/
Vector GazeIpOptMin::solve(const Vector &q0, Vector &xd,
                           Ipopt::ApplicationReturnStatus *exit_code, bool *exhalt,
                           iKinIterateCallback *iterate)
{
    Ipopt::SmartPtr<iKin_NLP> nlp;

    if (neckType)
        nlp=new HeadCenter_NLP(chain,q0,xd,*pLIC,exhalt);
    else
        nlp=new Eyes_NLP(chain,chain2,q0,xd,*pLIC,exhalt);

    nlp->set_scaling(obj_scaling,x_scaling,g_scaling);
    nlp->set_bound_inf(lowerBoundInf,upperBoundInf);
    nlp->set_callback(iterate);
    
    Ipopt::ApplicationReturnStatus status=optimize(GetRawPtr(nlp));

    if (exit_code)
        *exit_code=status;

    return nlp->get_qd();
}


