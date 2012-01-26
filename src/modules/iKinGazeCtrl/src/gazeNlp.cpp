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

#include <iCub/gazeNlp.h>
#include <iCub/utils.h>


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

    if (fabs(qty3)<ALMOST_ZERO)
        return false;

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
        for (int i=0; i<3; i++)
            H3(i,2)=AnaJacobZ_L(i,j)-dqty1*HR(i,2)-qty1*AnaJacobZ_R(i,j);
        dqty2=dot(H3,2,H1,3)+dot(H2L,2,GeoJacobP_L-GeoJacobP_R,j);
        dfp1=M.getCol(j)+Hz*((dqty2-2.0*qty1*qty2L*dqty1/qty3)/qty3);

        // derivative wrt pan left eye
        j=1;
        dqty1L=dot(HR,2,AnaJacobZ_L,j);
        for (int i=0; i<3; i++)
            H3(i,2)=AnaJacobZ_L(i,j)-dqty1*HR(i,2);
        dqty2L=dot(H3,2,H1,3)+dot(H2L,2,GeoJacobP_L,j);
        dfpL1=M.getCol(j)+Hz*((dqty2L-2.0*qty1*qty2L*dqty1L/qty3)/qty3);

        // derivative wrt pan right eye
        dqty1R=dot(AnaJacobZ_R,j,HL,2);
        for (int i=0; i<3; i++)
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
        for (int i=0; i<3; i++)
            H3(i,2)=qty1*AnaJacobZ_L(i,j)+dqty1*HL(i,2)-AnaJacobZ_R(i,j);
        dqty2=dot(H3,2,H1,3)+dot(H2R,2,GeoJacobP_L-GeoJacobP_R,j);
        dfp2=M.getCol(j)+Hz*((dqty2-2.0*qty1*qty2R*dqty1/qty3)/qty3);

        // derivative wrt pan left eye
        j=1;
        dqty1L=dot(HR,2,AnaJacobZ_L,j);
        for (int i=0; i<3; i++)
            H3(i,2)=qty1*AnaJacobZ_L(i,j)+dqty1L*HL(i,2);
        dqty2L=dot(H3,2,H1,3)+dot(H2R,2,GeoJacobP_L,j);
        dfpL2=Hz*((dqty2L-2.0*qty1*qty2R*dqty1L/qty3)/qty3);

        // derivative wrt pan right eye
        dqty1R=dot(AnaJacobZ_R,j,HL,2);
        for (int i=0; i<3; i++)
            H3(i,2)=dqty1R*HL(i,2)-AnaJacobZ_R(i,j);
        dqty2R=dot(H3,2,H1,3)+dot(H2R,2,-1.0*GeoJacobP_R,j);
        dfpR2=M.getCol(j)+Hz*((dqty2R-2.0*qty1*qty2R*dqty1R/qty3)/qty3);
    }

    for (int i=0; i<3; i++)
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

    return true;
}


/************************************************************************/
bool computeFixationPointOnly(iKinChain &eyeL, iKinChain &eyeR, Vector &fp)
{
    Matrix HL=eyeL.getH();
    Matrix HR=eyeR.getH();
    HL(3,3)=HR(3,3)=0.0;

    double qty1=dot(HR,2,HL,2);
    Matrix H1=HL-HR;
    Matrix H2L=HL-qty1*HR;
    Matrix H2R=qty1*HL-HR;
    double qty2L=dot(H2L,2,H1,3);
    double qty2R=dot(H2R,2,H1,3);
    double qty3=qty1*qty1-1.0;

    if (fabs(qty3)<ALMOST_ZERO)
        return false;

    double tL=qty2L/qty3;
    double tR=qty2R/qty3;

    for (int i=0; i<3; i++)
        fp[i]=0.5*(HL(i,3)+tL*HL(i,2)+HR(i,3)+tR*HR(i,2));

    return true;
}


/************************************************************************/
void iCubHeadCenter::allocate(const string &_type)
{
    // change DH parameters
    (*this)[getN()-2].setD(0.0);

    // block last two links
    blockLink(getN()-2,0.0);
    blockLink(getN()-1,0.0);
}


/************************************************************************/
void HeadCenter_NLP::setGravityDirection(const Vector &gDir)
{
    // we assume that gDir is provided in homogeneous form
    // rest pitch
    Vector gDirHp=SE3inv(chain.getH(2,true))*gDir;
    qRest[0]=-atan2(gDirHp[0],gDirHp[1]);
    qRest[0]=qRest[0]>chain(0).getMax()?chain(0).getMax():
             (qRest[0]<chain(0).getMin()?chain(0).getMin():qRest[0]);

    // rest roll
    Vector gDirHr=SE3inv(chain.getH(3,true))*gDir;
    qRest[1]=atan2(gDirHr[1],gDirHr[0]);
    qRest[1]=qRest[1]>chain(1).getMax()?chain(1).getMax():
             (qRest[1]<chain(1).getMin()?chain(1).getMin():qRest[1]);
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

        double offset=5.0*CTRL_DEG2RAD;
        double delta=1.0*CTRL_DEG2RAD;
        double pitch_cog=chain(0).getMin()+offset+delta/2.0;
        double c=10.0/delta;
        double _tanh=tanh(c*(q[0]-pitch_cog));

        // transition function and its first derivative
        // to block the roll around qRest[1] when the pitch
        // approaches its minimum
        fPitch=0.5*(1.0+_tanh);
        dfPitch=0.5*c*(1.0-_tanh*_tanh);
        
        GeoJacobP=chain.GeoJacobian();
        AnaJacobZ=chain.AnaJacobian(2);
    }
}


/************************************************************************/
bool HeadCenter_NLP::get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                                  Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    n=dim;
    m=3;
    nnz_jac_g=n+2*(n-1);
    nnz_h_lag=0;
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

    g_l[0]=g_l[1]=g_l[2]=lowerBoundInf;
    g_u[0]=-1.0+translationalTol;
    g_u[1]=g_u[2]=0.0;

    return true;
}


/************************************************************************/
bool HeadCenter_NLP::eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                            Ipopt::Number& obj_value)
{
    obj_value=0.0;

    for (Ipopt::Index i=0; i<n; i++)
    {
        double tmp=x[i]-qRest[i];
        obj_value+=tmp*tmp;
    }

    obj_value*=0.5;

    return true;
}


/************************************************************************/
bool HeadCenter_NLP::eval_grad_f(Ipopt::Index n, const Ipopt::Number* x,
                                 bool new_x, Ipopt::Number* grad_f)
{
    for (Ipopt::Index i=0; i<n; i++)
        grad_f[i]=x[i]-qRest[i];

    return true;
}


/************************************************************************/
bool HeadCenter_NLP::eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                            Ipopt::Index m, Ipopt::Number* g)
{
    computeQuantities(x);

    g[0]=cosAng;
    g[1]=(chain(1).getMin()-qRest[1])*fPitch-(x[1]-qRest[1]);
    g[2]=x[1]-qRest[1]-(chain(1).getMax()-qRest[1])*fPitch;

    return true;
}


/************************************************************************/
bool HeadCenter_NLP::eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                                Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow,
                                Ipopt::Index *jCol, Ipopt::Number* values)
{
    if (!values)
    {
        iRow[0]=0; jCol[0]=0;
        iRow[1]=0; jCol[1]=1;
        iRow[2]=0; jCol[2]=2;

        iRow[3]=1; jCol[3]=0;
        iRow[4]=1; jCol[4]=1;

        iRow[5]=2; jCol[5]=0;
        iRow[6]=2; jCol[6]=1;
    }
    else
    {
        computeQuantities(x);

        // dg[0]/dxi
        for (Ipopt::Index i=0; i<n; i++)
            values[i]=(dot(AnaJacobZ,i,Hxd,3)+dot(Hxd,2,GeoJacobP,i))/mod
                      -(cosAng*dot(Hxd,3,GeoJacobP,i))/(mod*mod);

        // dg[1]/dPitch
        values[3]=(chain(1).getMin()-qRest[1])*dfPitch;

        // dg[1]/dRoll
        values[4]=-1.0;

        // dg[2]/dPitch
        values[5]=-(chain(1).getMax()-qRest[1])*dfPitch;

        // dg[2]/dRoll
        values[6]=1.0;
    }

    return true;
}


/************************************************************************/
Vector GazeIpOptMin::solve(const Vector &q0, Vector &xd, const Vector &gDir,
                           Ipopt::ApplicationReturnStatus *exit_code, bool *exhalt,
                           iKinIterateCallback *iterate)
{
    Ipopt::SmartPtr<HeadCenter_NLP> nlp;
    nlp=new HeadCenter_NLP(chain,q0,xd,*pLIC,exhalt);

    nlp->set_scaling(obj_scaling,x_scaling,g_scaling);
    nlp->set_bound_inf(lowerBoundInf,upperBoundInf);
    nlp->set_translational_tol(translationalTol);
    nlp->set_callback(iterate);
    nlp->setGravityDirection(gDir);
    
    Ipopt::ApplicationReturnStatus status=optimize(GetRawPtr(nlp));

    if (exit_code!=NULL)
        *exit_code=status;

    return nlp->get_qd();
}


