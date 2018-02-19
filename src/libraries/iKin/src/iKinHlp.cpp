/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * Copyright (C) 2006-2010 RobotCub Consortium
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD-3-Clause license. See the accompanying LICENSE file for
 * details.
*/

#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinVocabs.h>
#include <iCub/iKin/iKinInv.h>
#include <iCub/iKin/iKinHlp.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


/************************************************************************/
void CartesianHelper::addVectorOption(Bottle &b, const int vcb, const Vector &v)
{
    Bottle &part=b.addList();
    part.addVocab(vcb);
    Bottle &vect=part.addList();

    for (size_t i=0; i<v.length(); i++)
        vect.addDouble(v[i]);
}


/************************************************************************/
bool CartesianHelper::getDesiredOption(const Bottle &reply, Vector &xdhat,
                                       Vector &odhat, Vector &qdhat)
{
    if (reply.size()==0)
        return false;

    if (reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK)
    {
        // xdhat and odhat part
        if (reply.check(Vocab::decode(IKINSLV_VOCAB_OPT_X)))
        {
            Bottle *xData=getEndEffectorPoseOption(reply);
            if (xData->size()==0)
                return false;

            xdhat.resize(3);
            for (size_t i=0; i<xdhat.length(); i++)
                xdhat[i]=xData->get(i).asDouble();

            odhat.resize(4);
            for (size_t i=0; i<odhat.length(); i++)
                odhat[i]=xData->get(xdhat.length()+i).asDouble();
        }
        else
            return false;

        // qdhat part
        if (reply.check(Vocab::decode(IKINSLV_VOCAB_OPT_Q)))
        {
            Bottle *qData=getJointsOption(reply);
            if (qData->size()==0)
                return false;

            qdhat.resize(qData->size());
            for (size_t i=0; i<qdhat.length(); i++)
                qdhat[i]=qData->get(i).asDouble();
        }
        else
            return false;

        return true;
    }
    else
        return false;
}


/************************************************************************/
void CartesianHelper::addTargetOption(Bottle &b, const Vector &xd)
{
    addVectorOption(b,IKINSLV_VOCAB_OPT_XD,xd);
}


/************************************************************************/
void CartesianHelper::addDOFOption(Bottle &b, const Vector &dof)
{
    addVectorOption(b,IKINSLV_VOCAB_OPT_DOF,dof);
}


/************************************************************************/
void CartesianHelper::addJointsResPosOption(Bottle &b, const Vector &restPos)
{
    addVectorOption(b,IKINSLV_VOCAB_OPT_REST_POS,restPos);
}


/************************************************************************/
void CartesianHelper::addJointsRestWeightsOption(Bottle &b, const Vector &restWeights)
{
    addVectorOption(b,IKINSLV_VOCAB_OPT_REST_WEIGHTS,restWeights);
}


/************************************************************************/
void CartesianHelper::addPoseOption(Bottle &b, const unsigned int pose)
{
    Bottle &posePart=b.addList();
    posePart.addVocab(IKINSLV_VOCAB_OPT_POSE);

    if (pose==IKINCTRL_POSE_FULL)
        posePart.addVocab(IKINSLV_VOCAB_VAL_POSE_FULL);
    else if (pose==IKINCTRL_POSE_XYZ)
        posePart.addVocab(IKINSLV_VOCAB_VAL_POSE_XYZ);
}


/************************************************************************/
void CartesianHelper::addModeOption(Bottle &b, const bool tracking)
{
    Bottle &modePart=b.addList();
    modePart.addVocab(IKINSLV_VOCAB_OPT_MODE);

    if (tracking)
        modePart.addVocab(IKINSLV_VOCAB_VAL_MODE_TRACK);
    else
        modePart.addVocab(IKINSLV_VOCAB_VAL_MODE_SINGLE);
}


/************************************************************************/
void CartesianHelper::addTokenOption(Bottle &b, const double token)
{
    Bottle &tokenPart=b.addList();
    tokenPart.addVocab(IKINSLV_VOCAB_OPT_TOKEN);
    tokenPart.addDouble(token);
}


/************************************************************************/
Bottle *CartesianHelper::getTargetOption(const Bottle &b)
{
    return b.find(Vocab::decode(IKINSLV_VOCAB_OPT_XD)).asList();
}


/************************************************************************/
Bottle *CartesianHelper::getEndEffectorPoseOption(const Bottle &b)
{
    return b.find(Vocab::decode(IKINSLV_VOCAB_OPT_X)).asList();
}


/************************************************************************/
Bottle *CartesianHelper::getJointsOption(const Bottle &b)
{
    return b.find(Vocab::decode(IKINSLV_VOCAB_OPT_Q)).asList();
}


/************************************************************************/
bool CartesianHelper::getTokenOption(const Bottle &b, double *token)
{
    if (b.check(Vocab::decode(IKINSLV_VOCAB_OPT_TOKEN)) && (token!=NULL))
    {
        *token=b.find(Vocab::decode(IKINSLV_VOCAB_OPT_TOKEN)).asDouble();
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool CartesianHelper::computeFixationPointData(iKinChain &eyeL,
                                               iKinChain &eyeR,
                                               Vector &fp)
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

    if (fabs(qty3)<IKIN_ALMOST_ZERO)
        return false;

    double tL=qty2L/qty3;
    double tR=qty2R/qty3;

    if (fp.length()!=3)
        fp.resize(3);

    for (int i=0; i<3; i++)
        fp[i]=0.5*(HL(i,3)+tL*HL(i,2)+HR(i,3)+tR*HR(i,2));

    return true;
}


/************************************************************************/
bool CartesianHelper::computeFixationPointData(iKinChain &eyeL,
                                               iKinChain &eyeR,
                                               Vector &fp, Matrix &J)
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

    if (fabs(qty3)<IKIN_ALMOST_ZERO)
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

    if (fp.length()!=3)
        fp.resize(3);

    if ((J.rows()!=3) || (J.cols()!=3))
        J.resize(3,3);

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


