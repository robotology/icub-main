/*
* Copyright (C) 2010-2011 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Serena Ivaldi, Matteo Fumagalli
* email:   serena.ivaldi@iit.it, matteo.fumagalli@iit.it
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

#include <cstdio>
#include <iostream>
#include <iomanip>
#include <cmath>

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace iCub::iDyn;
using namespace iCub::skinDynLib;

// #define DEBUG_FOOT_COM
//====================================
//
//      RIGID BODY TRANSFORMATION
//
//====================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
RigidBodyTransformation::RigidBodyTransformation(iDynLimb *_limb, const yarp::sig::Matrix &_H, const string &_info, bool _hasSensor, const FlowType kin, const FlowType wre, const NewEulMode _mode, unsigned int verb)
{
    limb = _limb;
    kinFlow = kin;
    wreFlow = wre;
    mode = _mode;
    info = _info;
    hasSensor=_hasSensor;
    F.resize(3);    F.zero();
    Mu.resize(3);   Mu.zero();
    w.resize(3);    w.zero();
    dw.resize(3);   dw.zero();
    ddp.resize(3);  ddp.zero();
    H.resize(4,4); H.eye();
    setRBT(_H);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
RigidBodyTransformation::~RigidBodyTransformation()
{
    // never delete the limb!! only stop pointing at it
    limb = NULL;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool RigidBodyTransformation::setRBT(const Matrix &_H)
{
    if((_H.cols()==4)&&(_H.rows()==4))
    {
        H=_H;
        return true;
    }
    else
    {
        if(verbose) fprintf(stderr,"RigidBodyTransformation: could not set RBT due to wrong sized matrix H: %zu,%zu instead of (4,4). Setting identity as default. \n",_H.cols(),_H.rows());
        H.resize(4,4);
        H.eye();
        return false;
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool RigidBodyTransformation::setKinematic(const Vector &wNode, const Vector &dwNode, const Vector &ddpNode)
{
    // set the RBT kinematic variables to the ones of the node
    w = wNode;
    dw = dwNode;
    ddp = ddpNode;
    //now apply the RBT transformation
    computeKinematic();
    // send the kinematic information to the limb - the limb knows whether it is on base or on end-eff
    return limb->initKinematicNewtonEuler(w,dw,ddp);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool RigidBodyTransformation::setKinematicMeasure(const Vector &w0, const Vector &dw0, const Vector &ddp0)
{
    return limb->initKinematicNewtonEuler(w0,dw0,ddp0);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool RigidBodyTransformation::setWrench(const Vector &FNode, const Vector &MuNode)
{
    //set the RBT force/moment with the one of the node
    F = FNode;
    Mu = MuNode;
    // now apply the RBT transformation
    computeWrench();
    // send the wrench to the limb - the limb knows whether it is on base or on end-eff
    return limb->initWrenchNewtonEuler(F,Mu);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool RigidBodyTransformation::setWrenchMeasure(const Vector &F0, const Vector &Mu0)
{
    return limb->initWrenchNewtonEuler(F0,Mu0);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool RigidBodyTransformation::setWrenchMeasure(iDynSensor *sensor, const Vector &Fsens, const Vector &Musens)
{
    return sensor->setSensorMeasures(Fsens,Musens);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix RigidBodyTransformation::getRBT() const
{
    return H;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix RigidBodyTransformation::getR()
{
    return H.submatrix(0,2,0,2);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix RigidBodyTransformation::getR6() const
{
    Matrix R(6,6); R.zero();
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            R(i,j) = H(i,j);
            R(i+3,j+3) = R(i,j);
        }
    }
    return R;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector RigidBodyTransformation::getr(bool proj)
{
    if(proj==false)
        return H.submatrix(0,2,0,3).getCol(3);
    return getR().transposed() * H.getCol(3).subVector(0,2);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void RigidBodyTransformation::getKinematic(Vector &wNode, Vector &dwNode, Vector &ddpNode)
{
    //read w,dw,ddp from the limb and stores them into the RBT variables
    limb->getKinematicNewtonEuler(w,dw,ddp);

    //now compute according to transformation
    computeKinematic();

    //apply the kinematics computations to the node
    wNode = w;
    dwNode = dw;
    ddpNode = ddp;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void RigidBodyTransformation::getWrench(Vector &FNode, Vector &MuNode)
{
    //read F,Mu from the limb and stores them into the RBT variables
    limb->getWrenchNewtonEuler(F,Mu);

    //now compute according to transformation
    computeWrench();

    //apply the wrench computations to the node
    FNode = FNode + F;
    MuNode = MuNode + Mu;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void RigidBodyTransformation::setInfoFlow(const FlowType kin, const FlowType wre)
{
    kinFlow=kin;
    wreFlow=wre;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
FlowType RigidBodyTransformation::getKinematicFlow() const
{
    return kinFlow;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
FlowType RigidBodyTransformation::getWrenchFlow() const
{
    return wreFlow;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void RigidBodyTransformation::computeKinematic()
{
    if(this->kinFlow== RBT_NODE_IN)
    {
        // note: these computations are similar to the Backward ones in
        // OneLinkNewtonEuler: compute AngVel/AngAcc/LinAcc Backward
        // but here are fastened and adapted to the RBT
        // note: w,dw,ddp are already set with the ones coming from the LIMB

        switch(mode)
        {
        case DYNAMIC:
        case DYNAMIC_CORIOLIS_GRAVITY:
        case DYNAMIC_W_ROTOR:
            ddp = getR() * ( ddp - cross(dw,getr(true)) - cross(w,cross(w,getr(true))) ) ;
            w   = getR() * w ;
            dw  = getR() * dw ;
            break;
        case STATIC:
            w   = 0.0;
            dw  = 0.0;
            ddp = getR() * ddp;
            break;
        }
    }
    else
    {
        // note: these computations are similar to the Forward ones in
        // OneLinkNewtonEuler: compute AngVel/AngAcc/LinAcc
        // but here are fastened and adapted to the RBT
        // note: w,dw,ddp are already set with the ones coming from the NODE

        switch(mode)
        {
        case DYNAMIC:
        case DYNAMIC_CORIOLIS_GRAVITY:
        case DYNAMIC_W_ROTOR:
            w   = getR().transposed() * w ;
            dw  = getR().transposed() * dw;
            ddp = getR().transposed() * (ddp) + cross(dw,getr(true)) + cross(w,cross(w,getr(true))) ;
            break;
        case STATIC:
            w   = 0.0;
            dw  = 0.0;
            ddp =  getR().transposed() * ddp ;
            break;
        }

    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void RigidBodyTransformation::computeWrench()
{
    if(this->wreFlow== RBT_NODE_IN)
    {
        // note: these computations are similar to the Backward ones in
        // OneLinkNewtonEuler: compute Force/Moment Backward
        // but here are fastened and adapted to the RBT
        // note: no switch(mode) is necessary because all modes have the same formula
        // note: F,Mu are already set with the ones coming from the LIMB

        Mu = cross( getr(), getR()*F ) + getR() * Mu ;
        F  = getR() * F;
    }
    else
    {
        // note: these computations are similar to the Forward ones in
        // OneLinkNewtonEuler: compute Force/Moment Forward
        // but here are fastened and adapted to the RBT
        // note: no switch(mode) is necessary because all modes have the same formula
        // note: F,Mu are already set with the ones coming from the NODE

        Mu = getR().transposed() * ( Mu - cross(getr(),getR() * F ));
        F  = getR().transposed() * F ;

    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool RigidBodyTransformation::isSensorized() const
{
    return hasSensor;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void RigidBodyTransformation::computeLimbKinematic()
{
    limb->computeKinematicNewtonEuler();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void RigidBodyTransformation::computeLimbWrench()
{
    limb->computeWrenchNewtonEuler();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
unsigned int RigidBodyTransformation::getNLinks() const
{
    return limb->getN();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
unsigned int RigidBodyTransformation::getDOF() const
{
    return limb->getDOF();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix RigidBodyTransformation::getH(const unsigned int iLink, const bool allLink)
{
    return limb->getH(iLink,allLink);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix RigidBodyTransformation::getH()
{
    return limb->getH();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector RigidBodyTransformation::getEndEffPose(const bool axisRep)
{
    return limb->EndEffPose(axisRep);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix RigidBodyTransformation::computeGeoJacobian(const unsigned int iLink, const Matrix &Pn, bool rbtRoto)
{
    if(rbtRoto==false)
        return limb->computeGeoJacobian(iLink,Pn);
    else
        return getR6() * limb->computeGeoJacobian(iLink,Pn);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix RigidBodyTransformation::computeGeoJacobian(const unsigned int iLink, const Matrix &Pn, const Matrix &H0, bool rbtRoto)
{
    if(rbtRoto==false)
        return limb->computeGeoJacobian(iLink,Pn,H0);
    else
        return getR6() * limb->computeGeoJacobian(iLink,Pn,H0);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix RigidBodyTransformation::computeGeoJacobian(const Matrix &Pn, bool rbtRoto)
{
    if(rbtRoto==false)
        return limb->computeGeoJacobian(Pn);
    else
        return getR6() * limb->computeGeoJacobian(Pn);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix RigidBodyTransformation::computeGeoJacobian(const Matrix &Pn, const Matrix &H0, bool rbtRoto)
{
    if(rbtRoto==false)
        return limb->computeGeoJacobian(Pn,H0);
    else
        return getR6() * limb->computeGeoJacobian(Pn,H0);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix RigidBodyTransformation::computeGeoJacobian(bool rbtRoto)
{
    if(rbtRoto==false)
        return limb->GeoJacobian();
    else
        return getR6() * limb->GeoJacobian();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix RigidBodyTransformation::computeGeoJacobian(const unsigned int iLink, bool rbtRoto)
{
    if(rbtRoto==false)
        return limb->GeoJacobian(iLink);
    else
        return getR6() * limb->GeoJacobian(iLink);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix RigidBodyTransformation::getH0() const
{
    return limb->getH0();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool RigidBodyTransformation::setH0(const Matrix &_H0)
{
    return limb->setH0(_H0);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //---------------
    // jacobians COM
    //---------------

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix RigidBodyTransformation::TESTING_computeCOMJacobian(const unsigned int iLink, bool rbtRoto)
{
    if(rbtRoto==false)
        return limb->TESTING_computeCOMJacobian(iLink);
    else
        return getR6() * limb->TESTING_computeCOMJacobian(iLink);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix RigidBodyTransformation::TESTING_computeCOMJacobian(const unsigned int iLink, const Matrix &Pn, bool rbtRoto)
{
   if(rbtRoto==false)
        return limb->TESTING_computeCOMJacobian(iLink, Pn);
    else
        return getR6() * limb->TESTING_computeCOMJacobian(iLink, Pn);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix RigidBodyTransformation::TESTING_computeCOMJacobian(const unsigned int iLink, const Matrix &Pn, const Matrix &_H0, bool rbtRoto)
{
   if(rbtRoto==false)
        return limb->TESTING_computeCOMJacobian(iLink, Pn, _H0);
    else
        return getR6() * limb->TESTING_computeCOMJacobian(iLink, Pn, _H0);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix RigidBodyTransformation::getHCOM(unsigned int iLink)
{
   return limb->getHCOM(iLink);
}




//====================================
//
//      i DYN NODE
//
//====================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynNode::iDynNode(const NewEulMode _mode)
{
    rbtList.clear();
    mode = _mode;
    verbose = iCub::skinDynLib::VERBOSE;
    zero();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynNode::iDynNode(const string &_info, const NewEulMode _mode, unsigned int verb)
{
    info=_info;
    rbtList.clear();
    mode = _mode;
    verbose = verb;
    zero();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynNode::zero()
{
    w.resize(3); w.zero();
    dw.resize(3); dw.zero();
    ddp.resize(3); ddp.zero();
    F.resize(3); F.zero();
    Mu.resize(3); Mu.zero();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynNode::addLimb(iDynLimb *limb, const Matrix &H, const FlowType kinFlow, const FlowType wreFlow, bool hasSensor)
{
    string infoRbt = limb->getType() + " to node";
    RigidBodyTransformation rbt(limb,H,infoRbt,hasSensor,kinFlow,wreFlow,mode,verbose);
    rbtList.push_back(rbt);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynNode::getRBT(unsigned int iLimb) const
{
    if(iLimb<rbtList.size())
    {
        return rbtList[iLimb].getRBT();
    }
    else
    {
        if(verbose) fprintf(stderr,"iDynNode: error, could not getRBT() due to out of range index: %d , while we have %d limbs. \n", iLimb, (int)rbtList.size() );
        return Matrix(0,0);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynNode::solveKinematics()
{
    unsigned int inputNode=0;

    //first find the limb (one!) which must get the measured kinematics data
    // e.g. the head gets this information from the inertial sensor on the head
    for(unsigned int i=0; i<rbtList.size(); i++)
    {
        if(rbtList[i].getKinematicFlow()==RBT_NODE_IN)
        {
            // measures are already set
            //then compute the kinematics pass in that limb
            rbtList[i].computeLimbKinematic();
            // and retrieve the kinematics data in the base/end
            rbtList[i].getKinematic(w,dw,ddp);
            //check
            inputNode++;
        }
    }

    //just check if the input node is only one (as it should be)
    if(inputNode==1)
    {
        //now forward the kinematic input from limbs whose kinematic flow is input type
        for(unsigned int i=0; i<rbtList.size(); i++)
        {
            if(rbtList[i].getKinematicFlow()==RBT_NODE_OUT)
            {
                //init the kinematics with the node information
                rbtList[i].setKinematic(w,dw,ddp);
                //solve kinematics in that limb/chain
                rbtList[i].computeLimbKinematic();
            }
        }
        return true;

    }
    else
    {
        if(verbose)
        {
            fprintf(stderr,"iDynNode error: there are %d limbs with Kinematic Flow = Input. Only one limb must have Kinematic Input from outside measurements/computations. \n",inputNode);
            fprintf(stderr,"Please check the coherence of the limb configuration in the node <%s> \n",info.c_str());
        }
        return false;
    }

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynNode::solveKinematics(const Vector &w0, const Vector &dw0, const Vector &ddp0)
{
    unsigned int inputNode=0;

    //first find the limb (one!) which must get the measured kinematics data
    // e.g. the head gets this information from the inertial sensor on the head
    for(unsigned int i=0; i<rbtList.size(); i++)
    {
        if(rbtList[i].getKinematicFlow()==RBT_NODE_IN)
        {
            rbtList[i].setKinematicMeasure(w0,dw0,ddp0);
            //then compute the kinematics pass in that limb
            rbtList[i].computeLimbKinematic();
            // and retrieve the kinematics data in the base/end
            rbtList[i].getKinematic(w,dw,ddp);
            //check
            inputNode++;
        }
    }

    //just check if the input node is only one (as it should be)
    if(inputNode==1)
    {
        //now forward the kinematic input from limbs whose kinematic flow is input type
        for(unsigned int i=0; i<rbtList.size(); i++)
        {
            if(rbtList[i].getKinematicFlow()==RBT_NODE_OUT)
            {
                //init the kinematics with the node information
                rbtList[i].setKinematic(w,dw,ddp);
                //solve kinematics in that limb/chain
                rbtList[i].computeLimbKinematic();
            }
        }
        return true;

    }
    else
    {
        if(verbose)
        {
            fprintf(stderr,"iDynNode: error: there are %d limbs with Kinematic Flow = Input. ",inputNode);
            fprintf(stderr," Only one limb must have Kinematic Input from outside measurements/computations. ");
            fprintf(stderr,"Please check the coherence of the limb configuration in the node <%s> \n", info.c_str());
        }
        return false;
    }

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynNode::setKinematicMeasure(const Vector &w0, const Vector &dw0, const Vector &ddp0)
{
    if( (w0.length()==3)&&(dw0.length()==3)&&(ddp0.length()==3))
    {
        // find the limb (one!) which must get the measured kinematics data
        // e.g. the head gets this information from the inertial sensor on the head
        for(unsigned int i=0; i<rbtList.size(); i++)
        {
            if(rbtList[i].getKinematicFlow()==RBT_NODE_IN)
            {
                rbtList[i].setKinematicMeasure(w0,dw0,ddp0);
                return true;
            }
        }
        if(verbose)
            fprintf(stderr,"iDynNode: error, there is not a node to set kinematic measure! \n");
        return false;
    }
    else
    {
        if(verbose)
        {
            fprintf(stderr,"iDynNode: error, could not set Kinematic measures due to wrong sized vectors: \n");
            fprintf(stderr," w,dw,ddp have length %d,%d,%d instead of 3,3,3. \n",(int)w0.length(),(int)dw0.length(),(int)ddp0.length());
        }
        return false;
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynNode::solveWrench()
{
    unsigned int outputNode = 0;
    F.zero(); Mu.zero();

    //first get the forces/moments from each limb
    //assuming that each limb has been properly set with the outcoming measured
    //forces/moments which are necessary for the wrench computation
    for(unsigned int i=0; i<rbtList.size(); i++)
    {
        if(rbtList[i].getWrenchFlow()==RBT_NODE_IN)
        {
            //compute the wrench pass in that limb
            rbtList[i].computeLimbWrench();
            //update the node force/moment with the wrench coming from the limb base/end
            // note that getWrench sum the result to F,Mu - because they are passed by reference
            // F = F + F[i], Mu = Mu + Mu[i]
            rbtList[i].getWrench(F,Mu);
            //check
            outputNode++;
        }
    }

    // node summation: already performed by each RBT
    // F = F + F[i], Mu = Mu + Mu[i]

    // at least one output node should exist
    // however if for testing purposes only one limb is attached to the node,
    // we can't avoid the computeWrench phase, but still we must remember that
    // it is not correct, because the node must be in balance
    if(outputNode==rbtList.size())
    {
        if(verbose>1)
        {
            fprintf(stderr,"iDynNode: warning: there are no limbs with Wrench Flow = Output. ");
            fprintf(stderr," At least one limb must have Wrench Output for balancing forces in the node. ");
            fprintf(stderr,"Please check the coherence of the limb configuration in the node <%s> \n",info.c_str());
        }
    }

    //now forward the wrench output from the node to limbs whose wrench flow is output type
    for(unsigned int i=0; i<rbtList.size(); i++)
    {
        if(rbtList[i].getWrenchFlow()==RBT_NODE_OUT)
        {
            //init the wrench with the node information
            rbtList[i].setWrench(F,Mu);
            //solve wrench in that limb/chain
            rbtList[i].computeLimbWrench();
        }
    }
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynNode::solveWrench(const Matrix &FM)
{
    bool inputWasOk = setWrenchMeasure(FM);
    //now that all is set, we can really solveWrench()
    return ( solveWrench() && inputWasOk );
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynNode::solveWrench(const Matrix &Fm, const Matrix &Mm)
{
    bool inputWasOk = setWrenchMeasure(Fm,Mm);
    //now that all is set, we can really solveWrench()
    return ( solveWrench() && inputWasOk );
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynNode::setWrenchMeasure(const Matrix &FM)
{
    Vector fi(3); fi.zero();
    Vector mi(3); mi.zero();
    Vector FMi(6);FMi.zero();
    bool inputWasOk = true;

    //check how many limbs have wrench input
    int inputNode = howManyWrenchInputs(false);

    // input (eg measured) wrenches are stored in a 6xN matrix: each column is a 6x1 vector
    // with force/moment; N is the number of columns, ie the number of measured/input wrenches to the limb
    // the order is assumed coherent with the one built when adding limbs
    // eg:
    // adding limbs: addLimb(limb1), addLimb(limb2), addLimb(limb3)
    // where limb1, limb3 have wrench flow input
    // passing wrenches: Matrix FM(6,2), FM.setcol(0,fm1), FM.setcol(1,fm3)
    if(FM.cols()<inputNode)
    {
        if(verbose)
        {
            fprintf(stderr,"iDynNode: could not solveWrench due to missing wrenches to initialize the computations: ");
            fprintf(stderr," only %zu f/m available instead of %d. Using default values, all zero. \n ",FM.cols(),inputNode);
        }
        inputWasOk = false;
    }
    if(FM.rows()!=6)
    {
        if(verbose)
        {
            fprintf(stderr,"iDynNode: could not solveWrench due to wrong sized init wrenches: ");
            fprintf(stderr," %zu instead of 6 (3+3). Using default values, all zero. \n",FM.rows());
        }
        inputWasOk = false;
    }

    //set the measured/input forces/moments from each limb
    if(inputWasOk)
    {
        inputNode = 0;
        for(unsigned int i=0; i<rbtList.size(); i++)
        {
            if(rbtList[i].getWrenchFlow()==RBT_NODE_IN)
            {
                // from the input matrix - read the input wrench
                FMi = FM.getCol(inputNode);
                fi[0]=FMi[0];fi[1]=FMi[1];fi[2]=FMi[2];
                mi[0]=FMi[3];mi[1]=FMi[4];mi[2]=FMi[5];
                inputNode++;
                //set the input wrench in the RBT->limb
                rbtList[i].setWrenchMeasure(fi,mi);
            }
        }
    }
    else
    {
        // default zero values if inputs are wrong sized
        for(unsigned int i=0; i<rbtList.size(); i++)
            if(rbtList[i].getWrenchFlow()==RBT_NODE_IN)
                rbtList[i].setWrenchMeasure(fi,mi);
    }

    //now that all is set, we can really solveWrench()
    return inputWasOk;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynNode::setWrenchMeasure(const Matrix &Fm, const Matrix &Mm)
{
    bool inputWasOk = true;

    //check how many limbs have wrench input
    int inputNode = howManyWrenchInputs(false);

    // input (eg measured) wrenches are stored in two 3xN matrix: each column is a 3x1 vector
    // with force/moment; N is the number of columns, ie the number of measured/input wrenches to the limb
    // the order is assumed coherent with the one built when adding limbs
    // eg:
    // adding limbs: addLimb(limb1), addLimb(limb2), addLimb(limb3)
    // where limb1, limb3 have wrench flow input
    // passing wrenches: Matrix Fm(3,2), Fm.setcol(0,f1), Fm.setcol(1,f3) and similar for moment
    if((Fm.cols()<inputNode)||(Mm.cols()<inputNode))
    {
        if(verbose)
        {
            fprintf(stderr,"iDynNode: could not setWrenchMeasure due to missing wrenches to initialize the computations: ");
            fprintf(stderr," only %zu/%zu f/m available instead of %d/%d. Using default values, all zero. \n",Fm.cols(),Mm.cols(),inputNode,inputNode);
        }
        inputWasOk = false;
    }
    if((Fm.rows()!=3)||(Mm.rows()!=3))
    {
        if(verbose)
        {
            fprintf(stderr,"iDynNode: could not setWrenchMeasure due to wrong sized init f/m: ");
            fprintf(stderr," %zu/%zu instead of 3/3. Using default values, all zero. \n",Fm.rows(),Mm.rows());
        }
        inputWasOk = false;
    }

    //set the measured/input forces/moments from each limb
    if(inputWasOk)
    {
        inputNode = 0;
        for(unsigned int i=0; i<rbtList.size(); i++)
        {
            if(rbtList[i].getWrenchFlow()==RBT_NODE_IN)
            {
                // from the input matrix
                //set the input wrench in the RBT->limb
                rbtList[i].setWrenchMeasure(Fm.getCol(inputNode),Mm.getCol(inputNode));
                inputNode++;
            }
        }
    }
    else
    {
        // default zero values if inputs are wrong sized
        Vector fi(3), mi(3); fi.zero(); mi.zero();
        for(unsigned int i=0; i<rbtList.size(); i++)
            if(rbtList[i].getWrenchFlow()==RBT_NODE_IN)
                rbtList[i].setWrenchMeasure(fi,mi);
    }

    //now that all is set, we can really solveWrench()
    return inputWasOk ;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
unsigned int iDynNode::howManyWrenchInputs(bool afterAttach) const
{
    unsigned int inputNode = 0;

    //check how many limbs have wrench input
    for(unsigned int i=0; i<rbtList.size(); i++)
        if(rbtList[i].getWrenchFlow()==RBT_NODE_IN)
            inputNode++;

    // if an attach has been done, we already set one wrench measure, so we don't have to do it again
    // and we expect FM (or F,M) to be smaller of 1
    if(afterAttach==true) inputNode--;

    return inputNode;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
unsigned int iDynNode::howManyKinematicInputs(bool afterAttach) const
{
    unsigned int inputNode = 0;

    //check how many limbs have kinematic input
    for(unsigned int i=0; i<rbtList.size(); i++)
        if(rbtList[i].getKinematicFlow()==RBT_NODE_IN)
            inputNode++;

    // if an attach has been done, we already set one kinematic measure, so we don't have to do it again
    // actually we expect the number to be zero, because only one limb has kinematic measures
    if(afterAttach==true) inputNode--;

    return inputNode;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynNode::getForce() const { return F;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynNode::getMoment() const {return Mu;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynNode::getAngVel() const {return w;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynNode::getAngAcc() const {return dw;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynNode::getLinAcc() const {return ddp;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        //-----------------
        //    jacobians
        //-----------------


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynNode::computeJacobian(unsigned int iChain)
{
    if(iChain<=rbtList.size())
    {
        return rbtList[iChain].computeGeoJacobian(false);
    }
    else
    {
        if(verbose) fprintf(stderr,"iDynNode: error, could not computeJacobian() due to out of range index: input limb has index %d > %d. Returning a null matrix. \n",iChain,(int)rbtList.size());
        return Matrix(0,0);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynNode::computeJacobian(unsigned int iChain, unsigned int iLink)
{
    if(iChain<=rbtList.size())
    {
        // the check on iLink<=N is performed by iKinChain when calling GeoJacobian(iLink)
        // from the RBT
        return rbtList[iChain].computeGeoJacobian(iLink,false);
    }
    else
    {
        if(verbose) fprintf(stderr,"iDynNode: error, could not computeJacobian() due to out of range index: input limb has index %d > %d. Returning a null matrix. \n",iChain,(int)rbtList.size());
        return Matrix(0,0);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynNode::computeJacobian(unsigned int iChainA, JacobType dirA, unsigned int iChainB, JacobType dirB)
{
    //first check param coherence:
    // - wrong limb index
    if( (iChainA > rbtList.size())||(iChainB > rbtList.size()) )
    {
        if(verbose) fprintf(stderr,"iDynNode: error, could not computeJacobian() due to out of range index: limbs have index %d,%d > %d. Returning a null matrix. \n",iChainA,iChainB,(int)rbtList.size());
        return Matrix(0,0);
    }
    // - jacobian .. in the same limb @_@
    if( iChainA==iChainB )
    {
        if(verbose) fprintf(stderr,"iDynNode: error, could not computeJacobian() due to weird index for chains %d: same chains are selected. Please check the indexes or use the method iDynNode::computeJacobian(unsigned int iChain). Returning a null matrix. \n",iChainA);
        return Matrix(0,0);
    }

    // params are ok, go on..

    // total number of joints = Ndof_A + Ndof_B
    // the total jacobian matrix
    Matrix J(6,rbtList[iChainA].getDOF() + rbtList[iChainB].getDOF()); J.zero();
    //the vector from the base-link (for the jac.) of limb A to the end-link (for the jac.) of limb B
    Matrix Pn;
    // the two jacobians
    Matrix J_A; Matrix J_B;
    // from base-link (for the jac.) of limb A to base (for the jac.) of limb B
    Matrix H_A_Node;

    // compute the roto-transf matrix between the base of limb A and the base of limb B
    // this is used to set the correct reference of vector Pn
    // H_A_Node is used to initialize J_B properly
    compute_Pn_HAN(iChainA, dirA, iChainB, dirB, Pn, H_A_Node);

    // now compute jacobian of first and second limb, setting the correct Pn
    // JA
    J_A = rbtList[iChainA].computeGeoJacobian(Pn,false);
    // JB
    // note: set H_A_node as the H0 of the B chain, but does not modify the chain original H0
    J_B = rbtList[iChainB].computeGeoJacobian(Pn,H_A_Node,false);

    // finally start building the Jacobian J=[J_A|J_B]
    unsigned int c=0;
    unsigned int JAcols = J_A.cols();
    unsigned int JBcols = J_B.cols();

    if(JAcols+JBcols!=J.cols())
    {
        fprintf(stderr,"iDynNode error: Jacobian should be 6x%zu instead is 6x%d \n",J.cols(),(JAcols+JBcols));
        fprintf(stderr,"Note:  limb A:  N=%d  DOF=%d  \n",rbtList[iChainA].getNLinks(),rbtList[iChainA].getDOF());
        fprintf(stderr,"Note:  limb B:  N=%d  DOF=%d  \n",rbtList[iChainB].getNLinks(),rbtList[iChainB].getDOF());
        J.resize(6,JAcols+JBcols);
    }

    for(unsigned int r=0; r<6; r++)
    {
        for(c=0;c<JAcols;c++)
            J(r,c)=J_A(r,c);
        for(c=0;c<JBcols;c++)
            J(r,JAcols+c)=J_B(r,c);
    }

    // now return the jacobian
    return J;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynNode::computeJacobian(unsigned int iChainA, JacobType dirA, unsigned int iChainB, unsigned int iLinkB, JacobType dirB)
{
    //first check param coherence:
    // - wrong limb index
    if( (iChainA > rbtList.size())||(iChainB > rbtList.size()) )
    {
        if(verbose) fprintf(stderr,"iDynNode: error, could not computeJacobian() due to out of range index: limbs have index %d,%d > %d. Returning a null matrix.\n",iChainA,iChainB,(int)rbtList.size());
        return Matrix(0,0);
    }
    // - jacobian .. in the same limb @_@
    if( iChainA==iChainB )
    {
        if(verbose) fprintf(stderr,"iDynNode: error, could not computeJacobian() due to weird index for chains %d : same chains are selected. Please check the indexes or use the method iDynNode::computeJacobian(unsigned int iChain). Returning a null matrix.\n",iChainA);
        return Matrix(0,0);
    }
    // - there's not a link with index iLink in that chain
    if( iLinkB >= rbtList[iChainB].getNLinks())
    {
        if(verbose) fprintf(stderr,"iDynNode: error, could not computeJacobian() due to out of range index for chain %d: the selected link %d > %d. Returning a null matrix. \n",iChainB,iLinkB,rbtList[iChainB].getNLinks());
        return Matrix(0,0);
    }

    // params are ok, go on..

    // total number of joints = Ndof_A + iLinkB
    // note that we are taking all the links in chainB, not only the DOF until iLinkB
    // the total jacobian matrix
    Matrix J(6,rbtList[iChainA].getDOF() + iLinkB ); J.zero();
    //the vector from the base-link (for the jac.) of limb A to the end-link (for the jac.) of limb B
    Matrix Pn;
    // the two jacobians
    Matrix J_A; Matrix J_B;
    // from base-link (for the jac.) of limb A to base (for the jac.) of limb B
    Matrix H_A_Node;

    // compute the roto-transf matrix between the base of limb A and the base of limb B
    // this is used to set the correct reference of vector Pn
    // H_A_Node is used to initialize J_B properly
    compute_Pn_HAN(iChainA, dirA, iChainB, iLinkB, dirB, Pn, H_A_Node);

    // now compute jacobian of first and second limb, setting the correct Pn
    // JA
    J_A = rbtList[iChainA].computeGeoJacobian(Pn,false);
    // JB
    // note: set H_A_node as the H0 of the B chain, but does not modify the chain original H0
    J_B = rbtList[iChainB].computeGeoJacobian(iLinkB,Pn,H_A_Node,false);

    // finally start building the Jacobian J=[J_A|J_B]
    unsigned int c=0;
    unsigned int JAcols = J_A.cols();
    unsigned int JBcols = J_B.cols();

    if(JAcols+JBcols!=J.cols())
    {
        fprintf(stderr,"iDynNode error: Jacobian should be 6x%zu instead is 6x%d \n",J.cols(),(JAcols+JBcols));
        fprintf(stderr,"Note:  limb A:  N=%d  DOF=%d  \n",rbtList[iChainA].getNLinks(),rbtList[iChainA].getDOF());
        fprintf(stderr,"Note:  limb B:  N=%d  DOF=%d  iLinkB=%d \n",rbtList[iChainB].getNLinks(),rbtList[iChainB].getDOF(),iLinkB);
        J.resize(6,JAcols+JBcols);
    }

    for(unsigned int r=0; r<6; r++)
    {
        for(c=0;c<JAcols;c++)
            J(r,c)=J_A(r,c);
        for(c=0;c<JBcols;c++)
            J(r,JAcols+c)=J_B(r,c);
    }

    // now return the jacobian
    return J;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynNode::compute_Pn_HAN(unsigned int iChainA, JacobType dirA, unsigned int iChainB, JacobType dirB, Matrix &Pn, Matrix &H_A_Node)
{
    // compute the roto-transf matrix between the base of limb A and the base of limb B
    // this is used to set the correct reference of vector Pn
    if(dirA==JAC_KIN)
    {
        // H_A_Node = H_A * RBT_A^T * RBT_B
        // note: RBT_A is transposed because we're going in the opposite direction wrt to one of the RBT
        H_A_Node = rbtList[iChainA].getH() * rbtList[iChainA].getRBT().transposed() * rbtList[iChainB].getRBT();
    }
    else //dirA==JAC_IKIN
    {
        // H_A_Node = H_A^-1 * RBT_A^T * RBT_B
        H_A_Node = SE3inv(rbtList[iChainA].getH()) * rbtList[iChainA].getRBT().transposed() * rbtList[iChainB].getRBT();
    }

    if(dirB==JAC_KIN)
    {
        // Pn = H_A_Node * H_B
        // Pn is the roto-transf matrix between base (of jac. in limb A) and end (of jac. in limb B)
        // it is needed because the two jacobians must refer to a common Pn
        Pn = H_A_Node * rbtList[iChainB].getH();
    }
    else //dirB==JAC_IKIN
    {
        // Pn = H_A_Node * H_B^-1
        Pn = H_A_Node * SE3inv(rbtList[iChainB].getH());
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynNode::compute_Pn_HAN(unsigned int iChainA, JacobType dirA, unsigned int iChainB, unsigned int iLinkB, JacobType dirB, Matrix &Pn, Matrix &H_A_Node)
{
    // compute the roto-transf matrix between the base of limb A and the base of limb B
    // this is used to set the correct reference of vector Pn
    if(dirA==JAC_KIN)
    {
        // H_A_Node = H_A * RBT_A^T * RBT_B
        // note: RBT_A is transposed because we're going in the opposite direction wrt to one of the RBT
        H_A_Node = rbtList[iChainA].getH() * rbtList[iChainA].getRBT().transposed() * rbtList[iChainB].getRBT();
    }
    else //dirA==JAC_IKIN
    {
        // H_A_Node = H_A^-1 * RBT_A^T * RBT_B
        H_A_Node = SE3inv(rbtList[iChainA].getH()) * rbtList[iChainA].getRBT().transposed() * rbtList[iChainB].getRBT();
    }

    if(dirB==JAC_KIN)
    {
        // Pn = H_A_Node * H_B(iLinkB)
        // Pn is the roto-transf matrix between base (of jac. in limb A) and end (of jac. in limb B)
        // it is needed because the two jacobians must refer to a common Pn
        // here for chain B we stop at the iLinkB link
        // allLink=true because we deal with all the links (even blocked ones): see motivation in the corresponding
        // jacobian with iLinkB
        Pn = H_A_Node * rbtList[iChainB].getH(iLinkB,true);
    }
    else //dirB==JAC_IKIN
    {
        // Pn = H_A_Node * H_B^-1
        Pn = H_A_Node * SE3inv(rbtList[iChainB].getH(iLinkB,true));
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynNode::computePose(unsigned int iChainA, JacobType dirA, unsigned int iChainB, JacobType dirB, const bool axisRep)
{
    //first check param coherence:
    // - wrong limb index
    if( (iChainA > rbtList.size())||(iChainB > rbtList.size()) )
    {
        if(verbose) fprintf(stderr,"iDynNode: error, could not computePose() due to out of range index: limbs have index %d,%d > %d. Returning a null matrix. \n",iChainA,iChainB,(int)rbtList.size());
        return Vector(0);
    }
    // - jacobian .. in the same limb @_@
    if( iChainA==iChainB )
    {
        if(verbose) fprintf(stderr,"iDynNode: error, could not computePose() due to weird index for chains %d: same chains are selected. Please check the indexes or use the method iDynNode::computeJacobian(unsigned int iChain). Returning a null matrix. \n",iChainA);
        return Vector(0);
    }

    // params are ok, go on..

    //the vector from the base-link (for the jac.) of limb A to the end-link (for the jac.) of limb B
    Matrix Pn, H_A_Node;
    // the pose vector
    Vector v;

    // compute the roto-transf matrix between the base of limb A and the base of limb B
    // this is used to set the correct reference of vector Pn
    // just ignore H_A_Node
    compute_Pn_HAN(iChainA, dirA, iChainB, dirB, Pn, H_A_Node);

    // now compute the pose vector v (see iKin for more details)
    if (axisRep)
    {
        v.resize(7);
        Vector r=dcm2axis(Pn);
        v[0]=Pn(0,3);
        v[1]=Pn(1,3);
        v[2]=Pn(2,3);
        v[3]=r[0];
        v[4]=r[1];
        v[5]=r[2];
        v[6]=r[3];
    }
    else
    {
        v.resize(6);
        Vector r(3); r.zero();
        // Euler Angles as XYZ (see dcm2angle.m)
        r[0]=atan2(-Pn(2,1),Pn(2,2));
        r[1]=asin(Pn(2,0));
        r[2]=atan2(-Pn(1,0),Pn(0,0));
        v[0]=Pn(0,3);
        v[1]=Pn(1,3);
        v[2]=Pn(2,3);
        v[3]=r[0];
        v[4]=r[1];
        v[5]=r[2];
    }

    return v;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynNode::computePose(unsigned int iChainA, JacobType dirA, unsigned int iChainB, unsigned int iLinkB, JacobType dirB, const bool axisRep)
{
    //first check param coherence:
    // - wrong limb index
    if( (iChainA > rbtList.size())||(iChainB > rbtList.size()) )
    {
        if(verbose) fprintf(stderr,"iDynNode: error, could not computePose() due to out of range index: limbs have index %d,%d > %d. Returning a null matrix. \n",iChainA,iChainB,(int)rbtList.size());
        return Vector(0);
    }
    // - jacobian .. in the same limb @_@
    if( iChainA==iChainB )
    {
        if(verbose) fprintf(stderr,"iDynNode: error, could not computePose() due to weird index for chains %d: same chains are selected. Please check the indexes or use the method iDynNode::computeJacobian(unsigned int iChain). Returning a null matrix. \n",iChainA);
        return Vector(0);
    }
    // - there's not a link with index iLink in that chain
    if( iLinkB >= rbtList[iChainB].getNLinks())
    {
        if(verbose) fprintf(stderr,"iDynNode: error, could not computePose() due to out of range index for chain %d: the selected link is %d > %d. Returning a null matrix. \n",iChainB,iLinkB,rbtList[iChainB].getNLinks());
        return Vector(0);
    }

    // params are ok, go on..

    //the vector from the base-link (for the jac.) of limb A to the end-link (for the jac.) of limb B
    Matrix Pn, H_A_Node;
    // the pose vector
    Vector v;

    // compute the roto-transf matrix between the base of limb A and the base of limb B
    // this is used to set the correct reference of vector Pn
    // just ignore H_A_Node
    compute_Pn_HAN(iChainA, dirA, iChainB, iLinkB, dirB, Pn, H_A_Node);

    // now compute the pose vector v (see iKin for more details)
    if (axisRep)
    {
        v.resize(7);
        Vector r=dcm2axis(Pn);
        v[0]=Pn(0,3);
        v[1]=Pn(1,3);
        v[2]=Pn(2,3);
        v[3]=r[0];
        v[4]=r[1];
        v[5]=r[2];
        v[6]=r[3];
    }
    else
    {
        v.resize(6);
        Vector r(3); r.zero();
        // Euler Angles as XYZ (see dcm2angle.m)
        r[0]=atan2(-Pn(2,1),Pn(2,2));
        r[1]=asin(Pn(2,0));
        r[2]=atan2(-Pn(1,0),Pn(0,0));
        v[0]=Pn(0,3);
        v[1]=Pn(1,3);
        v[2]=Pn(2,3);
        v[3]=r[0];
        v[4]=r[1];
        v[5]=r[2];
    }

    return v;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //---------------
    // jacobians COM
    //---------------

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynNode::TESTING_computeCOMJacobian(unsigned int iChain, unsigned int iLink)
{
    if(iChain<=rbtList.size())
    {
        // the check on iLink<=N is performed by iDynChain when calling computeCOMJacobian(iLink)
        // from the RBT
        return rbtList[iChain].TESTING_computeCOMJacobian(iLink,false);
    }
    else
    {
        if(verbose) fprintf(stderr,"iDynNode: error, could not computeCOMJacobian() due to out of range index: input limb has index %d > %d. Returning a null matrix. \n",iChain,(int)rbtList.size());
        return Matrix(0,0);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynNode::TESTING_computeCOMJacobian(unsigned int iChainA, JacobType dirA, unsigned int iChainB, unsigned int iLinkB, JacobType dirB)
{
    //first check param coherence:
    // - wrong limb index
    if( (iChainA > rbtList.size())||(iChainB > rbtList.size()) )
    {
        if(verbose) fprintf(stderr,"iDynNode: error, could not computeJacobian() due to out of range index: limbs have index %d,%d > %d. Returning a null matrix. \n",iChainA,iChainB,(int)rbtList.size());
        return Matrix(0,0);
    }
    // - jacobian .. in the same limb @_@
    if( iChainA==iChainB )
    {
        if(verbose) fprintf(stderr,"iDynNode: error, could not computeJacobian() due to weird index for chains %d: same chains are selected. Please check the indexes or use the method iDynNode::computeJacobian(unsigned int iChain). Returning a null matrix. \n",iChainA);
        return Matrix(0,0);
    }
    // - there's not a link with index iLink in that chain
    if( iLinkB >= rbtList[iChainB].getNLinks())
    {
        if(verbose) fprintf(stderr,"iDynNode: error, could not computeJacobian() due to out of range index for chain %d: the selected link is %d > %d. Returning a null matrix. \n",iChainB,iLinkB,rbtList[iChainB].getNLinks());
        return Matrix(0,0);
    }

    // params are ok, go on..

    // total number of joints = Ndof_A + iLinkB
    // note that we are taking all the links in chainB, not only the DOF until iLinkB
    // the total jacobian matrix
    Matrix J(6,rbtList[iChainA].getDOF() + iLinkB ); J.zero();
    //the vector from the base-link (for the jac.) of limb A to the end-link (for the jac.) of limb B
    Matrix Pn;
    // the two jacobians
    Matrix J_A; Matrix J_B;
    // from base-link (for the jac.) of limb A to base (for the jac.) of limb B
    Matrix H_A_Node;

    // compute the roto-transf matrix between the base of limb A and the base of limb B
    // H_A_Node is used to initialize J_B properly
    // Pn consider chain A, chain B until iLinkB, then the COM of iLinkB
    compute_Pn_HAN_COM(iChainA, dirA, iChainB, iLinkB, dirB, Pn, H_A_Node);

    // now compute jacobian of first and second limb, setting the correct Pn
    // JA
    J_A = rbtList[iChainA].computeGeoJacobian(Pn,false);
    // JB
    // note: set H_A_node as the H0 of the B chain, but does not modify the chain original H0
    J_B = rbtList[iChainB].TESTING_computeCOMJacobian(iLinkB,Pn,H_A_Node,false);

    // finally start building the Jacobian J=[J_A|J_B]
    unsigned int c=0;
    unsigned int JAcols = J_A.cols();
    unsigned int JBcols = J_B.cols();

    if(JAcols+JBcols!=J.cols())
    {
        fprintf(stderr,"iDynNode error: Jacobian should be 6x%zu instead is 6x%d \n",J.cols(),(JAcols+JBcols));
        fprintf(stderr,"Note:  limb A:  N=%d  DOF=%d  \n",rbtList[iChainA].getNLinks(),rbtList[iChainA].getDOF());
        fprintf(stderr,"Note:  limb B:  N=%d  DOF=%d  iLinkB=%d \n",rbtList[iChainB].getNLinks(),rbtList[iChainB].getDOF(),iLinkB);
        J.resize(6,JAcols+JBcols);
    }

    for(unsigned int r=0; r<6; r++)
    {
        for(c=0;c<JAcols;c++)
            J(r,c)=J_A(r,c);
        for(c=0;c<JBcols;c++)
            J(r,JAcols+c)=J_B(r,c);
    }

    // now return the jacobian
    return J;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynNode::compute_Pn_HAN_COM(unsigned int iChainA, JacobType dirA, unsigned int iChainB, unsigned int iLinkB, JacobType dirB, Matrix &Pn, Matrix &H_A_Node)
{
    // compute the roto-transf matrix between the base of limb A and the base of limb B
    // this is used to set the correct reference of vector Pn
    if(dirA==JAC_KIN)
    {
        // H_A_Node = H_A * RBT_A^T * RBT_B
        // note: RBT_A is transposed because we're going in the opposite direction wrt to one of the RBT
        H_A_Node = rbtList[iChainA].getH() * rbtList[iChainA].getRBT().transposed() * rbtList[iChainB].getRBT();
    }
    else //dirA==JAC_IKIN
    {
        // H_A_Node = H_A^-1 * RBT_A^T * RBT_B
        H_A_Node = SE3inv(rbtList[iChainA].getH()) * rbtList[iChainA].getRBT().transposed() * rbtList[iChainB].getRBT();
    }

    if(dirB==JAC_KIN)
    {
        // Pn = H_A_Node * H_B(iLinkB)
        // Pn is the roto-transf matrix between base (of jac. in limb A) and end (of jac. in limb B)
        // it is needed because the two jacobians must refer to a common Pn
        // here for chain B we stop at the iLinkB link
        // allLink=true because we deal with all the links (even blocked ones): see motivation in the corresponding
        // jacobian with iLinkB
        Pn = H_A_Node * rbtList[iChainB].getHCOM(iLinkB);
    }
    else //dirB==JAC_IKIN
    {
        // Pn = H_A_Node * H_B^-1
        Pn = H_A_Node * SE3inv(rbtList[iChainB].getHCOM(iLinkB));
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~






//====================================
//
//      i DYN SENSOR NODE
//
//====================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynSensorNode::iDynSensorNode(const NewEulMode _mode)
:iDynNode(_mode)
{
    sensorList.clear();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynSensorNode::iDynSensorNode(const string &_info, const NewEulMode _mode, unsigned int verb)
:iDynNode(_info,_mode,verb)
{
    sensorList.clear();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynSensorNode::addLimb(iDynLimb *limb, const Matrix &H, const FlowType kinFlow, const FlowType wreFlow)
{
    iDynNode::addLimb(limb,H,kinFlow,wreFlow,false);
    iDynSensor *noSens = NULL;
    sensorList.push_back(noSens);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynSensorNode::addLimb(iDynLimb *limb, const Matrix &H, iDynSensor *sensor, const FlowType kinFlow, const FlowType wreFlow)
{
    iDynNode::addLimb(limb,H,kinFlow,wreFlow,true);
    sensorList.push_back(sensor);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynSensorNode::solveWrench()
{
    unsigned int outputNode = 0;
    // set to zero the node force/moment
    F.zero(); Mu.zero();

    //first get the forces/moments from each limb
    //assuming that each limb has been properly set with the outcoming measured
    //forces/moments which are necessary for the wrench computation
    for(unsigned int i=0; i<rbtList.size(); i++)
    {
        if(rbtList[i].getWrenchFlow()==RBT_NODE_IN)
        {
            //compute the wrench pass in that limb
            // if there's a sensor, we must use iDynSensor
            // otherwise we use the limb method as usual
            if(rbtList[i].isSensorized()==true)
                sensorList[i]->computeWrenchFromSensorNewtonEuler();
            else
                rbtList[i].computeLimbWrench();

            //update the node force/moment with the wrench coming from the limb base/end
            // note that getWrench sum the result to F,Mu - because they are passed by reference
            // F = F + F[i], Mu = Mu + Mu[i]
            rbtList[i].getWrench(F,Mu);
            //check
            outputNode++;
        }
    }

    // node summation: already performed by each RBT
    // F = F + F[i], Mu = Mu + Mu[i]

    // at least one output node should exist
    // however if for testing purposes only one limb is attached to the node,
    // we can't avoid the computeWrench phase, but still we must remember that
    // it is not correct, because the node must be in balanc
    if(outputNode==rbtList.size())
    {
        if(verbose>1)
        {
            fprintf(stderr,"iDynNode: warning: there are no limbs with Wrench Flow = Output. At least one limb must have Wrench Output for balancing forces in the node. \n");
            fprintf(stderr,"Please check the coherence of the limb configuration in the node <%s>\n",info.c_str());
        }
    }

    //now forward the wrench output from the node to limbs whose wrench flow is output type
    // assuming they don't have a FT sensor
    for(unsigned int i=0; i<rbtList.size(); i++)
    {
        if(rbtList[i].getWrenchFlow()==RBT_NODE_OUT)
        {
            //init the wrench with the node information
            rbtList[i].setWrench(F,Mu);
            //solve wrench in that limb/chain
            rbtList[i].computeLimbWrench();
        }
    }
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynSensorNode::setWrenchMeasure(const Matrix &FM, bool afterAttach)
{
    Vector fi(3); fi.zero();
    Vector mi(3); mi.zero();
    Vector FMi(6);FMi.zero();
    bool inputWasOk = true;

    //check how many limbs have wrench input
    int inputNode = howManyWrenchInputs(afterAttach);

    // input (eg measured) wrenches are stored in a 6xN matrix: each column is a 6x1 vector
    // with force/moment; N is the number of columns, ie the number of measured/input wrenches to the limb
    // the order is assumed coherent with the one built when adding limbs
    // eg:
    // adding limbs: addLimb(limb1), addLimb(limb2), addLimb(limb3)
    // where limb1, limb3 have wrench flow input
    // passing wrenches: Matrix FM(6,2), FM.setcol(0,fm1), FM.setcol(1,fm3)
    if(FM.cols()<inputNode)
    {
        if(verbose)
        {
            fprintf(stderr,"iDynNode: could not setWrenchMeasure due to missing wrenches to initialize the computations: only %zu f/m available instead of %d. Using default values, all zero.\n",FM.cols(),inputNode);
            if(afterAttach==true)
            fprintf(stderr,"          Remember that the first limb receives wrench input during an attach from another node. \n");
        }
        inputWasOk = false;
    }
    if(FM.rows()!=6)
    {
        if(verbose)
            fprintf(stderr,"iDynNode: could not setWrenchMeasure due to wrong sized init wrenches: %zu instead of 6 (3+3). Using default values, all zero.\n",FM.rows());
        inputWasOk = false;
    }

    //set the input forces/moments from each limb at base/end
    // note: if afterAttach=true we set the wrench only in limbs 1,2,..
    // if afterAttach=false, we set the wrench in all limbs 0,1,2,..
    unsigned int startLimb=0;
    if(afterAttach) startLimb=1;

    //set the measured/input forces/moments from each limb
    if(inputWasOk)
    {
        inputNode = 0;
        for(unsigned int i=startLimb; i<rbtList.size(); i++)
        {
            if(rbtList[i].getWrenchFlow()==RBT_NODE_IN)
            {
                // from the input matrix - read the input wrench
                FMi = FM.getCol(inputNode);
                fi[0]=FMi[0];fi[1]=FMi[1];fi[2]=FMi[2];
                mi[0]=FMi[3];mi[1]=FMi[4];mi[2]=FMi[5];
                inputNode++;
                //set the input wrench in the RBT->limb
                // if there's a sensor, set on the sensor
                // otherwise on base/end as usual
                if(rbtList[i].isSensorized()==true)
                    rbtList[i].setWrenchMeasure(sensorList[i],fi,mi);
                else
                    rbtList[i].setWrenchMeasure(fi,mi);
            }
        }
    }
    else
    {
        // default zero values if inputs are wrong sized
        for(unsigned int i=startLimb; i<rbtList.size(); i++)
            if(rbtList[i].getWrenchFlow()==RBT_NODE_IN)
            {
                if(rbtList[i].isSensorized()==true)
                    rbtList[i].setWrenchMeasure(sensorList[i],fi,mi);
                else
                    rbtList[i].setWrenchMeasure(fi,mi);
            }
    }

    //now that all is set, we can really solveWrench()
    return inputWasOk ;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynSensorNode::setWrenchMeasure(const Matrix &Fm, const Matrix &Mm, bool afterAttach)
{
    bool inputWasOk = true;

    //check how many limbs have wrench input
    int inputNode = howManyWrenchInputs(afterAttach);

    // input (eg measured) wrenches are stored in two 3xN matrix: each column is a 3x1 vector
    // with force/moment; N is the number of columns, ie the number of measured/input wrenches to the limb
    // the order is assumed coherent with the one built when adding limbs
    // eg:
    // adding limbs: addLimb(limb1), addLimb(limb2), addLimb(limb3)
    // where limb1, limb3 have wrench flow input
    // passing wrenches: Matrix Fm(3,2), Fm.setcol(0,f1), Fm.setcol(1,f3) and similar for moment
    if((Fm.cols()<inputNode)||(Mm.cols()<inputNode))
    {
        if(verbose)
        {
            fprintf(stderr,"iDynNode: could not setWrenchMeasure due to missing wrenches to initialize the computations: only %zu/%zu f/m available instead of %d/%d. Using default values, all zero.\n",Fm.cols(),Mm.cols(),inputNode,inputNode);
            if(afterAttach==true)
            fprintf(stderr,"          Remember that the first limb receives wrench input during an attach from another node.\n");
        }
        inputWasOk = false;
    }
    if((Fm.rows()!=3)||(Mm.rows()!=3))
    {
        if(verbose) fprintf(stderr,"iDynNode: could not setWrenchMeasure due to wrong sized init f/m: %zu/%zu instead of 3/3. Using default values, all zero.\n",Fm.rows(),Mm.rows());
        inputWasOk = false;
    }

    //set the input forces/moments from each limb at base/end
    // note: if afterAttach=true we set the wrench only in limbs 1,2,..
    // if afterAttach=false, we set the wrench in all limbs 0,1,2,..
    unsigned int startLimb=0;
    if(afterAttach) startLimb=1;

    //set the measured/input forces/moments from each limb
    if(inputWasOk)
    {
        inputNode = 0;
        for(unsigned int i=startLimb; i<rbtList.size(); i++)
        {
            if(rbtList[i].getWrenchFlow()==RBT_NODE_IN)
            {
                // from the input matrix
                //set the input wrench in the RBT->limb
                // if there's a sensor, set on the sensor
                // otherwise on base/end as usual
                if(rbtList[i].isSensorized()==true)
                    rbtList[i].setWrenchMeasure(sensorList[i],Fm.getCol(inputNode),Mm.getCol(inputNode));
                else
                    rbtList[i].setWrenchMeasure(Fm.getCol(inputNode),Mm.getCol(inputNode));
                inputNode++;
            }
        }
    }
    else
    {
        // default zero values if inputs are wrong sized
        Vector fi(3), mi(3); fi.zero(); mi.zero();
        for(unsigned int i=startLimb; i<rbtList.size(); i++)
            if(rbtList[i].getWrenchFlow()==RBT_NODE_IN)
                rbtList[i].setWrenchMeasure(fi,mi);
    }

    //now that all is set, we can call solveWrench() or update()...
    return inputWasOk ;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynSensorNode::estimateSensorsWrench(const Matrix &FM, bool afterAttach)
{
    unsigned int inputNode = 0;
    unsigned int outputNode = 0;
    unsigned int numSensor = 0;
    bool inputWasOk = true;
    Vector fi(3); fi.zero();
    Vector mi(3); mi.zero();
    Vector FMi(6);FMi.zero();
    Matrix ret;

    //reset node wrench
    F.zero(); Mu.zero();

    // solve kinematics
    solveKinematics();

    //check how many limbs have wrench input
    inputNode = howManyWrenchInputs(afterAttach);

    //first check if the input is correct
    if(FM.rows()!=6)
    {
        if(verbose) fprintf(stderr,"iDynSensorNode: could not setWrenchMeasure due to wrong sized init wrenches matrix: %zu rows instead of 6 (3+3). Using default values, all zero.\n",FM.rows());
        inputWasOk = false;
    }
    if(FM.cols()!=inputNode)
    {
        if(verbose)
            fprintf(stderr,"iDynSensorNode: could not setWrenchMeasure due to wrong sized init wrenches: %zu instead of %d. Using default values, all zero. \n",FM.cols(),inputNode);
        if(afterAttach==true)
            fprintf(stderr,"                Remember that the first limb receives wrench input during an attach from another node.\n");
        inputWasOk = false;
    }

    //set the input forces/moments from each limb at base/end
    // note: if afterAttach=true we set the wrench only in limbs 1,2,..
    // if afterAttach=false, we set the wrench in all limbs 0,1,2,..
    unsigned int startLimb=0;
    if(afterAttach) startLimb=1;

    if(inputWasOk)
    {
        inputNode = 0;
        for(unsigned int i=startLimb; i<rbtList.size(); i++)
        {
            if(rbtList[i].getWrenchFlow()==RBT_NODE_IN)
            {
                // from the input matrix - read the input wrench
                FMi = FM.getCol(inputNode);
                fi[0]=FMi[0];fi[1]=FMi[1];fi[2]=FMi[2];
                mi[0]=FMi[3];mi[1]=FMi[4];mi[2]=FMi[5];
                inputNode++;
                //set the input wrench in the RBT->limb
                // set on base/end as usual
                rbtList[i].setWrenchMeasure(fi,mi);
            }
        }
    }
    else
    {
        // default zero values if inputs are wrong sized
        for(unsigned int i=startLimb; i<rbtList.size(); i++)
            if(rbtList[i].getWrenchFlow()==RBT_NODE_IN)
                rbtList[i].setWrenchMeasure(fi,mi);
    }

    //first get the forces/moments from each limb
    //assuming that each limb has been properly set with the outcoming measured
    //forces/moments which are necessary for the wrench computation
    for(unsigned int i=0; i<rbtList.size(); i++)
    {
        if(rbtList[i].getWrenchFlow()==RBT_NODE_IN)
        {
            //compute the wrench pass in that limb
            // if there's a sensor, it's the same because here we estimate its value
            rbtList[i].computeLimbWrench();

            //update the node force/moment with the wrench coming from the limb base/end
            // note that getWrench sum the result to F,Mu - because they are passed by reference
            // F = F + F[i], Mu = Mu + Mu[i]
            rbtList[i].getWrench(F,Mu);
            //check
            outputNode++;
        }
    }

    // node summation: already performed by each RBT
    // F = F + F[i], Mu = Mu + Mu[i]

    // at least one output node should exist
    // however if for testing purposes only one limb is attached to the node,
    // we can't avoid the computeWrench phase, but still we must remember that
    // it is not correct, because the node must be in balanc
    if(outputNode==rbtList.size())
    {
        if(verbose>1)
        {
            fprintf(stderr,"iDynSensorNode: warning: there are no limbs with Wrench Flow = Output. ");
            fprintf(stderr," At least one limb must have Wrench Output for balancing forces in the node. ");
            fprintf(stderr,"Please check the coherence of the limb configuration in the node <%s> \n",info.c_str());
        }
    }

    //now forward the wrench output from the node to limbs whose wrench flow is output type
    // assuming they don't have a FT sensor
    for(unsigned int i=0; i<rbtList.size(); i++)
    {
        if(rbtList[i].getWrenchFlow()==RBT_NODE_OUT)
        {
            //init the wrench with the node information
            rbtList[i].setWrench(F,Mu);
            //solve wrench in that limb/chain
            rbtList[i].computeLimbWrench();
        }
    }

    // now look for sensors
    numSensor = 0;
    for(unsigned int i=0; i<rbtList.size(); i++)
    {
        //now we can estimate the sensor wrench if there's a sensor
        // if has sensor, compute its wrench
        if(rbtList[i].isSensorized()==true)
        {
            sensorList[i]->computeSensorForceMoment();
            numSensor++;
        }
    }

    //now build the matrix with wrenches
    if(numSensor>0)
    {
        ret.resize(6,numSensor);
        numSensor=0;
        for(unsigned int i=0; i<rbtList.size(); i++)
            if(rbtList[i].isSensorized()==true)
            {
                ret.setCol(numSensor,sensorList[i]->getSensorForceMoment());
                numSensor++;
            }
    }
    return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
unsigned int iDynSensorNode::howManySensors() const
{
    unsigned int numSensor = 0;
    for(unsigned int i=0; i<rbtList.size(); i++)
    {
        if(rbtList[i].isSensorized()==true)
            numSensor++;
    }
    return numSensor;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//====================================
//
//       iDYN SENSOR TORSO NODE
//
//====================================


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynSensorTorsoNode::iDynSensorTorsoNode(const NewEulMode _mode, unsigned int verb)
:iDynSensorNode("torso_node",_mode,verb)
{

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynSensorTorsoNode::iDynSensorTorsoNode(const string &_info,const NewEulMode _mode, unsigned int verb)
:iDynSensorNode(_info,_mode,verb)
{

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynSensorTorsoNode::~iDynSensorTorsoNode()
{
    delete rightSensor; rightSensor = NULL;
    delete leftSensor;  leftSensor = NULL;
    delete up;          up = NULL;
    delete right;       right = NULL;
    delete left;        left = NULL;

    rbtList.clear();
    sensorList.clear();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynSensorTorsoNode::setInertialMeasure(const Vector &w0, const Vector &dw0, const Vector &ddp0)
{
    return setKinematicMeasure(w0,dw0,ddp0);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynSensorTorsoNode::setSensorMeasurement(const Vector &FM_right, const Vector &FM_left)
{
    Matrix FM(6,2); FM.zero();
    if((FM_right.length()==6)&&(FM_left.length()==6))
    {
        FM.setCol(0,FM_right);
        FM.setCol(1,FM_left);
        return setWrenchMeasure(FM,true);
    }
    else
    {
        if(verbose) fprintf(stderr,"Node <%s> could not set sensor measurements properly due to wrong sized vectors. FM right/left have length %d,%d instead of 6,6. Setting everything to zero. \n",name.c_str(),(int)FM_right.length(),(int)FM_left.length());
        setWrenchMeasure(FM,true);
        return false;
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynSensorTorsoNode::setSensorMeasurement(const Vector &FM_right, const Vector &FM_left, const Vector &FM_up)
{
    Matrix FM(6,3); FM.zero();
    if((FM_right.length()==6)&&(FM_left.length()==6)&&(FM_up.length()==6))
    {
        // order: up 0 - right 1 - left 2
        FM.setCol(0,FM_up);
        FM.setCol(1,FM_right);
        FM.setCol(2,FM_left);
        return setWrenchMeasure(FM,false);
    }
    else
    {
        if(verbose) fprintf(stderr,"Node <%s> could not set sensor measurements properly due to wrong sized vectors. FM up/right/left have length %d,%d,%d instead of 6,6. Setting everything to zero. \n",name.c_str(),(int)FM_up.length(),(int)FM_right.length(),(int)FM_left.length());
        setWrenchMeasure(FM);
        return false;
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynSensorTorsoNode::update()
{
    bool isOk = true;
    isOk = solveKinematics();
    isOk = isOk && solveWrench();
    return isOk;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynSensorTorsoNode::update(const Vector &w0, const Vector &dw0, const Vector &ddp0, const Vector &FM_right, const Vector &FM_left, const Vector &FM_up)
{
    bool inputOk = true;

    if((FM_right.length()==6)&&(FM_left.length()==6)&&(FM_up.length()==6)&&(w0.length()==3)&&(dw0.length()==3)&&(ddp0.length()==3))
    {
        setInertialMeasure(w0,dw0,ddp0);
        setSensorMeasurement(FM_right,FM_left,FM_up);
        return update();
    }
    else
    {
        if(verbose)
        {
            fprintf(stderr,"Node <%s> error, could not update() due to wrong sized vectors. ",name.c_str());
            fprintf(stderr," w0,dw0,ddp0 have size %d,%d,%d instead of 3,3,3. \n",(int)w0.length(),(int)dw0.length(),(int)ddp0.length());
            fprintf(stderr," FM up/right/left have size %d,%d,%d instead of 6,6,6. \n",(int)FM_up.length(),(int)FM_right.length(),(int)FM_left.length());
            fprintf(stderr," Updating without new values.\n");
        }
        update();
        return false;
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynSensorTorsoNode::update(const Vector &FM_right, const Vector &FM_left, bool afterAttach)
{
    if(afterAttach==false)
    {
        if(verbose)
        {
            fprintf(stderr,"Node <%s> error, could not update() due to missing wrench vectors. ",name.c_str());
            fprintf(stderr,"This type of update() only works after an attachTorso() or after having set the central limb wrench and kinematics variables. ");
            fprintf(stderr,"You should try with the other update() methods. ");
            fprintf(stderr,"Could not perform update(): exiting. \n");
        }
        return false;
    }

    if((FM_right.length()==6)&&(FM_left.length()==6))
    {
        Matrix FM(6,2);
        FM.setCol(0,FM_right);
        FM.setCol(1,FM_left);
        setWrenchMeasure(FM,true);
        return update();
    }
    else
    {
        if(verbose)
        {
            fprintf(stderr,"Node <%s> error, could not update() due to wrong sized vectors. ",name.c_str());
            fprintf(stderr,"FM right/left have size %d,%d instead of 6,6. Updating without new values. \n",(int)FM_right.length(),(int)FM_left.length());
        }
        update();
        return false;
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //----------------
    //      GET
    //----------------

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynSensorTorsoNode::getForces(const string &limbType)
{
    if(limbType==up_name)               return up->getForces();
    else if(limbType==left_name)        return left->getForces();
    else if(limbType==right_name)       return right->getForces();
    else
    {
        if(verbose) fprintf(stderr,"Node <%s> there's not a limb named %s. Only %s,%s,%s are available. \n",name.c_str(),limbType.c_str(),left_name.c_str(),right_name.c_str(),up_name.c_str());
        return Matrix(0,0);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynSensorTorsoNode::getMoments(const string &limbType)
{
    if(limbType==up_name)           return up->getMoments();
    else if(limbType==left_name)    return left->getMoments();
    else if(limbType==right_name)   return right->getMoments();
    else
    {
        if(verbose) fprintf(stderr,"Node <%s> there's not a limb named %s. Only %s,%s,%s are available. \n",name.c_str(),limbType.c_str(),left_name.c_str(),right_name.c_str(),up_name.c_str());
        return Matrix(0,0);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::getTorques(const string &limbType)
{
    if(limbType==up_name)           return up->getTorques();
    else if(limbType==left_name)    return left->getTorques();
    else if(limbType==right_name)   return right->getTorques();
    else
    {
        if(verbose) fprintf(stderr,"Node <%s> there's not a limb named %s. Only %s,%s,%s are available. \n",name.c_str(),limbType.c_str(),left_name.c_str(),right_name.c_str(),up_name.c_str());
        return Vector(0);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::getTorsoForce() const { return F;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::getTorsoMoment() const{ return Mu;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::getTorsoAngVel() const{ return w;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::getTorsoAngAcc() const{ return dw;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::getTorsoLinAcc() const{ return ddp;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool   iDynSensorTorsoNode::computeCOM()
{
    unsigned int i=0;
    yarp::sig::Vector COM_UP(4); COM_UP.zero();
    yarp::sig::Vector COM_RT(4); COM_RT.zero();
    yarp::sig::Vector COM_LF(4); COM_LF.zero();
    total_COM_UP.resize(4);total_COM_UP.zero();
    total_COM_RT.resize(4);total_COM_RT.zero();
    total_COM_LF.resize(4);total_COM_LF.zero();
    total_mass_UP=0.0;
    total_mass_RT=0.0;
    total_mass_LF=0.0;

    iCub::iDyn::iDynLimb *limb =       0;
    yarp::sig::Matrix    *orig =       0;
    yarp::sig::Vector    *total_COM  = 0;
    double               *total_mass = 0;

    for (int n=0; n<3; n++)
    {
        switch (n)
        {
            case 0:
                limb =        (this->left);
                orig =       &(this->HLeft);
                total_COM  = &(total_COM_LF);
                total_mass = &(total_mass_LF);
            break;
            case 1:
                limb =        (this->right);
                orig =       &(this->HRight);
                total_COM  = &(total_COM_RT);
                total_mass = &(total_mass_RT);
            break;
            case 2:
                limb =        (this->up);
                orig =       &(this->HUp);
                total_COM  = &(total_COM_UP);
                total_mass = &(total_mass_UP);
            break;
        }

        for (i=0; i<limb->getN(); i++)
        {
            (*total_mass)=(*total_mass)+limb->getMass(i);
            COM=limb->getCOM(i).getCol(3);
                yarp::sig::Matrix m = limb->getH(i, true);
            (*total_COM) = (*total_COM) + ((*orig) * m * COM) * limb->getMass(i);
        }
        if (fabs(*total_mass) > 0.00001)
            {(*total_COM)=(*total_COM)/(*total_mass);  }
        else
            {(*total_COM).zero();}
    }
    return true;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool   iDynSensorTorsoNode::EXPERIMENTAL_computeCOMjacobian()
{
    iCub::iDyn::iDynLimb *limb =       0;
    yarp::sig::Matrix    *orig =       0;
    yarp::sig::Matrix    *COM_jacob  = 0;
    double               *total_mass = 0;


    for (int n=0; n<3; n++)
    {
        switch (n)
        {
            case 0:
                limb =        (this->left);
                orig =       &(this->HLeft);
                COM_jacob  = &(COM_jacob_LF);
                total_mass = &(total_mass_LF);
            break;
            case 1:
                limb =        (this->right);
                orig =       &(this->HRight);
                COM_jacob  = &(COM_jacob_RT);
                total_mass = &(total_mass_RT);
            break;
            case 2:
                limb =        (this->up);
                orig =       &(this->HUp);
                COM_jacob  = &(COM_jacob_UP);
                total_mass = &(total_mass_UP);
            break;
        }

        //here do the computation
        double* partial_mass = 0;
        Vector* partial_COM  = 0;
        partial_mass = new double [limb->getN()];
        partial_COM  = new Vector [limb->getN()];
        COM_jacob->resize(6,limb->getN());

        //obtain the list of Z vectors (used in the final step)
        deque<Matrix> intH;
        intH.push_back(*orig);
        for (unsigned int i=0; i<limb->getN(); i++)
            intH.push_back((*orig)*limb->getH(i,true));

        for (unsigned int iLink=0; iLink<limb->getN(); iLink++)
        {
            partial_mass[iLink]=0;
            partial_COM [iLink].resize(4);
            partial_COM [iLink].zero();

            // this is the transformation from the link root to current link iLink
            yarp::sig::Matrix m = (*orig) * limb->getH(iLink, true);

            //partial COMs are computed in this block
            for (unsigned int i=iLink; i<limb->getN(); i++)
            {
                yarp::sig::Matrix m_i = (*orig) * limb->getH(i, true);
                partial_COM[iLink] =partial_COM[iLink] + m_i * limb->getCOM(i).getCol(3) * limb->getMass(i);
                partial_mass[iLink]=partial_mass[iLink]+limb->getMass(i);
                if(partial_mass[iLink]==0) partial_mass[iLink]=1e-15;
            }
            partial_COM[iLink] = (partial_COM[iLink]/partial_mass[iLink])-intH[iLink].getCol(3);

#ifdef DEBUG_P_COM
            printf ("parCOM: %d %+3.3f %+3.3f %+3.3f \n", iLink, partial_COM[iLink](0), partial_COM[iLink](1), partial_COM[iLink](2));
            printf ("n     : %d %+3.3f %+3.3f %+3.3f \n", iLink, intH[iLink].getCol(3)(0),intH[iLink].getCol(3)(1),intH[iLink].getCol(3)(2));
            printf ("p-nCOM: %d %+3.3f %+3.3f %+3.3f \n", iLink, partial_COM[iLink](0), partial_COM[iLink](1), partial_COM[iLink](2));
#endif
            //partial COM jacobian are computed in this block
            double mass_coeff = partial_mass[iLink]/(*total_mass);

            Vector Z = intH[iLink].subcol(0,2,3);
            Vector w = cross(Z,partial_COM[iLink].subVector(0,2));

            (*COM_jacob)(0,iLink)=mass_coeff*w[0];
            (*COM_jacob)(1,iLink)=mass_coeff*w[1];
            (*COM_jacob)(2,iLink)=mass_coeff*w[2];
            (*COM_jacob)(3,iLink)=Z(0);
            (*COM_jacob)(4,iLink)=Z(1);
            (*COM_jacob)(5,iLink)=Z(2);
        }

        delete [] partial_mass;
        delete [] partial_COM;
    }
    return true;
}

    //------------------
    //    LIMB CALLS
    //------------------

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::setAng(const string &limbType, const Vector &_q)
{
    if(limbType==up_name)           return up->setAng(_q);
    else if(limbType==left_name)    return left->setAng(_q);
    else if(limbType==right_name)   return right->setAng(_q);
    else
    {
        if(verbose) fprintf(stderr,"Node <%s> there's not a limb named %s. Only %s,%s,%s are available. \n",name.c_str(),limbType.c_str(),left_name.c_str(),right_name.c_str(),up_name.c_str());
        return Vector(0);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::getAng(const string &limbType)
{
    if(limbType==up_name)           return up->getAng();
    else if(limbType==left_name)    return left->getAng();
    else if(limbType==right_name)   return right->getAng();
    else
    {
        if(verbose) fprintf(stderr,"Node <%s> there's not a limb named %s. Only %s,%s,%s are available. \n",name.c_str(),limbType.c_str(),left_name.c_str(),right_name.c_str(),up_name.c_str());
        return Vector(0);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynSensorTorsoNode::setAng(const string &limbType, const unsigned int i, double _q)
{
    if(limbType==up_name)           return up->setAng(i,_q);
    else if(limbType==left_name)    return left->setAng(i,_q);
    else if(limbType==right_name)   return right->setAng(i,_q);
    else
    {
        if(verbose) fprintf(stderr,"Node <%s> there's not a limb named %s. Only %s,%s,%s are available. \n",name.c_str(),limbType.c_str(),left_name.c_str(),right_name.c_str(),up_name.c_str());
        return 0.0;
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynSensorTorsoNode::getAng(const string &limbType, const unsigned int i)
{
    if(limbType==up_name)           return up->getAng(i);
    else if(limbType==left_name)    return left->getAng(i);
    else if(limbType==right_name)   return right->getAng(i);
    else
    {
        if(verbose) fprintf(stderr,"Node <%s> there's not a limb named %s. Only %s,%s,%s are available. \n",name.c_str(),limbType.c_str(),left_name.c_str(),right_name.c_str(),up_name.c_str());
        return 0.0;
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::setDAng(const string &limbType, const Vector &_dq)
{
    if(limbType==up_name)           return up->setDAng(_dq);
    else if(limbType==left_name)    return left->setDAng(_dq);
    else if(limbType==right_name)   return right->setDAng(_dq);
    else
    {
        if(verbose) fprintf(stderr,"Node <%s> there's not a limb named %s. Only %s,%s,%s are available. \n",name.c_str(),limbType.c_str(),left_name.c_str(),right_name.c_str(),up_name.c_str());
        return Vector(0);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::getDAng(const string &limbType)
{
    if(limbType==up_name)           return up->getDAng();
    else if(limbType==left_name)    return left->getDAng();
    else if(limbType==right_name)   return right->getDAng();
    else
    {
        if(verbose) fprintf(stderr,"Node <%s> there's not a limb named %s. Only %s,%s,%s are available. \n",name.c_str(),limbType.c_str(),left_name.c_str(),right_name.c_str(),up_name.c_str());
        return Vector(0);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynSensorTorsoNode::setDAng(const string &limbType, const unsigned int i, double _dq)
{
    if(limbType==up_name)           return up->setDAng(i,_dq);
    else if(limbType==left_name)    return left->setDAng(i,_dq);
    else if(limbType==right_name)   return right->setDAng(i,_dq);
    else
    {
        if(verbose) fprintf(stderr,"Node <%s> there's not a limb named %s. Only %s,%s,%s are available. \n",name.c_str(),limbType.c_str(),left_name.c_str(),right_name.c_str(),up_name.c_str());
        return 0.0;
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynSensorTorsoNode::getDAng(const string &limbType, const unsigned int i)
{
    if(limbType==up_name)           return up->getDAng(i);
    else if(limbType==left_name)    return left->getDAng(i);
    else if(limbType==right_name)   return right->getDAng(i);
    else
    {
        if(verbose) fprintf(stderr,"Node <%s> there's not a limb named %s. Only %s,%s,%s are available. \n",name.c_str(),limbType.c_str(),left_name.c_str(),right_name.c_str(),up_name.c_str());
        return 0.0;
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::setD2Ang(const string &limbType, const Vector &_ddq)
{
    if(limbType==up_name)           return up->setD2Ang(_ddq);
    else if(limbType==left_name)    return left->setD2Ang(_ddq);
    else if(limbType==right_name)   return right->setD2Ang(_ddq);
    else
    {
        if(verbose) fprintf(stderr,"Node <%s> there's not a limb named %s. Only %s,%s,%s are available. \n",name.c_str(),limbType.c_str(),left_name.c_str(),right_name.c_str(),up_name.c_str());
        return Vector(0);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::getD2Ang(const string &limbType)
{
    if(limbType==up_name)           return up->getD2Ang();
    else if(limbType==left_name)    return left->getD2Ang();
    else if(limbType==right_name)   return right->getD2Ang();
    else
    {
        if(verbose) fprintf(stderr,"Node <%s> there's not a limb named %s. Only %s,%s,%s are available. \n",name.c_str(),limbType.c_str(),left_name.c_str(),right_name.c_str(),up_name.c_str());
        return Vector(0);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynSensorTorsoNode::setD2Ang(const string &limbType, const unsigned int i, double _ddq)
{
    if(limbType==up_name)           return up->setD2Ang(i,_ddq);
    else if(limbType==left_name)    return left->setD2Ang(i,_ddq);
    else if(limbType==right_name)   return right->setD2Ang(i,_ddq);
    else
    {
        if(verbose) fprintf(stderr,"Node <%s> there's not a limb named %s. Only %s,%s,%s are available. \n",name.c_str(),limbType.c_str(),left_name.c_str(),right_name.c_str(),up_name.c_str());
        return 0.0;
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynSensorTorsoNode::getD2Ang(const string &limbType, const unsigned int i)
{
    if(limbType==up_name)           return up->getD2Ang(i);
    else if(limbType==left_name)    return left->getD2Ang(i);
    else if(limbType==right_name)   return right->getD2Ang(i);
    else
    {
        if(verbose) fprintf(stderr,"Node <%s> there's not a limb named %s. Only %s,%s,%s are available. \n",name.c_str(),limbType.c_str(),left_name.c_str(),right_name.c_str(),up_name.c_str());
        return 0.0;
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
unsigned int iDynSensorTorsoNode::getNLinks(const string &limbType) const
{
    if(limbType==up_name)           return up->getN();
    else if(limbType==left_name)    return left->getN();
    else if(limbType==right_name)   return right->getN();
    else
    {
        if(verbose) fprintf(stderr,"Node <%s> there's not a limb named %s. Only %s,%s,%s are available. \n",name.c_str(),limbType.c_str(),left_name.c_str(),right_name.c_str(),up_name.c_str());
        return 0;
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynSensorTorsoNode::clearContactList()
{
    leftSensor->clearContactList();
    rightSensor->clearContactList();
}



//====================================
//
//          UPPER TORSO
//
//====================================


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubUpperTorso::iCubUpperTorso(version_tag _tag, const NewEulMode _mode, unsigned int verb)
:iDynSensorTorsoNode("upper-torso",_mode,verb)
{
    tag=_tag;
    build();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCubUpperTorso::build()
{
    left    = new iCubArmNoTorsoDyn("left",KINFWD_WREBWD);
    right   = new iCubArmNoTorsoDyn("right",KINFWD_WREBWD);
    if (tag.head_version == 2) {
        std::string ver {"v2."};
        ver += std::to_string(tag.head_subversion);
        up     = new iCubNeckInertialDynV2(KINBWD_WREBWD, ver);
    }
    else {
        up     = new iCubNeckInertialDyn(KINBWD_WREBWD);
    }

    SensorLinkNewtonEuler* armLeftSensor = new iCubArmSensorLink("left", mode, verbose);
    SensorLinkNewtonEuler* armRightSensor = new iCubArmSensorLink("right", mode, verbose);
    // FT sensor is in position 2 in the kinematic chain in both arms
    // note position 5 if with torso, 2 without torso
    unsigned int SENSOR_LINK_INDEX = 2;

    leftSensor = new iDynContactSolver(dynamic_cast<iCubArmNoTorsoDyn*>(left), SENSOR_LINK_INDEX, armLeftSensor,
        "leftArmContactSolver",mode,LEFT_ARM, verbose);
    rightSensor = new iDynContactSolver(dynamic_cast<iCubArmNoTorsoDyn*>(right), SENSOR_LINK_INDEX, armRightSensor,
        "rightArmContactSolver",mode,RIGHT_ARM, verbose);

    HUp.resize(4,4);    HUp.eye();
    HLeft.resize(4,4);  HLeft.zero();
    HRight.resize(4,4); HRight.zero();

    double theta = CTRL_DEG2RAD * (180.0-15.0);
    /*
    HLeft(0,0) = cos(theta);    HLeft(0,1) = 0.0;       HLeft(0,2) = sin(theta);    HLeft(0,3) = -24.8786e-3;
    HLeft(1,0) = 0.0;           HLeft(1,1) = 1.0;       HLeft(1,2) = 0.0;           HLeft(1,3) = -50.0000e-3;
    HLeft(2,0) = -sin(theta);   HLeft(2,1) = 0.0;       HLeft(2,2) = cos(theta);    HLeft(2,3) =  -6.0472e-3;
    HLeft(3,3) = 1.0;
    HRight(0,0) = -cos(theta);  HRight(0,1) = 0.0;  HRight(0,2) = -sin(theta);  HRight(0,3) = -24.8786e-3;
    HRight(1,0) = 0.0;          HRight(1,1) = -1.0; HRight(1,2) = 0.0;          HRight(1,3) = -50.0000e-3;
    HRight(2,0) = -sin(theta);  HRight(2,1) = 0.0;  HRight(2,2) = cos(theta);   HRight(2,3) =   6.0472e-3;
    HRight(3,3) = 1.0;
    */

    HLeft(0,0) = cos(theta);    HLeft(0,1) = 0.0;       HLeft(0,2) = sin(theta);    HLeft(0,3) =  0.00294;
    HLeft(1,0) = 0.0;           HLeft(1,1) = 1.0;       HLeft(1,2) = 0.0;           HLeft(1,3) = -0.050;
    HLeft(2,0) = -sin(theta);   HLeft(2,1) = 0.0;       HLeft(2,2) = cos(theta);    HLeft(2,3) = -0.11026;
    HLeft(3,3) = 1.0;
    HRight(0,0) = -cos(theta);  HRight(0,1) = 0.0;  HRight(0,2) = -sin(theta);  HRight(0,3) =  0.00294;
    HRight(1,0) = 0.0;          HRight(1,1) = -1.0; HRight(1,2) = 0.0;          HRight(1,3) = -0.050;
    HRight(2,0) = -sin(theta);  HRight(2,1) = 0.0;  HRight(2,2) = cos(theta);   HRight(2,3) =  0.11026;
    HRight(3,3) = 1.0;

    // order: head - right arm - left arm
    addLimb(up,HUp,RBT_NODE_IN,RBT_NODE_IN);
    addLimb(right,HRight,rightSensor,RBT_NODE_OUT,RBT_NODE_IN);
    addLimb(left,HLeft,leftSensor,RBT_NODE_OUT,RBT_NODE_IN);

    left_name = "left_arm";
    right_name= "right_arm";
    up_name   = "head";
    name      = "upper_torso";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubUpperTorso::~iCubUpperTorso()
{
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//====================================
//
//          LOWER TORSO
//
//====================================


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubLowerTorso::iCubLowerTorso(version_tag _tag, const NewEulMode _mode, unsigned int verb)
:iDynSensorTorsoNode("lower-torso",_mode,verb)
{
    tag=_tag;
    build();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCubLowerTorso::build()
{

    SensorLinkNewtonEuler* legLeftSensor = new iCubLegSensorLink("left", mode, verbose);
    SensorLinkNewtonEuler* legRightSensor = new iCubLegSensorLink("right", mode, verbose);
    unsigned int SENSOR_LINK_INDEX = 1;

    if (tag.legs_version == 2)
    {
        left        = new iCubLegDynV2("left",KINFWD_WREBWD);
        right       = new iCubLegDynV2("right",KINFWD_WREBWD);

        leftSensor = new iDynContactSolver(dynamic_cast<iCubLegDynV2*>(left), SENSOR_LINK_INDEX, legLeftSensor,
        "leftLegContactSolver",mode,LEFT_LEG,verbose);
        rightSensor = new iDynContactSolver(dynamic_cast<iCubLegDynV2*>(right), SENSOR_LINK_INDEX, legRightSensor,
        "rightLegContactSolver",mode,RIGHT_LEG,verbose);
    }
    else
    {
        left    = new iCubLegDyn("left",KINFWD_WREBWD);
        right   = new iCubLegDyn("right",KINFWD_WREBWD);

        leftSensor = new iDynContactSolver(dynamic_cast<iCubLegDyn*>(left), SENSOR_LINK_INDEX, legLeftSensor,
        "leftLegContactSolver",mode,LEFT_LEG,verbose);
        rightSensor = new iDynContactSolver(dynamic_cast<iCubLegDyn*>(right), SENSOR_LINK_INDEX, legRightSensor,
        "rightLegContactSolver",mode,RIGHT_LEG,verbose);
    }

    up      = new iCubTorsoDyn("lower",KINBWD_WREBWD);


    HUp.resize(4,4);    HUp.zero();
    HUp(0,1)=-1.0;  // 0 -1  0
    HUp(1,2)=-1.0;  // 0  0 -1
    HUp(2,0)= 1.0;  // 1  0  0
    HUp(3,3)= 1.0;

    HLeft.resize(4,4);  HRight.resize(4,4);
    HLeft.zero();       HRight.zero();

    HLeft(0,0)=1.0;         //  1  0  0
    HLeft(1,2)=1.0;         //  0  0  1
    HLeft(2,1)=-1.0;        //  0 -1  0
    HLeft(3,3)=1.0;         //
    HLeft(2,3)=-0.1199; HLeft(1,3)=-0.0681;

    HRight(0,0)=1.0;        //  1  0  0
    HRight(1,2)=1.0;        //  0  0  1
    HRight(2,1)=-1.0;       //  0 -1  0
    HRight(3,3)=1.0;        //
    HRight(2,3)=-0.1199;HRight(1,3)=0.0681;

    // order: torso - right leg - left leg
    addLimb(up,HUp,RBT_NODE_IN,RBT_NODE_IN);
    addLimb(right,HRight,rightSensor,RBT_NODE_OUT,RBT_NODE_IN);
    addLimb(left,HLeft,leftSensor,RBT_NODE_OUT,RBT_NODE_IN);

    left_name = "left_leg";
    right_name= "right_leg";
    up_name   = "torso";
    name      = "lower_torso";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubLowerTorso::~iCubLowerTorso()
{
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//====================================
//
//          iCUB WHOLE BODY
//
//====================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubWholeBody::iCubWholeBody(version_tag _tag, const NewEulMode mode, unsigned int verbose)
{
    //create all limbs
    tag = _tag;
    upperTorso = new iCubUpperTorso(tag,mode,verbose);
    lowerTorso = new iCubLowerTorso(tag,mode,verbose);

    //now create a connection between upperTorso node and Torso ( lowerTorso->up == Torso )
    Matrix H(4,4);
    H.eye();
    //H  is no used currently since the transformation is an identity
    rbt = new RigidBodyTransformation(lowerTorso->up,H,"connection between lower and upper torso",false,RBT_NODE_OUT,RBT_NODE_OUT,mode,verbose);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubWholeBody::~iCubWholeBody()
{
    if (upperTorso) delete upperTorso; upperTorso = NULL;
    if (lowerTorso) delete lowerTorso; lowerTorso = NULL;
    if (rbt)        delete rbt;        rbt        = NULL;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCubWholeBody::attachLowerTorso(const Vector &FM_right_leg, const Vector &FM_left_leg)
{
    Vector in_w   = upperTorso->getTorsoAngVel();
    Vector in_dw  = upperTorso->getTorsoAngAcc();
    Vector in_ddp = upperTorso->getTorsoLinAcc();
    Vector in_F   = upperTorso->getTorsoForce();
    Vector in_M   = upperTorso->getTorsoMoment();

    Vector out_w;   out_w.resize(3);
    Vector out_dw;  out_dw.resize(3);
    Vector out_ddp; out_ddp.resize(3);
    Vector out_F;   out_F.resize(3);
    Vector out_M;   out_M.resize(3);

    //kinematics:
    out_w[0]= in_w[0];
    out_w[1]= in_w[1];
    out_w[2]= in_w[2];

    out_dw[0]= in_dw[0];
    out_dw[1]= in_dw[1];
    out_dw[2]= in_dw[2];

    out_ddp[0]= in_ddp[0];
    out_ddp[1]= in_ddp[1];
    out_ddp[2]= in_ddp[2];

    //wrenches:
    out_F[0]= in_F[0];
    out_F[1]= in_F[1];
    out_F[2]= in_F[2];

    out_M[0]= in_M[0];
    out_M[1]= in_M[1];
    out_M[2]= in_M[2];

    Vector FUP ;
    Vector FLL ;
    Vector FRL ;
    FUP.resize(6);
    FUP[0] = out_F[0];
    FUP[1] = out_F[1];
    FUP[2] = out_F[2];
    FUP[3] = out_M[0];
    FUP[4] = out_M[1];
    FUP[5] = out_M[2];
    lowerTorso->setKinematicMeasure(out_w,out_dw,out_ddp);
    lowerTorso->setSensorMeasurement(FM_right_leg,FM_left_leg,FUP);

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCubWholeBody::computeCOM()
{
    //upper torso COM computation
    yarp::sig::Matrix T0 = lowerTorso->HUp;
    yarp::sig::Matrix T1 = lowerTorso->up->getH(2,true);

    upperTorso->computeCOM();

    upper_mass =   upperTorso->total_mass_UP
                  +upperTorso->total_mass_LF
                  +upperTorso->total_mass_RT;

    upper_COM = (upperTorso->total_COM_UP * upperTorso->total_mass_UP+
                 upperTorso->total_COM_LF * upperTorso->total_mass_LF+
                 upperTorso->total_COM_RT * upperTorso->total_mass_RT)/ upper_mass;
    //upper_COM.push_back(1.0);
    upper_COM= T0 * T1 * upper_COM;

    //lower torso COM computation
    lowerTorso->computeCOM();

    lower_mass =   lowerTorso->total_mass_UP
                  +lowerTorso->total_mass_LF
                  +lowerTorso->total_mass_RT;

    lower_COM  = (lowerTorso->total_COM_UP * lowerTorso->total_mass_UP+
                  lowerTorso->total_COM_LF * lowerTorso->total_mass_LF+
                  lowerTorso->total_COM_RT * lowerTorso->total_mass_RT)/ lower_mass;

    whole_mass = lower_mass+upper_mass;
    whole_COM  = ((upper_COM*upper_mass)+(lower_COM*lower_mass))/(upper_mass+lower_mass);
    whole_COM.pop_back();

    sw_getcom = 1; //USEFUL FOR THE DOUBLE CALL TO getCOMjacobian()
    return true;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCubWholeBody::getCOM(BodyPart which_part, Vector &COM, double & mass)
{
    COM.zero();
    mass=0.0;
    yarp::sig::Matrix T0;
    yarp::sig::Matrix T1;
    yarp::sig::Matrix hright;

    // hright.resize(4,4);  hright.zero();
    // double theta = CTRL_DEG2RAD * (180.0-15.0);
    // hright(0,0)=1.0;     //  1  0  0
    // hright(1,2)=1.0;     //  0  0  1
    // hright(2,1)=-1.0;    //  0 -1  0
    // hright(3,3)=1.0;
    // hright(2,3)=-0.1199;
    // hright(1,3)=0.0681;
    // yarp::sig::Matrix rot6x6; rot6x6.resize(6,6); rot6x6.zero();
    // rot6x6.setSubmatrix(hright.submatrix(0,2,0,2),0,0);
    // rot6x6.setSubmatrix(hright.submatrix(0,2,0,2),3,3);

    switch (which_part)
    {
        case BODY_PART_ALL:
            COM=whole_COM;
            mass=whole_mass;

            #ifdef DEBUG_FOOT_COM
                fprintf(stderr, "Whole COM being computed w.r.t Foot Reference Frame... \n");
            #endif

        break;
        case LOWER_BODY_PARTS:
            COM=this->lower_COM;
            mass=lower_mass;
            COM.pop_back();
        break;
        case UPPER_BODY_PARTS:
            //T0 = lowerTorso->HUp;
            //T1 = lowerTorso->up->getH(2,true);
            //COM=this->upper_COM; //respect to neck
            //COM= T0 * T1 * this->upper_COM; //respect to root
            COM= this->upper_COM; //rotation is done aslread in computation?????
            mass=upper_mass;
            COM.pop_back();
        break;
        case LEFT_LEG:
            COM=this->lowerTorso->total_COM_LF;
            mass=this->lowerTorso->total_mass_LF;
            COM.pop_back();
        break;
        case RIGHT_LEG:
            COM=this->lowerTorso->total_COM_RT;
            mass=this->lowerTorso->total_mass_RT;
            COM.pop_back();
        break;
        case TORSO:
            COM=this->lowerTorso->total_COM_UP;
            mass=this->lowerTorso->total_mass_UP;
            COM.pop_back();
        break;
        case LEFT_ARM:
            T0 = lowerTorso->HUp;
            T1 = lowerTorso->up->getH(2,true);
            mass=this->upperTorso->total_mass_LF;
            COM=this->upperTorso->total_COM_LF;
            //COM.push_back(1.0);
            COM= T0 * T1 * COM;
            COM.pop_back();
        break;
        case RIGHT_ARM:
            T0 = lowerTorso->HUp;
            T1 = lowerTorso->up->getH(2,true);
            mass=this->upperTorso->total_mass_RT;
            COM=this->upperTorso->total_COM_RT;
            //COM.push_back(1.0);
            COM= T0 * T1 * COM;
            COM.pop_back();
        break;
        case HEAD:
            T0 = lowerTorso->HUp;
            T1 = lowerTorso->up->getH(2,true);
            mass=this->upperTorso->total_mass_UP;
            COM=this->upperTorso->total_COM_UP;
            //COM.push_back(1.0);
            COM= T0 * T1 * COM;
            COM.pop_back();
        break;
        default:
            COM.zero();
            mass=0.0;
        break;
    }
    return true;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCubWholeBody::getAllPositions(Vector &pos)
{
    Vector ll = this->lowerTorso->left->getAng();
    Vector rl = this->lowerTorso->right->getAng();
    Vector to = this->lowerTorso->up->getAng();
    Vector la = this->upperTorso->left->getAng();
    Vector ra = this->upperTorso->right->getAng();
    Vector hd = this->upperTorso->up->getAng();
    pos.clear();
    pos.resize(32,0.0);
    int i=0; int j=0;
    for (i=0; i<6; i++, j++) pos[j] = ll[i];
    for (i=0; i<6; i++, j++) pos[j] = rl[i];
    for (i=0; i<3; i++, j++) pos[j] = to[i];
    for (i=0; i<7; i++, j++) pos[j] = la[i];
    for (i=0; i<7; i++, j++) pos[j] = ra[i];
    for (i=0; i<3; i++, j++) pos[j] = hd[i];

    return true;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCubWholeBody::getAllVelocities(Vector &vel)
{
    Vector ll = this->lowerTorso->left->getDAng();
    Vector rl = this->lowerTorso->right->getDAng();
    Vector to = this->lowerTorso->up->getDAng();
    Vector la = this->upperTorso->left->getDAng();
    Vector ra = this->upperTorso->right->getDAng();
    Vector hd = this->upperTorso->up->getDAng();
    vel.clear();
    vel.resize(32,0.0);
    int i=0; int j=0;
    for (i=0; i<6; i++, j++) vel[j] = ll[i];
    for (i=0; i<6; i++, j++) vel[j] = rl[i];
    for (i=0; i<3; i++, j++) vel[j] = to[i];
    for (i=0; i<7; i++, j++) vel[j] = la[i];
    for (i=0; i<7; i++, j++) vel[j] = ra[i];
    for (i=0; i<3; i++, j++) vel[j] = hd[i];

    return true;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*
bool iCubWholeBody::getAllAccelerations(Vector &acc)
{
    Vector ll = this->lowerTorso->left->getD2Ang();
    Vector rl = this->lowerTorso->right->getD2Ang();
    Vector to = this->lowerTorso->up->getD2Ang();
    Vector la = this->upperTorso->left->getD2Ang();
    Vector ra = this->upperTorso->right->getD2Ang();
    Vector hd = this->upperTorso->up->getD2Ang();
    acc.clear();
    acc.resize(32,0.0);
    int i=0; int j=0;
    for (i=0; i<6; i++, j++) acc[j] = ll[i];
    for (i=0; i<6; i++, j++) acc[j] = rl[i];
    for (i=0; i<3; i++, j++) acc[j] = to[i];
    for (i=0; i<7; i++, j++) acc[j] = la[i];
    for (i=0; i<7; i++, j++) acc[j] = ra[i];
    for (i=0; i<3; i++, j++) acc[j] = hd[i];

    return true;
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCubWholeBody::EXPERIMENTAL_computeCOMjacobian()
{
    //prepare the transformation matrices
    yarp::sig::Matrix T0 = lowerTorso->HUp;
    yarp::sig::Matrix T1 = lowerTorso->up->getH(2,true);

    int r=0;
    int ct=0;
    int c=0;

    //upper torso COM jacobian computation
    upperTorso->EXPERIMENTAL_computeCOMjacobian();

    //lower torso COM jacobian computation
    lowerTorso->EXPERIMENTAL_computeCOMjacobian();

    //order the partial jacobians in the main jacobian.
    this->COM_Jacob.resize(6,32);
    COM_Jacob.zero();

    //Adding Lower Torso columns
    for (r=0; r<3; r++)
        {
            ct = 0;
            //left_leg
            for (c=0; c<6; c++, ct++)
                COM_Jacob(r,ct) = lowerTorso->COM_jacob_LF(r,c);
            //right leg
            for (c=0; c<6; c++, ct++)
                COM_Jacob(r,ct) = lowerTorso->COM_jacob_RT(r,c);
            //torso
            for (c=0; c<3; c++, ct++)
                COM_Jacob(r,ct) = lowerTorso->COM_jacob_UP(r,c);
        }

    Matrix tmp_Jac;
    tmp_Jac.resize(6,32);


    /*Upper body Jacobian Matrices (arms and head) require a transformation from NECK reference frame to ROOT
      reference frame. In order to get these velocity transform Jacobians to ROOT we have to multiply each by
      a 6x6 matrix containing the rotational matrix from T0*T1 (Transformation matrix from NECK to ROOT). */

    //Velocity Transform Jacobian
    Matrix T_ad;
    T_ad.resize(6,6);
    T_ad = adjoint(T0*T1);
    Matrix RotZero; RotZero.resize(3,3); RotZero.zero();
    T_ad.setSubmatrix(RotZero,0,3);

    tmp_Jac.zero();
    tmp_Jac.setSubmatrix(T_ad*upperTorso->COM_jacob_LF,0,15);
    Matrix larm = T_ad*upperTorso->COM_jacob_LF;
    COM_Jacob+=tmp_Jac;

    tmp_Jac.zero();
    tmp_Jac.setSubmatrix(T_ad*upperTorso->COM_jacob_RT,0,22);
    Matrix rarm = T_ad*upperTorso->COM_jacob_RT;
    COM_Jacob+=tmp_Jac;

    tmp_Jac.zero();
    yarp::sig::Matrix COMHead = upperTorso->COM_jacob_UP;
    tmp_Jac.setSubmatrix(T_ad*COMHead.submatrix(0,5,0,2),0,29);

    Matrix mm = T_ad*upperTorso->COM_jacob_UP;
    COM_Jacob+=tmp_Jac;

    return true;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCubWholeBody::EXPERIMENTAL_getCOMjacobian(BodyPart which_part, Matrix &jac)
{
    jac = COM_Jacob;
    Matrix T_rotT0 = adjoint(lowerTorso->HUp);
    Matrix RotZero; RotZero.resize(3,3); RotZero.zero();
    T_rotT0.setSubmatrix(RotZero,0,3);
    // T_rotT0 is a similar Transformation matrix for putting the Torso GeoJacobian from its base link
    // reference frame to the ROOT reference frame.
    Matrix Jac_Torso = T_rotT0*lowerTorso->up->GeoJacobian();

        Matrix T0 = lowerTorso->HUp;
        Matrix T1 = lowerTorso->up->getH(2,true);
        Matrix H_T0T1; H_T0T1.resize(4,4); H_T0T1 = T0*T1;
        Matrix rotT0T1; rotT0T1.resize(3,3); rotT0T1 = H_T0T1.submatrix(0,2,0,2);
        // Left Arm COM seen from Root RcL i.e. R*NcL where NcL is the COM vector of the left arm w.r.t. Neck
        Vector LFcom; LFcom = upperTorso->total_COM_LF;
        Vector RcL = rotT0T1*(LFcom.subVector(0,2));

        // Right Arm COM seen from Root RcR i.e. R*NcR where NcR is the COM vector of the left arm w.r.t. Neck
        Vector RTcom; RTcom = upperTorso->total_COM_RT;
        Vector RcR = rotT0T1*(RTcom.subVector(0,2));

        //Head COM Seen from ROOT RcH i.e. R*NcH where NcH is the com of the head w.r.t the Neck
        Vector HDcom; HDcom = upperTorso->total_COM_UP;
        Vector RcH = rotT0T1*(HDcom.subVector(0,2));

        // First additional matrix RIGHT ARM
        Matrix L1; L1.resize(6,3); L1.zero();
        L1.setRow(0,  -1*(Jac_Torso.getRow(5)*RcR(1))     +      Jac_Torso.getRow(4)*RcR(2));
        L1.setRow(1,      Jac_Torso.getRow(5)*RcR(0)      +   -1*Jac_Torso.getRow(3)*RcR(2));
        L1.setRow(2,  -1*(Jac_Torso.getRow(4)*RcR(0))     +      Jac_Torso.getRow(3)*RcR(1));
        // Second additional matrix LEFT ARM
        Matrix L2; L2.resize(6,3); L2.zero();
        L2.setRow(0,  -1*(Jac_Torso.getRow(5)*RcL(1))     +      Jac_Torso.getRow(4)*RcL(2));
        L2.setRow(1,      Jac_Torso.getRow(5)*RcL(0)      +   -1*Jac_Torso.getRow(3)*RcL(2));
        L2.setRow(2,  -1*(Jac_Torso.getRow(4)*RcL(0))     +      Jac_Torso.getRow(3)*RcL(1));
        // Third addition matrix HEAD
        Matrix L3; L3.resize(6,3); L3.zero();
        L3.setRow(0,  -1*(Jac_Torso.getRow(5)*RcH(1))     +      Jac_Torso.getRow(4)*RcH(2));
        L3.setRow(1,      Jac_Torso.getRow(5)*RcH(0)      +   -1*Jac_Torso.getRow(3)*RcH(2));
        L3.setRow(2,  -1*(Jac_Torso.getRow(4)*RcH(0))     +      Jac_Torso.getRow(3)*RcH(1));

        //Preparing the hright matrix for passing COM Jacobian toRIGHT FOOT REFERENCE FRAME
        Matrix hright;
        hright.resize(4,4); hright.zero();
        hright(0,0)=1.0;        //  1  0  0
        hright(1,2)=1.0;        //  0  0  1
        hright(2,1)=-1.0;       //  0 -1  0
        hright(3,3)=1.0;        //
        hright(2,3)=-0.1199;hright(1,3)=0.0681;

        Matrix rRf; rRf.resize(6,6); rRf.zero();
        Matrix rTf; rTf.resize(4,4); rTf.zero();
        Matrix fTr; fTr.resize(4,4); fTr.zero();
        Matrix fLr; fLr.resize(6,32); fLr.zero();
        Matrix fRr; fRr.resize(6,6); fRr.zero();
        Vector rCw; rCw.resize(3); rCw.zero(); //Whole Body COM w.r.t. ROOT.
        rCw = whole_COM;
        Vector fCw; fCw.resize(3); fCw.zero();

        Matrix fJr; fJr.resize(6,32); fJr.zero(); //Right Leg GeoJacobian from Root to Foot.
        Matrix rJf; rJf.resize(6,32); rJf.zero(); //Right Leg GeoJacobianf rom Foot to Root.
        Matrix HH; HH.resize(6,32); HH.zero();    //To be used to obtain fJr.
        Matrix LL; LL.resize(3,3); LL.zero();     //Used to build HH.
        Matrix hright_rot6x6; hright_rot6x6.resize(6,6); hright_rot6x6.zero(); //used to rotate right leg jacobian to root

#ifdef DEBUG_FOOT_COM
//DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING
//DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING

        //Transforming the Whole Body (WB) COM Jacobian to the right foot reference frame
            rTf = lowerTorso->right->getH(); //Here we get rTf
            rTf = hright*rTf;                                                   //CHECKED

            hright_rot6x6.setSubmatrix(hright.submatrix(0,2,0,2), 0,0);         //CHECKED.
            hright_rot6x6.setSubmatrix(hright.submatrix(0,2,0,2), 3,3);

            rRf.setSubmatrix(rTf.submatrix(0,2,0,2), 0,0);                      //CHECKED
            rRf.setSubmatrix(rTf.submatrix(0,2,0,2), 3,3);                      //CHECKED
        //Now we need the geometric Jacobian of the right leg from FOOT to ROOT.
            rJf.setSubmatrix( hright_rot6x6*lowerTorso->right->GeoJacobian(),0,6); //We first get the GeoJacobian from Foot to Root.

        //Now we build the H matrix that will be summed up to rJf in order to get fJr.
        //But first we obtain individual elements of H for which we build matrix LL

            //ALL LL LINES CHECKED.
            LL(0,0) = rRf(1,0)*rTf(2,3) - rRf(2,0)*rTf(1,3);
            LL(0,1) = rRf(2,0)*rTf(0,3) - rRf(0,0)*rTf(2,3);
            LL(0,2) = rRf(0,0)*rTf(1,3) - rRf(1,0)*rTf(0,3);

            LL(1,0) = rRf(1,1)*rTf(2,3) - rRf(2,1)*rTf(1,3);
            LL(1,1) = rRf(2,1)*rTf(0,3) - rRf(0,1)*rTf(2,3);
            LL(1,2) = rRf(0,1)*rTf(1,3) - rRf(1,1)*rTf(0,3);

            LL(2,0) = rRf(1,2)*rTf(2,3) - rRf(2,2)*rTf(1,3);
            LL(2,1) = rRf(2,2)*rTf(0,3) - rRf(0,2)*rTf(2,3);
            LL(2,2) = rRf(0,2)*rTf(1,3) - rRf(1,2)*rTf(0,3);

            //ALL HHs CHECKED
            HH.setSubrow( LL(0,0)*rJf.getRow(3) + LL(0,1)*rJf.getRow(4) + LL(0,2)*rJf.getRow(5) , 0,6);
            HH.setSubrow( LL(1,0)*rJf.getRow(3) + LL(1,1)*rJf.getRow(4) + LL(1,2)*rJf.getRow(5) , 1,6);
            HH.setSubrow( LL(2,0)*rJf.getRow(3) + LL(2,1)*rJf.getRow(4) + LL(2,2)*rJf.getRow(5) , 2,6);

        //Now we add up the HH matrix to rRf.transposed()*rJf to fintally obtain fJr.
            fJr = -1*HH - (rRf.transposed())*rJf; //TO BE ADDED TO fLr + fRr * rJw

        // Next matrix to fill in is fLr which is a 6x32 matrix filled with zeros except for the columns for the
        // right leg. This matrix will be added to our 'rotated' jac.
        // Thus, we first get fTr
            fTr.setSubmatrix(rTf.submatrix(0,2,0,2).transposed() ,0,0);         //CHECKED
            fTr.setSubcol(-1*fTr.submatrix(0,2,0,2)*rTf.subcol(0,3,3) ,0,3);    //CHECKED
            fTr(3,3) = 1;

            fRr.setSubmatrix(fTr.submatrix(0,2,0,2),0,0);                       //CHECKED
            fRr.setSubmatrix(fTr.submatrix(0,2,0,2),3,3);                       //CHECKED
        // Next we prepare the fLr matrix
            fCw = fTr.submatrix(0,2,0,2)*rCw;                                   //NEEDED LIKE THIS FROM THE EQUATION
            fLr.setSubrow(-fCw(1)*fJr.getRow(5) + fCw(2)*fJr.getRow(4), 0,6);
            fLr.setSubrow( fCw(0)*fJr.getRow(5) - fCw(2)*fJr.getRow(3), 1,6);
            fLr.setSubrow(-fCw(0)*fJr.getRow(4) + fCw(1)*fJr.getRow(3), 2,6);

//DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING
//DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING DEBUGGING
#endif

    Vector temp_com; temp_com.resize(4,1); temp_com.zero();

    unsigned int r,c,ct=0;
    double tmp, tmp2; tmp = tmp2 = 0.0;
    switch (which_part)
    {
        case BODY_PART_ALL:
        for (r=0; r<6; r++)
        {
            ct = 0;
            tmp = lowerTorso->total_mass_LF /  whole_mass; for (c=0; c<6; c++, ct++) jac(r,ct) *= tmp;
            tmp = lowerTorso->total_mass_RT /  whole_mass; for (c=0; c<6; c++, ct++) jac(r,ct) *= tmp;
            tmp = lowerTorso->total_mass_UP /  whole_mass;
            tmp2 = upperTorso->total_mass_LF/whole_mass + upperTorso->total_mass_RT/whole_mass + upperTorso->total_mass_UP/whole_mass;
            for (c=0; c<3; c++, ct++){
                jac(r,ct) *= tmp;
                jac(r,ct) += tmp2*Jac_Torso(r,c);
                jac(r,ct) += (upperTorso->total_mass_RT/whole_mass)*L1(r,c) + (upperTorso->total_mass_LF/whole_mass)*L2(r,c) + (upperTorso->total_mass_UP/whole_mass)*L3(r,c);
            }
            tmp = upperTorso->total_mass_LF /  whole_mass; for (c=0; c<7; c++, ct++) jac(r,ct) *= tmp;
            tmp = upperTorso->total_mass_RT /  whole_mass; for (c=0; c<7; c++, ct++) jac(r,ct) *= tmp;
            tmp = upperTorso->total_mass_UP /  whole_mass; for (c=0; c<3; c++, ct++) jac(r,ct) *= tmp;
        }

        #ifdef DEBUG_FOOT_COM
        //Now we can finally add all the matricial terms that compose fJw
            jac = fJr + fLr + fRr*jac;
            temp_com.setSubvector(0,whole_COM);     //CHECKED
            temp_com(3)=1;                          //CHECKED
            if (sw_getcom)
            {
                temp_com = fTr*temp_com;                //CHECKED
                whole_COM = temp_com.subVector(0,2);    //CHECKED
            }
            // whole_COM(0) = -temp_com(2);
            // whole_COM(2) = temp_com(0);
        #endif

        break;
        case LOWER_BODY_PARTS:
        for (r=0; r<6; r++)
        {
            ct = 0;
            tmp = lowerTorso->total_mass_LF /  lower_mass; for (c=0; c<6; c++, ct++) jac(r,ct) *= tmp;
            tmp = lowerTorso->total_mass_RT /  lower_mass; for (c=0; c<6; c++, ct++) jac(r,ct) *= tmp;
            tmp = lowerTorso->total_mass_UP /  lower_mass; for (c=0; c<3; c++, ct++) jac(r,ct) *= tmp;
            for (c=0; c<7; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<7; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<3; c++, ct++) jac(r,ct) = 0;
        }


        break;
        case UPPER_BODY_PARTS:
        for (r=0; r<6; r++)
        {
            ct = 0;
            for (c=0; c<6; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<6; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<3; c++, ct++) jac(r,ct) = 0;
            tmp = upperTorso->total_mass_LF /  upper_mass; for (c=0; c<7; c++, ct++) jac(r,ct) *= tmp;
            tmp = upperTorso->total_mass_RT /  upper_mass; for (c=0; c<7; c++, ct++) jac(r,ct) *= tmp;
            tmp = upperTorso->total_mass_UP /  upper_mass; for (c=0; c<3; c++, ct++) jac(r,ct) *= tmp;

                fprintf(stderr, "COM_Jacob2\n %s\n\n", COM_Jacob.submatrix(0,5,29,31).toString().c_str());

        }
        break;
        case LEFT_LEG:
        for (r=0; r<6; r++)
        {
            ct = 0;
            //for (c=0; c<6; c++, ct++) jac(r,ct) *= 1;
            for (c=0; c<6; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<3; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<7; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<7; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<3; c++, ct++) jac(r,ct) = 0;
        }
        break;
        case RIGHT_LEG:
        for (r=0; r<6; r++)
        {
            ct = 0;
            for (c=0; c<6; c++, ct++) jac(r,ct) = 0;
            //for (c=0; c<6; c++, ct++) jac(r,ct) *= 1;
            for (c=0; c<3; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<7; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<7; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<3; c++, ct++) jac(r,ct) = 0;
        }
        break;
        case TORSO:
        for (r=0; r<6; r++)
        {
            ct = 0;
            for (c=0; c<6; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<6; c++, ct++) jac(r,ct) = 0;
            //for (c=0; c<3; c++, ct++) jac(r,ct) *= 1;
            for (c=0; c<7; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<7; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<3; c++, ct++) jac(r,ct) = 0;
        }
        break;
        case LEFT_ARM:
        for (r=0; r<6; r++)
        {
            ct = 0;
            for (c=0; c<6; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<6; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<3; c++, ct++) jac(r,ct) = 0;
            //for (c=0; c<7; c++, ct++) jac(r,ct) *= 1;
            for (c=0; c<7; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<3; c++, ct++) jac(r,ct) = 0;
        }
        break;
        case RIGHT_ARM:
        for (r=0; r<6; r++)
        {
            ct = 0;
            for (c=0; c<6; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<6; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<3; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<7; c++, ct++) jac(r,ct) = 0;
            //for (c=0; c<7; c++, ct++) jac(r,ct) *= 1;
            for (c=0; c<3; c++, ct++) jac(r,ct) = 0;
        }
        break;
        case HEAD:
        for (r=0; r<6; r++)
        {
            ct = 0;
            for (c=0; c<6; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<6; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<3; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<7; c++, ct++) jac(r,ct) = 0;
            for (c=0; c<7; c++, ct++) jac(r,ct) = 0;
            //for (c=0; c<3; c++, ct++) jac(r,ct) *= 1;
        }
        break;
    }
    return true;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCubWholeBody::EXPERIMENTAL_getCOMvelocity(BodyPart which_part, Vector &com_vel, Vector &dq)
{
    Matrix jac;
    sw_getcom = 0;
    EXPERIMENTAL_getCOMjacobian(which_part,jac);
    sw_getcom = 1;

    Vector jvel;
    getAllVelocities(jvel);
    dq = jvel;

    com_vel = jac*jvel;
    return true;
}



