/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace ctrl;
using namespace iKin;
using namespace iDyn;

//====================================
//
//		RIGID BODY TRANSFORMATION
//
//====================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
RigidBodyTransformation::RigidBodyTransformation(iDynLimb *_limb, const yarp::sig::Matrix &_H, const string &_info, const FlowType kin, const FlowType wre, const NewEulMode _mode, unsigned int verb)
{
	limb = _limb;
	kinFlow = kin;
	wreFlow = wre;
	mode = _mode;
	info = _info;
	F.resize(3);	F.zero();
	Mu.resize(3);	Mu.zero();
	w.resize(3);	w.zero();
	dw.resize(3);	dw.zero();
	ddp.resize(3);	ddp.zero();
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
		if(verbose)
			cerr<<"RigidBodyTransformation: could not set RBT due to wrong sized matrix H: "
				<<"("<<_H.cols()<<","<<_H.rows()<<") instead of (4,4). Setting identity as default."<<endl;
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
Vector RigidBodyTransformation::getr(bool proj)	
{
	if(proj==false)
		return H.submatrix(0,2,0,3).getCol(3);
	else
		return (-1.0 * getR().transposed() * (H.submatrix(0,2,0,3)).getCol(3));
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
			w	= getR().transposed() * w ;			
			dw	= getR().transposed() * dw ;
			ddp =  getR().transposed() * (ddp - cross(dw,getr(true)) - cross(w,cross(w,getr(true))) );
			break;
		case STATIC:	
			w	= 0.0;
			dw	= 0.0;
			ddp = getR().transposed() * ddp;
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
			w	= getR() * w ;	
			dw	= getR() * dw ;
			ddp = getR() * ddp + cross(dw,getr(true)) + cross(w,cross(w,getr(true))) ;
			break;
		case STATIC:	
			w	= 0.0;
			dw	= 0.0;
			ddp =  getR() * ddp ;
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





//====================================
//
//		i DYN NODE
//
//====================================
	
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynNode::iDynNode(const NewEulMode _mode)
{
	rbtList.clear();
	mode = _mode;
	verbose = VERBOSE;
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
void iDynNode::addLimb(iDynLimb *limb, const Matrix &H, const FlowType kinFlow, const FlowType wreFlow)
{
	string infoRbt = limb->getType() + " to node";
	RigidBodyTransformation rbt(limb,H,infoRbt,kinFlow,wreFlow,mode,verbose);
	rbtList.push_back(rbt);
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
			rbtList[i].limb->computeKinematicNewtonEuler();
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
				rbtList[i].limb->computeKinematicNewtonEuler();
			}
		}
		return true;
	
	}
	else
	{
		if(verbose)
			cerr<<"iDynNode: error: there are "<<inputNode<<"limbs with Kinematic Flow = Input. "
				<<" Only one limb must have Kinematic Input from outside measurements/computations. "
				<<"Please check the coherence of the limb configuration in the node '"<<info<<"'"<<endl;
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
			fprintf(stderr,"i'm here!!!");
			//compute the wrench pass in that limb
			rbtList[i].limb->computeWrenchNewtonEuler();
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

	//just check if the there is at least one output node (as it should be)
	//if(outputNode<rbtList.size())
	{
		//now forward the wrench output from the node to limbs whose wrench flow is output type
		for(unsigned int i=0; i<rbtList.size(); i++)
		{
			fprintf(stderr,"i'm here2!!!");
			if(rbtList[i].getWrenchFlow()==RBT_NODE_OUT)
			{
				fprintf(stderr,"i'm here3!!!");
				//init the wrench with the node information
				rbtList[i].setWrench(F,Mu);
				//solve wrench in that limb/chain
				rbtList[i].limb->computeWrenchNewtonEuler();
			}
		}
		return true;
	}
	/*else
	{
		if(verbose)
			cerr<<"iDynNode: error: there are no limbs with Wrench Flow = Output. "
				<<" At least one limb must have Wrench Output for balancing forces in the node. "
				<<"Please check the coherence of the limb configuration in the node '"<<info<<"'"<<endl;
		return false;
	}*/
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynNode::solveWrench(const Matrix &FM)
{
	int inputNode = 0;
	Vector fi(3); fi.zero();
	Vector mi(3); mi.zero();
	Vector FMi(6);FMi.zero();

	//check how many limbs have wrench input
	for(unsigned int i=0; i<rbtList.size(); i++)
		if(rbtList[i].getWrenchFlow()==RBT_NODE_IN)			
			inputNode++;
		
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
			cerr<<"iDynNode: could not solveWrench due to missing wrenches to initialize the computations: "
				<<" only "<<FM.cols()<<" f/m available instead of "<<inputNode<<endl;
		return false;
	}
	if(FM.rows()!=6)
	{
		if(verbose)
			cerr<<"iDynNode: could not solveWrench due to wrong sized init wrenches: "
				<<FM.rows()<<" instead of 6 (3+3)"<<endl;
		return false;
	}

	//set the measured/input forces/moments from each limb
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

	//now that all is set, we can really solveWrench()
	return solveWrench();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynNode::solveWrench(const Matrix &F, const Matrix &M)
{
	int inputNode = 0;

	//check how many limbs have wrench input
	for(unsigned int i=0; i<rbtList.size(); i++)
		if(rbtList[i].getWrenchFlow()==RBT_NODE_IN)			
			inputNode++;
		
	// input (eg measured) wrenches are stored in two 3xN matrix: each column is a 3x1 vector
	// with force/moment; N is the number of columns, ie the number of measured/input wrenches to the limb
	// the order is assumed coherent with the one built when adding limbs
	// eg: 
	// adding limbs: addLimb(limb1), addLimb(limb2), addLimb(limb3)
	// where limb1, limb3 have wrench flow input
	// passing wrenches: Matrix F(3,2), F.setcol(0,f1), F.setcol(1,f3) and similar for moment
	if((F.cols()<inputNode)||(M.cols()<inputNode))
	{
		if(verbose)
			cerr<<"iDynNode: could not solveWrench due to missing wrenches to initialize the computations: "
				<<" only "<<F.cols()<<"/"<<M.cols()<<" f/m available instead of "<<inputNode<<"/"<<inputNode<<endl;
		return false;
	}
	if((F.rows()!=3)||(M.rows()!=3))
	{
		if(verbose)
			cerr<<"iDynNode: could not solveWrench due to wrong sized init f/m: "
				<<F.rows()<<"/"<<M.rows()<<" instead of 3/3 "<<endl;
		return false;
	}

	//set the measured/input forces/moments from each limb
	inputNode = 0;
	for(unsigned int i=0; i<rbtList.size(); i++)
	{
		if(rbtList[i].getWrenchFlow()==RBT_NODE_IN)			
		{
			// from the input matrix
			//set the input wrench in the RBT->limb
			rbtList[i].setWrenchMeasure(F.getCol(inputNode),M.getCol(inputNode));
			inputNode++;
		}
	}

	//now that all is set, we can really solveWrench()
	return solveWrench();
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynNode::getForce() const {	return F;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynNode::getMoment() const {return Mu;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynNode::getAngVel() const {return w;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynNode::getAngAcc() const {return dw;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynNode::getLinAcc() const {return ddp;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
