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
RigidBodyTransformation::RigidBodyTransformation(iDynLimb *_limb, const yarp::sig::Matrix &_H, const string &_info, bool _hasSensor, const FlowType kin, const FlowType wre, const NewEulMode _mode, unsigned int verb)
{
	limb = _limb;
	kinFlow = kin;
	wreFlow = wre;
	mode = _mode;
	info = _info;
	hasSensor=_hasSensor;
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
			ddp = getR() * ( ddp - cross(dw,getr(true)) - cross(w,cross(w,getr(true))) ) ;
			w	= getR() * w ;	
			dw	= getR() * dw ;	
			break;
		case STATIC:	
			w	= 0.0;
			dw	= 0.0;
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
			ddp = getR().transposed() * (ddp + cross(dw,getr(true)) + cross(w,cross(w,getr(true))) );
			w	= getR().transposed() * w ;			
			dw	= getR().transposed() * dw ;
			break;
		case STATIC:	
			w	= 0.0;
			dw	= 0.0;
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
void iDynNode::addLimb(iDynLimb *limb, const Matrix &H, const FlowType kinFlow, const FlowType wreFlow, bool hasSensor)
{
	string infoRbt = limb->getType() + " to node";
	RigidBodyTransformation rbt(limb,H,infoRbt,hasSensor,kinFlow,wreFlow,mode,verbose);
	rbtList.push_back(rbt);
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
			cerr<<"iDynNode: error: there are "<<inputNode<<"limbs with Kinematic Flow = Input. "
				<<" Only one limb must have Kinematic Input from outside measurements/computations. "
				<<"Please check the coherence of the limb configuration in the node '"<<info<<"'"<<endl;
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
			cerr<<"iDynNode: error: there are "<<inputNode<<"limbs with Kinematic Flow = Input. "
				<<" Only one limb must have Kinematic Input from outside measurements/computations. "
				<<"Please check the coherence of the limb configuration in the node '"<<info<<"'"<<endl;
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
				rbtList[i].setKinematicMeasure(w0,dw0,ddp0);	
		}
		return true;
	}
	else
	{
		if(verbose)
			cerr<<"iDynNode: error, could not set Kinematic measures due to wrong sized vectors. "
				<<" w,dw,ddp have lenght "<<w0.length()<<","<<dw0.length()<<","<<ddp0.length()
				<<" instead of 3,3,3." <<endl;
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
	// it is not correct, because the node must be in balanc
	if(outputNode==rbtList.size())
	{
		if(verbose)
			cerr<<"iDynNode: warning: there are no limbs with Wrench Flow = Output. "
				<<" At least one limb must have Wrench Output for balancing forces in the node. "
				<<"Please check the coherence of the limb configuration in the node '"<<info<<"'"<<endl;
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
	int inputNode = 0;
	Vector fi(3); fi.zero();
	Vector mi(3); mi.zero();
	Vector FMi(6);FMi.zero();
	bool inputWasOk = true;

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
				<<" only "<<FM.cols()<<" f/m available instead of "<<inputNode<<endl
				<<"          Using default values, all zero."<<endl;
		inputWasOk = false;
	}
	if(FM.rows()!=6)
	{
		if(verbose)
			cerr<<"iDynNode: could not solveWrench due to wrong sized init wrenches: "
				<<FM.rows()<<" instead of 6 (3+3)"<<endl
				<<"          Using default values, all zero."<<endl;
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
	int inputNode = 0;
	bool inputWasOk = true;

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
	// passing wrenches: Matrix Fm(3,2), Fm.setcol(0,f1), Fm.setcol(1,f3) and similar for moment
	if((Fm.cols()<inputNode)||(Mm.cols()<inputNode))
	{
		if(verbose)
			cerr<<"iDynNode: could not setWrenchMeasure due to missing wrenches to initialize the computations: "
				<<" only "<<Fm.cols()<<"/"<<Mm.cols()<<" f/m available instead of "<<inputNode<<"/"<<inputNode<<endl
				<<"          Using default values, all zero."<<endl;
		inputWasOk = false;
	}
	if((Fm.rows()!=3)||(Mm.rows()!=3))
	{
		if(verbose)
			cerr<<"iDynNode: could not setWrenchMeasure due to wrong sized init f/m: "
				<<Fm.rows()<<"/"<<Mm.rows()<<" instead of 3/3 "<<endl
				<<"          Using default values, all zero."<<endl;
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







//====================================
//
//		i DYN SENSOR NODE
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
		if(verbose)
			cerr<<"iDynNode: warning: there are no limbs with Wrench Flow = Output. "
				<<" At least one limb must have Wrench Output for balancing forces in the node. "
				<<"Please check the coherence of the limb configuration in the node '"<<info<<"'"<<endl;
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
bool iDynSensorNode::setWrenchMeasure(const Matrix &FM)
{
	int inputNode = 0;
	Vector fi(3); fi.zero();
	Vector mi(3); mi.zero();
	Vector FMi(6);FMi.zero();
	bool inputWasOk = true;

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
			cerr<<"iDynNode: could not setWrenchMeasure due to missing wrenches to initialize the computations: "
				<<" only "<<FM.cols()<<" f/m available instead of "<<inputNode<<endl
				<<"          Using default values, all zero."<<endl;
		inputWasOk = false;
	}
	if(FM.rows()!=6)
	{
		if(verbose)
			cerr<<"iDynNode: could not setWrenchMeasure due to wrong sized init wrenches: "
				<<FM.rows()<<" instead of 6 (3+3)"<<endl
				<<"          Using default values, all zero."<<endl;
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
		for(unsigned int i=0; i<rbtList.size(); i++)
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
bool iDynSensorNode::setWrenchMeasure(const Matrix &Fm, const Matrix &Mm)
{
	int inputNode = 0;
	bool inputWasOk = true;

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
	// passing wrenches: Matrix Fm(3,2), Fm.setcol(0,f1), Fm.setcol(1,f3) and similar for moment
	if((Fm.cols()<inputNode)||(Mm.cols()<inputNode))
	{
		if(verbose)
			cerr<<"iDynNode: could not setWrenchMeasure due to missing wrenches to initialize the computations: "
				<<" only "<<Fm.cols()<<"/"<<Mm.cols()<<" f/m available instead of "<<inputNode<<"/"<<inputNode<<endl
				<<"          Using default values, all zero."<<endl;
		inputWasOk = false;
	}
	if((Fm.rows()!=3)||(Mm.rows()!=3))
	{
		if(verbose)
			cerr<<"iDynNode: could not setWrenchMeasure due to wrong sized init f/m: "
				<<Fm.rows()<<"/"<<Mm.rows()<<" instead of 3/3 "<<endl
				<<"          Using default values, all zero."<<endl;
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
		for(unsigned int i=0; i<rbtList.size(); i++)	
			if(rbtList[i].getWrenchFlow()==RBT_NODE_IN)			
				rbtList[i].setWrenchMeasure(fi,mi);	
	}

	//now that all is set, we can really solveWrench()
	return inputWasOk ;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~





//====================================
//
//	     iDYN SENSOR TORSO NODE   
//
//====================================


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynSensorTorsoNode::iDynSensorTorsoNode(const NewEulMode _mode, unsigned int verb)
:iDynSensorNode("torso_node",_mode,verb)
{
	build();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynSensorTorsoNode::build()
{
	left	= new iCubArmNoTorsoDyn("left",KINFWD_WREBWD);
	right	= new iCubArmNoTorsoDyn("right",KINFWD_WREBWD);
	up		= new iCubNeckInertialDyn(KINBWD_WREBWD);

	leftSensor = new iDynSensorArmNoTorso(dynamic_cast<iCubArmNoTorsoDyn*>(left),mode,verbose);
	rightSensor= new iDynSensorArmNoTorso(dynamic_cast<iCubArmNoTorsoDyn*>(right),mode,verbose);

	HUp.resize(4,4);	HUp.eye();
	HLeft.resize(4,4);	HLeft.eye();
	HRight.resize(4,4);	HRight.eye();

	// order: head - right - left
	addLimb(up,HUp,RBT_NODE_IN,RBT_NODE_IN);
	addLimb(right,HRight,rightSensor,RBT_NODE_OUT,RBT_NODE_IN);
	addLimb(left,HLeft,leftSensor,RBT_NODE_OUT,RBT_NODE_IN);

	left_name = "left_arm";
	right_name= "right_arm";
	up_name	  = "head";
	name	  = "upper-torso";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynSensorTorsoNode::~iDynSensorTorsoNode()
{
	delete rightSensor; rightSensor = NULL;
	delete leftSensor;	leftSensor = NULL;
	delete up;			up = NULL;
	delete right;		right = NULL;
	delete left;		left = NULL;

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
	Vector FM_up(6); FM_up.zero();
	return setSensorMeasurement(FM_right,FM_left,FM_up);
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
		return setWrenchMeasure(FM);
	}
	else
	{
		if(verbose)
			cerr<<"Node <"<<name<<"> could not set sensor measurements properly due to wrong sized vectors. "
				<<" FM up/right/left have lenght "<<FM_up.length()<<","<<FM_right.length()<<","<<FM_left.length()
				<<" instead of 6,6. Setting everything to zero. "<<endl;
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
			cerr<<"Node <"<<name<<"> error, could not update() due to wrong sized vectors. "
				<<" w0,dw0,ddp0 have size "<<w0.length()<<","<<dw0.length()<<","<<ddp0.length()<<" instead of 3,3,3. "
				<<" FM up/right/left have size "<<FM_up.length()<<","<<FM_right.length()<<","<<FM_left.length()<<" instead of 6,6,6. "
				<<"            Updating without new values."<< endl;
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
	if(limbType==up_name)				return up->getForces();
	else if(limbType==left_name)		return left->getForces();
	else if(limbType==right_name)		return right->getForces();
	else
	{		
		if(verbose)	cerr<<"Node <"<<name<<"> there's not a limb named "<<limbType<<". Only "<<left_name<<","<<right_name<<","<<up_name<<" are available. "<<endl;
		return Matrix(0,0);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynSensorTorsoNode::getMoments(const string &limbType)
{
	if(limbType==up_name)			return up->getMoments();
	else if(limbType==left_name)	return left->getMoments();
	else if(limbType==right_name)	return right->getMoments();
	else
	{		
		if(verbose)	cerr<<"Node <"<<name<<"> there's not a limb named "<<limbType<<". Only "<<left_name<<","<<right_name<<","<<up_name<<" are available. "<<endl;
		return Matrix(0,0);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::getTorques(const string &limbType)
{
	if(limbType==up_name)			return up->getTorques();
	else if(limbType==left_name)	return left->getTorques();
	else if(limbType==right_name)	return right->getTorques();
	else
	{		
		if(verbose) cerr<<"Node <"<<name<<"> there's not a limb named "<<limbType<<". Only "<<left_name<<","<<right_name<<","<<up_name<<" are available. "<<endl;
		return Vector(0);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::getTorsoForce() const
{
	return F;
}	
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::getTorsoMoment() const
{
	return Mu;
}	
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::getTorsoAngVel() const
{
	return w;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::getTorsoAngAcc() const
{
	return dw;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::getTorsoLinAcc() const
{
	return ddp;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	//------------------
	//    LIMB CALLS
	//------------------

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::setAng(const string &limbType, const Vector &_q)
{
	if(limbType==up_name)			return up->setAng(_q);
	else if(limbType==left_name)	return left->setAng(_q);
	else if(limbType==right_name)	return right->setAng(_q);
	else
	{		
		if(verbose)	cerr<<"Node <"<<name<<"> there's not a limb named "<<limbType<<". Only "<<left_name<<","<<right_name<<","<<up_name<<" are available. "<<endl;
		return Vector(0);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::getAng(const string &limbType)
{
	if(limbType==up_name)			return up->getAng();
	else if(limbType==left_name)	return left->getAng();
	else if(limbType==right_name)	return right->getAng();
	else
	{		
		if(verbose)	cerr<<"Node <"<<name<<"> there's not a limb named "<<limbType<<". Only "<<left_name<<","<<right_name<<","<<up_name<<" are available. "<<endl;
		return Vector(0);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynSensorTorsoNode::setAng(const string &limbType, const unsigned int i, double _q)
{
	if(limbType==up_name)			return up->setAng(i,_q);
	else if(limbType==left_name)	return left->setAng(i,_q);
	else if(limbType==right_name)	return right->setAng(i,_q);
	else
	{		
		if(verbose)	cerr<<"Node <"<<name<<"> there's not a limb named "<<limbType<<". Only "<<left_name<<","<<right_name<<","<<up_name<<" are available. "<<endl;
		return 0.0;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynSensorTorsoNode::getAng(const string &limbType, const unsigned int i)
{
	if(limbType==up_name)			return up->getAng(i);
	else if(limbType==left_name)	return left->getAng(i);
	else if(limbType==right_name)	return right->getAng(i);
	else
	{		
		if(verbose)	cerr<<"Node <"<<name<<"> there's not a limb named "<<limbType<<". Only "<<left_name<<","<<right_name<<","<<up_name<<" are available. "<<endl;
		return 0.0;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::setDAng(const string &limbType, const Vector &_dq)
{
	if(limbType==up_name)			return up->setDAng(_dq);
	else if(limbType==left_name)	return left->setDAng(_dq);
	else if(limbType==right_name)	return right->setDAng(_dq);
	else
	{		
		if(verbose)	cerr<<"Node <"<<name<<"> there's not a limb named "<<limbType<<". Only "<<left_name<<","<<right_name<<","<<up_name<<" are available. "<<endl;
		return Vector(0);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::getDAng(const string &limbType)
{
	if(limbType==up_name)			return up->getDAng();
	else if(limbType==left_name)	return left->getDAng();
	else if(limbType==right_name)	return right->getDAng();
	else
	{		
		if(verbose)	cerr<<"Node <"<<name<<"> there's not a limb named "<<limbType<<". Only "<<left_name<<","<<right_name<<","<<up_name<<" are available. "<<endl;
		return Vector(0);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynSensorTorsoNode::setDAng(const string &limbType, const unsigned int i, double _dq)
{
	if(limbType==up_name)			return up->setDAng(i,_dq);
	else if(limbType==left_name)	return left->setDAng(i,_dq);
	else if(limbType==right_name)	return right->setDAng(i,_dq);
	else
	{		
		if(verbose)	cerr<<"Node <"<<name<<"> there's not a limb named "<<limbType<<". Only "<<left_name<<","<<right_name<<","<<up_name<<" are available. "<<endl;
		return 0.0;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynSensorTorsoNode::getDAng(const string &limbType, const unsigned int i)    
{
	if(limbType==up_name)			return up->getDAng(i);
	else if(limbType==left_name)	return left->getDAng(i);
	else if(limbType==right_name)	return right->getDAng(i);
	else
	{		
		if(verbose)	cerr<<"Node <"<<name<<"> there's not a limb named "<<limbType<<". Only "<<left_name<<","<<right_name<<","<<up_name<<" are available. "<<endl;
		return 0.0;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::setD2Ang(const string &limbType, const Vector &_ddq)
{
	if(limbType==up_name)			return up->setD2Ang(_ddq);
	else if(limbType==left_name)	return left->setD2Ang(_ddq);
	else if(limbType==right_name)	return right->setD2Ang(_ddq);
	else
	{		
		if(verbose)	cerr<<"Node <"<<name<<"> there's not a limb named "<<limbType<<". Only "<<left_name<<","<<right_name<<","<<up_name<<" are available. "<<endl;
		return Vector(0);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensorTorsoNode::getD2Ang(const string &limbType)
{
	if(limbType==up_name)			return up->getD2Ang();
	else if(limbType==left_name)	return left->getD2Ang();
	else if(limbType==right_name)	return right->getD2Ang();
	else
	{		
		if(verbose)	cerr<<"Node <"<<name<<"> there's not a limb named "<<limbType<<". Only "<<left_name<<","<<right_name<<","<<up_name<<" are available. "<<endl;
		return Vector(0);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynSensorTorsoNode::setD2Ang(const string &limbType, const unsigned int i, double _ddq)
{
	if(limbType==up_name)			return up->setD2Ang(i,_ddq);
	else if(limbType==left_name)	return left->setD2Ang(i,_ddq);
	else if(limbType==right_name)	return right->setD2Ang(i,_ddq);
	else
	{		
		if(verbose)	cerr<<"Node <"<<name<<"> there's not a limb named "<<limbType<<". Only "<<left_name<<","<<right_name<<","<<up_name<<" are available. "<<endl;
		return 0.0;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynSensorTorsoNode::getD2Ang(const string &limbType, const unsigned int i)
{
	if(limbType==up_name)			return up->getD2Ang(i);
	else if(limbType==left_name)	return left->getD2Ang(i);
	else if(limbType==right_name)	return right->getD2Ang(i);
	else
	{		
		if(verbose)	cerr<<"Node <"<<name<<"> there's not a limb named "<<limbType<<". Only "<<left_name<<","<<right_name<<","<<up_name<<" are available. "<<endl;
		return 0.0;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
unsigned int iDynSensorTorsoNode::getNLinks(const string &limbType) const
{
	if(limbType==up_name)			return up->getN();
	else if(limbType==left_name)	return left->getN();
	else if(limbType==right_name)	return right->getN();
	else
	{		
		if(verbose)	cerr<<"Node <"<<name<<"> there's not a limb named "<<limbType<<". Only "<<left_name<<","<<right_name<<","<<up_name<<" are available. "<<endl;
		return 0;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//====================================
//
//	        UPPER TORSO   
//
//====================================


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubUpperTorso::iCubUpperTorso(const NewEulMode _mode, unsigned int verb)
:iDynSensorTorsoNode(_mode,verb)
{}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCubUpperTorso::build()
{
	left	= new iCubArmNoTorsoDyn("left",KINFWD_WREBWD);
	right	= new iCubArmNoTorsoDyn("right",KINFWD_WREBWD);
	up		= new iCubNeckInertialDyn(KINBWD_WREBWD);

	leftSensor = new iDynSensorArmNoTorso(dynamic_cast<iCubArmNoTorsoDyn*>(left),mode,verbose);
	rightSensor= new iDynSensorArmNoTorso(dynamic_cast<iCubArmNoTorsoDyn*>(right),mode,verbose);

	HUp.resize(4,4);	HUp.eye();
	HLeft.resize(4,4);	HLeft.eye();
	HRight.resize(4,4);	HRight.eye();

	// order: head - right arm - left arm
	addLimb(up,HUp,RBT_NODE_IN,RBT_NODE_IN);
	addLimb(right,HRight,rightSensor,RBT_NODE_OUT,RBT_NODE_IN);
	addLimb(left,HLeft,leftSensor,RBT_NODE_OUT,RBT_NODE_IN);

	left_name = "left_arm";
	right_name= "right_arm";
	up_name	  = "head";
	name	  = "upper_torso";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//====================================
//
//	        LOWER TORSO   
//
//====================================


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubLowerTorso::iCubLowerTorso(const NewEulMode _mode, unsigned int verb)
:iDynSensorTorsoNode(_mode,verb)
{}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCubLowerTorso::build()
{
	left	= new iCubLegDyn("left",KINFWD_WREBWD);
	right	= new iCubLegDyn("right",KINFWD_WREBWD);
	up		= new iCubTorsoDyn("lower",KINBWD_WREBWD);

	leftSensor = new iDynSensorLeg(dynamic_cast<iCubLegDyn*>(left),mode,verbose);
	rightSensor= new iDynSensorLeg(dynamic_cast<iCubLegDyn*>(right),mode,verbose);

	HUp.resize(4,4);	HUp.eye();
	HLeft.resize(4,4);	HLeft.eye();
	HRight.resize(4,4);	HRight.eye();

	// order: torso - right leg - left leg
	addLimb(up,HUp,RBT_NODE_IN,RBT_NODE_IN);
	addLimb(right,HRight,rightSensor,RBT_NODE_OUT,RBT_NODE_IN);
	addLimb(left,HLeft,leftSensor,RBT_NODE_OUT,RBT_NODE_IN);

	left_name = "left_leg";
	right_name= "right_leg";
	up_name	  = "torso";
	name	  = "lower_torso";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//====================================
//
//	        iCUB WHOLE BODY  
//
//====================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubWholeBody::iCubWholeBody(const NewEulMode mode, unsigned int verbose)
{
	//create all limbs
	upperTorso = new iCubUpperTorso(mode,verbose);
	lowerTorso = new iCubLowerTorso(mode,verbose);
	
	//now create a connection between upperTorso node and Torso (limb)
	Matrix H(4,4); H.eye();
	rbt = new RigidBodyTransformation(lowerTorso->up,H,"connection between lower and upper torso",false,RBT_NODE_OUT,RBT_NODE_OUT,mode,verbose);

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubWholeBody::~iCubWholeBody()
{
	delete upperTorso; upperTorso = NULL;
	delete lowerTorso; lowerTorso = NULL;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


