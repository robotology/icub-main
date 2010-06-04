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

#include <iostream>
#include <yarp/math/Math.h>
#include <iCub/ctrl/ctrlMath.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynInv.h>
#include <iCub/iDyn/iDynFwd.h>
#include <deque>
#include <string>

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace ctrl;
using namespace iKin;
using namespace iDyn;


//================================
//
//		I DYN SENSOR
//
//================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynSensor::iDynSensor(iDynChain *_c, string _info, const NewEulMode _mode, unsigned int verb)
:iDynInvSensor(_c, _info, _mode, verb)
{}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynSensor::iDynSensor(iDynChain *_c, unsigned int i, const Matrix &_H, const Matrix &_HC, const double _m, const Matrix &_I, string _info, const NewEulMode _mode, unsigned int verb)
:iDynInvSensor(_c, i, _H, _HC, _m, _I, _info, _mode, verb)
{}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynSensor::setSensorMeasures(const Vector &F, const Vector &Mu)
{
	return sens->setMeasuredFMu(F,Mu);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynSensor::setSensorMeasures(const Vector &FM)
{
	if(FM.length()==6)
	{ 
		Vector f(3); f[0]=FM[0];f[1]=FM[1];f[2]=FM[2];
		Vector m(3); m[0]=FM[3];m[1]=FM[4];m[2]=FM[5];
		return sens->setMeasuredFMu(f,m);
	}
	else
	{
		if(verbose)
			cerr<<"iDynSensor error: could not set sensor measures due to wrong sized vector: "
			<<FM.length()<<" instead of 6 (3+3)"<<endl;
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynSensor::ForwardFromBase()
{
	if(chain->NE == NULL)		
	{
		chain->prepareNewtonEuler(mode);
		chain->initNewtonEuler();
	}
	//the iDynChain independently solve the forward phase of the limb
	//setting w,dw,ddp,ddpC
	chain->NE->ForwardFromBase();
	sens->ForwardAttachToLink(chain->refLink(lSens));
	//the sensor does not need to retrieve w,dw,ddp,ddpC in this case 	
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//   main computation methods
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynSensor::computeFromSensorNewtonEuler()
{
	//first forward all the quantities w,dw,.. in the chain
	ForwardFromBase();
	//then propagate forces and moments
	//from sensor to lSens
	sens->ForwardForcesMomentsToLink(chain->refLink(lSens));
	//from lSens to End
	chain->NE->InverseToEnd(lSens);
	//from lSens to Base
	chain->NE->InverseToBase(lSens);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynSensor::computeFromSensorNewtonEuler(const Vector &F, const Vector &Mu)
{
	//set measured F/Mu on the sensor
	if(setSensorMeasures(F,Mu))
	{
		computeFromSensorNewtonEuler();
		return true;
	}
	else
		return false;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynSensor::computeFromSensorNewtonEuler(const Vector &FMu)
{
	//set measured F/Mu on the sensor
	if(setSensorMeasures(FMu))
	{
		computeFromSensorNewtonEuler();
		return true;
	}
	else
		return false;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//  get methods, overloaded from iDyn
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynSensor::getForces() const							{return chain->getForces();}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynSensor::getMoments() const							{return chain->getMoments();}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensor::getTorques() const							{return chain->getTorques();}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensor::getForce(const unsigned int iLink) const		{return chain->getForce(iLink);}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensor::getMoment(const unsigned int iLink) const	{return chain->getMoment(iLink);}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynSensor::getTorque(const unsigned int iLink) const	{return chain->getTorque(iLink);}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynSensor::getForcesNewtonEuler() const					{return chain->getForcesNewtonEuler();}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynSensor::getMomentsNewtonEuler() const				{return chain->getMomentsNewtonEuler();}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensor::getTorquesNewtonEuler() const				{return chain->getTorquesNewtonEuler();}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynSensor::getForceMomentEndEff() const					{return chain->getForceMomentEndEff();}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//======================================
//
//		 iDYN SENSOR ARM
//
//======================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynSensorArm::iDynSensorArm(iCubArmDyn *_c, const NewEulMode _mode, unsigned int verb)
:iDynSensor(_c->asChain(),_c->getType(),_mode,verb)
{
	// FT sensor is in position 5 in the kinematic chain in both arms
	lSens = 5;
	// the arm type determines the sensor properties
	if( !((_c->getType()=="left")||(_c->getType()=="right"))  )
	{
		if(verbose)
		cerr<<"iDynSensorArm error: type is not left/right. iCub only has a left and a right arm, it is not an octopus :)"<<endl
			<<"iDynSensorArm: assuming right arm."<<endl;
		// set the sensor with the default value
		sens = new iCubArmSensorLink("right",mode,verbose);
	}
	else
	{
		// set the sensor properly
		sens = new iCubArmSensorLink(_c->getType(),mode,verbose);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string iDynSensorArm::getType() const
{ 
	return sens->getType();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//======================================
//
//		 iDYN INV SENSOR LEG
//
//======================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynSensorLeg::iDynSensorLeg(iCubLegDyn *_c, const NewEulMode _mode, unsigned int verb)
:iDynSensor(_c->asChain(),_c->getType(),_mode,verb)
{
	// FT sensor is in position 2 in the kinematic chain in both legs
	lSens = 2;
	// the leg type determines the sensor properties
	if( !((_c->getType()=="left")||(_c->getType()=="right"))  )
	{
		if(verbose)
			cerr<<"iDynSensorLeg error: type is not left/right. iCub only has a left and a right leg, it is not a millipede :)"<<endl
				<<"iDynSensorLeg: assuming right leg."<<endl;
		// set the sensor with the default value
		sens = new iCubLegSensorLink("right",mode,verbose);
	}
	else
	{
		// set the sensor properly
		sens = new iCubLegSensorLink(_c->getType(),mode,verbose);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string iDynSensorLeg::getType() const
{ 
	return sens->getType();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


