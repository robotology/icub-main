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
#include <stdio.h>
#include <deque>
#include <string>

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace iCub::iDyn;

//================================
//
//		CONTACT NEWTON EULER
//
//================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ContactNewtonEuler::ContactNewtonEuler(unsigned int verb)
{
    verbose = verb;
    info = "contact";
    H.resize(4,4); H.eye();
    F.resize(3); F.zero();
    Mu.resize(3); Mu.zero();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ContactNewtonEuler::sconnect()
{
    H.eye();
    F.zero(); Mu.zero();
    info = "no contact";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ContactNewtonEuler::zero()
{
    H.eye();
    F.zero(); Mu.zero();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~
//   get methods
//~~~~~~~~~~~~~~~~~~~~~~

Vector	ContactNewtonEuler::getForce()	const   {return F;}
Vector	ContactNewtonEuler::getMoment()	const	{return Mu;}
Matrix	ContactNewtonEuler::getH()		const   {return H;}
Matrix	ContactNewtonEuler::getR()      const   {return H.submatrix(0,2,0,2);}
Vector	ContactNewtonEuler::getr()      const   {return H.submatrix(0,2,0,3).getCol(3);}

//~~~~~~~~~~~~~~~~~~~~~~
//   set methods
//~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ContactNewtonEuler::setVerbose(unsigned int verb)
{
	verbose = verb;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ContactNewtonEuler::setInfo(const string &_info)
{
	info = _info;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool ContactNewtonEuler::setForce(const Vector &_F)
{
	if(_F.length()==3)
	{
		F=_F;
		return true;
	}
	else
	{
		if(verbose)
			fprintf(stderr,"ContactNewtonEuler error, could not set force due to wrong dimension: %ld instead of 3.\n",_F.length());
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool ContactNewtonEuler::setMoment(const Vector &_Mu)
{
	if(_Mu.length()==3)
	{
		Mu=_Mu;
		return true;
	}
	else
	{
		if(verbose)
			fprintf(stderr,"ContactNewtonEuler error, could not set moment due to wrong dimension: %ld instead of 3.\n",_Mu.length());
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool ContactNewtonEuler::setMeasuredFMu(const Vector &_FM)
{
	if((_FM.length()==6))
	{
        F = _FM.subVector(0,2);
        Mu = _FM.subVector(3,5);
		return true;
	}
	else
	{
		F.resize(3);	F.zero();
		Mu.resize(3);	Mu.zero();
	
		if(verbose)
            fprintf(stderr,"ContactNewtonEuler error: could not set F/Mu due to wrong dimensions: (%ld) instead of (6 = 3,3). Default zero is set.\n",_FM.length());
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool ContactNewtonEuler::setH(const Matrix &_H)
{
    if((_H.rows()==4)&&(_H.cols()==4))
	{
        H.resize(4,4); H=_H;
		return true;
	}
	else
	{
		H.resize(4,4);	H.eye();
		if(verbose)
            fprintf(stderr,"ContactNewtonEuler error: could not set H due to wrong dimensions: (%ld,%ld) instead of (4,4). Default identity is set.\n",_H.rows(),_H.cols());
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~
//   compute methods
//~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector ContactNewtonEuler::getForceMoment() const
{
	Vector ret(6); ret.zero();
	ret[0]=F[0]; ret[1]=F[1]; ret[2]=F[2];
	ret[3]=Mu[0]; ret[4]=Mu[1]; ret[5]=Mu[2];
	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string ContactNewtonEuler::toString() const
{
    string ret = "[Contact]: " + info;

    char buffer[300]; int j=0;		  
	j=sprintf(buffer," [Force]: %.3f,%.3f,%.3f",F(0),F(1),F(2));
	j+=sprintf(buffer+j," [Moment]: %.3f,%.3f,%.3f",Mu(0),Mu(1),Mu(2));	
    Vector r = getr();
	j+=sprintf(buffer+j," [r]: %.3f,%.3f,%.3f",r(0),r(1),r(2));
		
	ret.append(buffer);
	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ContactNewtonEuler::computeExternalWrench(OneLinkNewtonEuler *link, OneLinkNewtonEuler *next)
{
    //a compute backward
    F = getR() * ( next->getR() * ( next->getForce() + next->getMass() * next->getLinAccC() ) - link->getForce() );

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//================================
//
//		ONE LINK NEWTON EULER
//
//================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
OneLinkNewtonEuler::OneLinkNewtonEuler(iDynLink *dlink)
{
	mode = DYNAMIC;
	verbose = NO_VERBOSE;
	info = "link";
	link = dlink;
	z0.resize(3); z0.zero(); z0(2)=1;	
	zm.resize(3); zm.zero();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
OneLinkNewtonEuler::OneLinkNewtonEuler(const NewEulMode _mode, unsigned int verb, iDynLink *dlink)
{
	mode = _mode;
	verbose = verb;
	info = "link";
	link = dlink;
	z0.resize(3); z0.zero(); z0(2)=1;	
	zm.resize(3); zm.zero();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::zero()
{
	z0.resize(3); z0.zero(); z0(2)=1;	
	zm.resize(3); zm.zero();
	if (link != NULL) link->zero();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	 //~~~~~~~~~~~~~~~~~~~~~~
	 //   set methods
	 //~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneLinkNewtonEuler::setAsBase(const Vector &_w, const Vector &_dw, const Vector &_ddp)
{
	zero();
	setAngVel(_w);
	setAngAcc(_dw);
	setLinAcc(_ddp);
	info = "base";
	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneLinkNewtonEuler::setAsBase(const Vector &_F, const Vector &_Mu)
{
	zero();
	setForce(_F);
	setMoment(_Mu);
	info = "base";
	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneLinkNewtonEuler::setAsFinal(const Vector &_F, const Vector &_Mu)
{
	zero();
	setForce(_F);
	setMoment(_Mu);
	info = "final";
	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneLinkNewtonEuler::setAsFinal(const Vector &_w, const Vector &_dw, const Vector &_ddp)
{
	zero();
	setAngVel(_w); 
	setAngAcc(_dw); 
	setLinAcc(_ddp);
	info = "final";
	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::setVerbose(unsigned int verb)
{
	verbose = verb;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::setMode(const NewEulMode _mode)
{
	mode = _mode;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneLinkNewtonEuler::setZM(const Vector &_zm)
{
	if(_zm.length()==3)
	{
		zm = _zm;
		return true;
	}
	else
	{
		if(verbose)
        {
            fprintf(stderr,"OneLinkNewtonEuler error: could not set Zm due to wrong size vector: ");
            fprintf(stderr,"%ld instead of 3. \n",_zm.length());
        }
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneLinkNewtonEuler::setForce(const Vector &_F)
{
	if(_F.length()==3)
	{
		link->F = _F;
		return true;
	}
	else
	{
		if(verbose)	fprintf(stderr,"OneLink error, could not set force due to wrong size: %ld instead of 3.\n",_F.length());
		return false;		
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneLinkNewtonEuler::setMoment(const Vector &_Mu)
{
	if(_Mu.length()==3)
	{
		link->Mu = _Mu;
		return true;
	}
	else
	{
		if(verbose)	fprintf(stderr,"OneLink error, could not set moment due to wrong size: %ld instead of 3.\n",_Mu.length());
		return false;		
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::setTorque(const double _Tau)
{
	link->Tau = _Tau;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneLinkNewtonEuler::setAngVel(const Vector &_w)
{
	if(_w.length()==3)
	{
		link->w = _w; 
		return true;
	}
	else
	{
		if(verbose) fprintf(stderr,"OneLink error, could not set w due to wrong size: %ld instead of 3. \n", _w.length());
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneLinkNewtonEuler::setAngAcc(const Vector &_dw)
{
	if(_dw.length()==3)
	{
		link->dw = _dw; 
		return true;
	}
	else
	{
		if(verbose)	fprintf(stderr,"OneLink error, could not set dw due to wrong size: %ld instead of 3. \n",_dw.length() );
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneLinkNewtonEuler::setLinAcc(const Vector &_ddp)
{
	if(_ddp.length()==3)
	{
		link->ddp = _ddp; 
		return true;
	}
	else
	{
		if(verbose)	fprintf(stderr,"OneLink error, could not set ddp due to wrong size: %ld instead of 3. \n",_ddp.length() );
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneLinkNewtonEuler::setLinAccC(const Vector &_ddpC)
{
	if(_ddpC.length()==3)
	{
		link->ddpC = _ddpC; 
		return true;
	}
	else
	{
		if(verbose)	fprintf(stderr,"OneLink error, could not set ddpC due to wrong size: %ld instead of 3. \n",_ddpC.length() );
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneLinkNewtonEuler::setAngAccM(const Vector &_dwM)
{
	if(_dwM.length()==3)
	{
		link->dwM = _dwM; 
		return true;
	}
	else
	{
		if(verbose)	fprintf(stderr,"OneLink error, could not set dwM due to wrong size: %ld instead of 3. \n",_dwM.length());
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::setInfo(const string &_info)
{
	info = _info;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneLinkNewtonEuler::setMeasuredFMu(const Vector &_F, const Vector &_Mu)
{	
	if(link != NULL) 
	{
		if((_F.length()==3)&&(_Mu.length()==3))
		{
			link->setForceMoment(_F,_Mu);
			return true;
		}
		else
		{
			if(verbose) fprintf(stderr,"OneLinkNewtonEuler error: could not set forces/moments due to wrong dimensions: (%ld,%ld) instead of (3,3). \n",_F.length(),_Mu.length());
			return false;
		}		
	}
	else		
	{
		if(verbose) fprintf(stderr,"OneLinkNewtonEuler error: could not set forces/moments due to missing link. \n");
		return false;
	}
	

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneLinkNewtonEuler::setMeasuredTorque(const double _Tau)
{
	if(link != NULL) 
	{
		link->setTorque(_Tau);
		return true;
	}
	else
	{
		if(verbose) fprintf(stderr,"OneLinkNewtonEuler error: could not set torque due to missing link. \n");
		return false;
	}

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string OneLinkNewtonEuler::toString() const
{
	string ret = "[Link/frame]: " + info + " [Mode]: " + NewEulMode_s[mode];

	char buffer[300]; int j=0;		  
	j=sprintf(buffer," [Torque]: %f",link->Tau);
	j+=sprintf(buffer+j," [Force]: %.3f,%.3f,%.3f",link->F(0),link->F(1),link->F(2));
	j+=sprintf(buffer+j," [Moment]: %.3f,%.3f,%.3f",link->Mu(0),link->Mu(1),link->Mu(2));	
	if(verbose)
	{
		j+=sprintf(buffer+j," [mass]: %.3f",link->m);
		Vector r = link->getr();
		j+=sprintf(buffer+j," [r]: %.3f,%.3f,%.3f",r(0),r(1),r(2));
		r = link->getrC();
		j+=sprintf(buffer+j," [rC]: %.3f,%.3f,%.3f",r(0),r(1),r(2));

		if(mode!=STATIC)
		{
			Matrix I = link->getInertia();
			j+=sprintf(buffer+j," [Inertia]: %.3f, %.3f, %.3f; %.3f, %.3f, %.3f; %.3f, %.3f, %.3f",I(0,0),I(0,1),I(0,2),I(1,0),I(1,1),I(1,2),I(2,0),I(2,1),I(2,2));
		}
	}
	ret.append(buffer);
	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


	 //~~~~~~~~~~~~~~~~~~~~~~
	 //   get methods
	 //~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
NewEulMode OneLinkNewtonEuler::getMode() const		{ return mode;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector OneLinkNewtonEuler::getZM() const			{ return zm;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector OneLinkNewtonEuler::getAngVel()	const		{ return link->w;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector OneLinkNewtonEuler::getAngAcc()	const		{ return link->dw;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector OneLinkNewtonEuler::getAngAccM() const		{ return link->dwM;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector OneLinkNewtonEuler::getLinAcc() const		{ return link->ddp;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector OneLinkNewtonEuler::getLinAccC() const		{ return link->ddpC;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector OneLinkNewtonEuler::getForce() const			{ return link->F;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector OneLinkNewtonEuler::getMoment(bool isBase) const		{ return link->Mu;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double OneLinkNewtonEuler::getTorque() const		{ return link->Tau;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string OneLinkNewtonEuler::getInfo() const			{ return info;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix OneLinkNewtonEuler::getR()					{ return link->getR();}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix OneLinkNewtonEuler::getRC()					{ return link->getRC();}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double OneLinkNewtonEuler::getIm() const			{ return link->Im;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double OneLinkNewtonEuler::getD2q() const			{ return link->ddq;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double OneLinkNewtonEuler::getDq() const			{ return link->dq;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double OneLinkNewtonEuler::getKr() const			{ return link->kr;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double OneLinkNewtonEuler::getFs() const			{ return link->Fs;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double OneLinkNewtonEuler::getFv() const			{ return link->Fv;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double OneLinkNewtonEuler::getMass()const			{ return link->m;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix	OneLinkNewtonEuler::getInertia()const		{ return link->getInertia();}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	OneLinkNewtonEuler::getr(bool proj)			{ return link->getr(proj);}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	OneLinkNewtonEuler::getrC(bool proj)		{ return link->getrC(proj);} 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	 //~~~~~~~~~~~~~~~~~~~~~~
	 //   core computation
	 //~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::computeAngVel( OneLinkNewtonEuler *prev)
{
	switch(mode)
	{
	case DYNAMIC_CORIOLIS_GRAVITY:
	case DYNAMIC:
	case DYNAMIC_W_ROTOR:
		setAngVel( (getR()).transposed() * ( prev->getAngVel() + getDq() * z0 ));
		break;
	case STATIC:
		Vector av(3); av.zero();
		setAngVel(av);
		break;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::computeAngVelBackward( OneLinkNewtonEuler *next)
{
	switch(mode)
	{
	case DYNAMIC_CORIOLIS_GRAVITY:
	case DYNAMIC:
	case DYNAMIC_W_ROTOR:
		setAngVel( next->getR() * ( next->getAngVel()) - next->getDq() * z0 );
		break;
	case STATIC:
		Vector av(3); av.zero();
		setAngVel(av);
		break;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::computeAngAcc( OneLinkNewtonEuler *prev)
{
	switch(mode)
	{
	case DYNAMIC:
	case DYNAMIC_W_ROTOR:
		setAngAcc( (getR()).transposed() * ( prev->getAngAcc() + getD2q() * z0 + getDq() * cross(prev->getAngVel(),z0) ));
		break;
	case DYNAMIC_CORIOLIS_GRAVITY:
		setAngAcc( (getR()).transposed() * ( prev->getAngAcc() + getDq() * cross(prev->getAngVel(),z0) ));
		break;
	case STATIC:
		Vector aa(3); aa.zero();
		setAngAcc(aa);
		break;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::computeAngAccBackward( OneLinkNewtonEuler *next)
{
	switch(mode)
	{
	case DYNAMIC:
	case DYNAMIC_W_ROTOR:
		setAngAcc( next->getR() * next->getAngAcc() - next->getD2q() * z0 - next->getDq() * cross(getAngVel(),z0) );
		break;
	case DYNAMIC_CORIOLIS_GRAVITY:
		setAngAcc( next->getR() * next->getAngAcc() - next->getDq() * cross(getAngVel(),z0) );
		break;
	case STATIC:
		Vector aa(3); aa.zero();
		setAngAcc(aa);
		break;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::computeLinAcc( OneLinkNewtonEuler *prev)
{
	switch(mode)
	{
	case DYNAMIC:
	case DYNAMIC_CORIOLIS_GRAVITY:
	case DYNAMIC_W_ROTOR:
		setLinAcc( (getR()).transposed() * prev->getLinAcc() 
			+ cross(getAngAcc(),getr(true))
			+ cross(getAngVel(),cross(getAngVel(),getr(true))) );
		break;
	case STATIC:
		setLinAcc( (getR()).transposed() * prev->getLinAcc() );
		break;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::computeLinAccBackward( OneLinkNewtonEuler *next)
{
	switch(mode)
	{
	case DYNAMIC:
	case DYNAMIC_CORIOLIS_GRAVITY:
	case DYNAMIC_W_ROTOR:
		setLinAcc(next->getR() * (next->getLinAcc() 
			- cross(next->getAngAcc(),next->getr(true)) 
			- cross(next->getAngVel(),cross(next->getAngVel(),next->getr(true))) ));
		break;
	case STATIC:
		setLinAcc( next->getR() * next->getLinAcc() );
		break;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::computeLinAccC()
{
	switch(mode)
	{
	case DYNAMIC:
	case DYNAMIC_CORIOLIS_GRAVITY:
	case DYNAMIC_W_ROTOR:
		setLinAccC( getLinAcc() + cross(getAngAcc(),getrC(true)) + cross(getAngVel(),cross(getAngVel(),getrC(true))));
		break;
	case STATIC:
		setLinAccC( getLinAcc());
		break;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::computeAngAccM( OneLinkNewtonEuler *prev)
{
	switch(mode)
	{
	case DYNAMIC_CORIOLIS_GRAVITY:
	case DYNAMIC:
		setAngAccM( prev->getAngAcc());
		break;
	case DYNAMIC_W_ROTOR:
		setAngAccM( prev->getAngAcc() + (getKr() * getD2q()) * zm
			+ (getKr() * getDq()) * cross(prev->getAngVel(),zm) );
		break;
	case STATIC:
		Vector aaM(3); aaM.zero();
		setAngAccM( aaM );
		break;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::computeForceBackward( OneLinkNewtonEuler *next)
{
	setForce( next->getR() * ( next->getForce() + next->getMass() * next->getLinAccC() ) );
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::computeForceForward( OneLinkNewtonEuler *prev)
{
	setForce( getR().transposed() * prev->getForce() - getMass() * getLinAccC() );
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::computeMomentBackward( OneLinkNewtonEuler *next)
{
	switch(mode)
	{
	case DYNAMIC_CORIOLIS_GRAVITY:
	case DYNAMIC:		
	setMoment( cross(next->getr() , next->getR() * next->getForce()) 
			+ cross(next->getr() + next->getR() * next->getrC() , next->getR() * (next->getMass() * next->getLinAccC())) 
			+ next->getR() * next->getMoment(false)
			+ next->getR() * next->getInertia() * next->getAngAcc() 
			+ next->getR() * cross( next->getAngVel() , next->getInertia() * next->getAngVel())
			);
		break;
	case DYNAMIC_W_ROTOR:
	setMoment( cross(next->getr() , next->getR() * next->getForce()) 
			+ cross(next->getr() + next->getR() * next->getrC() , next->getR() * (next->getMass() * next->getLinAccC())) 
			+ next->getR() * next->getMoment()
			+ next->getR() * next->getInertia() * next->getAngAcc() 
			+ next->getR() * cross( next->getAngVel() , next->getInertia() * next->getAngVel())
			+ next->getKr() * next->getD2q() * next->getIm() * next->getZM()
			+ next->getKr() * next->getDq() * next->getIm() * cross(next->getAngVel(),next->getZM()) );
		break;
	case STATIC:
	setMoment( cross(next->getr(),next->getR()*next->getForce()) 
			+ cross(next->getr()+next->getR()*next->getrC(),next->getR()*(next->getMass() * next->getLinAccC())) 
			+ next->getR() * next->getMoment(false));
		break;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::computeMomentForward( OneLinkNewtonEuler *prev)
{
	switch(mode)
	{
	case DYNAMIC_CORIOLIS_GRAVITY:
	case DYNAMIC:
		setMoment( getR().transposed() * ( prev->getMoment(false) - cross(getr(),getR()*getForce())
			- cross(getr()+getR()*getrC(),getR()*(getMass() * getLinAccC()))
			- getR() * getInertia() * getAngAcc()
			- getR() * cross( getAngVel() , getInertia() * getAngVel())	) );
		break;
	case DYNAMIC_W_ROTOR:
		setMoment( getR().transposed() * ( prev->getMoment(false) - cross(getr(),getR()*getForce())
			- cross(getr()+getR()*getrC(),getR()*(getMass() * getLinAccC()))
			- getR() * getInertia() * getAngAcc()
			- getR() * cross( getAngVel() , getInertia() * getAngVel())	
			- getKr() * getD2q() * getIm() * getZM()
			- getKr() * getDq() * getIm() * cross(getAngVel(),getZM())	
			));
		break;
	case STATIC:
		setMoment( getR().transposed() * ( prev->getMoment(false) - cross(getr(),getR()*getForce())
			- cross(getr()+getR()*getrC(),getR()*(getMass() * getLinAccC()))
			) );
		break;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::computeTorque(OneLinkNewtonEuler *prev)
{
	switch(mode)
	{
	case DYNAMIC_CORIOLIS_GRAVITY:
	case DYNAMIC:
	case STATIC:
		setTorque( dot(prev->getMoment(true),z0) );
		break;
	case DYNAMIC_W_ROTOR:
		setTorque( dot(prev->getMoment(true),z0) + getKr() * getIm() * dot(getAngAccM(),zm) 
			+ getFv() * getDq() + getFs() * sign(getDq()) );
		break;		
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	 //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	 //    main computation methods
	 //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::ForwardKinematics( OneLinkNewtonEuler *prev)
{
	this->computeAngVel(prev);
	this->computeAngAcc(prev);
	this->computeLinAcc(prev);
	this->computeLinAccC();
	this->computeAngAccM(prev);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::BackwardKinematics( OneLinkNewtonEuler *prev)
{
	this->computeAngVelBackward(prev);
	this->computeAngAccBackward(prev);
	this->computeLinAccBackward(prev);
	this->computeLinAccC();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::BackwardWrench( OneLinkNewtonEuler *next)
{
	this->computeForceBackward(next);
	this->computeMomentBackward(next);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneLinkNewtonEuler::ForwardWrench( OneLinkNewtonEuler *prev)
{
	this->computeForceForward(prev);
	this->computeMomentForward(prev);
	this->computeTorque(prev);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//================================
//
//		BASE LINK NEWTON EULER
//
//================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
BaseLinkNewtonEuler::BaseLinkNewtonEuler(const Matrix &_H0, const NewEulMode _mode, unsigned int verb)
: OneLinkNewtonEuler(_mode,verb,NULL)
{
	info = "base";
	w.resize(3);	w.zero();
	dw.resize(3);	dw.zero();
	ddp.resize(3);	ddp.zero();
	H0.resize(4,4);	H0.eye();
	if((_H0.rows()==4)&&(_H0.cols()==4))
		H0 = _H0;
	else
		if(verbose)
        {
            fprintf(stderr,"BaseLink error, could not set H0 due to wrong dimensions: ( %ld,%ld) instead of (4,4). \n",_H0.rows(),_H0.cols());
            fprintf(stderr," Default is set. \n");
        }
	F.resize(3);	F.zero();
	Mu.resize(3);	Mu.zero();
	Tau = 0.0;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
BaseLinkNewtonEuler::BaseLinkNewtonEuler(const Matrix &_H0, const Vector &_w, const Vector &_dw, const Vector &_ddp, const NewEulMode _mode, unsigned int verb)
: OneLinkNewtonEuler(_mode,verb,NULL)
{
	info = "base";
	w.resize(3);	w.zero();
	dw.resize(3);	dw.zero();
	ddp.resize(3);	ddp.zero();
	H0.resize(4,4);	H0.eye();
	if((_H0.rows()==4)&&(_H0.cols()==4))
		H0 = _H0;
	else
		if(verbose)
        {
            fprintf(stderr,"BaseLink error, could not set H0 due to wrong dimensions: ( %ld,%ld) instead of (4,4). \n",_H0.rows(),_H0.cols());
            fprintf(stderr," Default is set. \n");
        }
	F.resize(3);	F.zero();
	Mu.resize(3);	Mu.zero();
	Tau = 0.0;
	setAsBase(_w,_dw,_ddp);			
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool BaseLinkNewtonEuler::setAsBase(const Vector &_w, const Vector &_dw, const Vector &_ddp)
{
	if((_w.length()==3)&&(_dw.length()==3)&&(_ddp.length()==3))
	{
		this->setAngVel(_w);
		this->setAngAcc(_dw);
		this->setLinAcc(_ddp);
		return true;
	}
	else
	{
		w.resize(3);	w.zero();
		dw.resize(3);	dw.zero();
		ddp.resize(3);	ddp.zero();
	
		if(verbose)
        {
            fprintf(stderr,"BaseLinkNewtonEuler error: could not set w/dw/ddp due to wrong dimensions: (%ld,%ld,%ld) instead of (3,3,3). ",_w.length(),_dw.length(),_ddp.length());
            fprintf(stderr," Default is set. \n");
        }
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool BaseLinkNewtonEuler::setAsBase(const Vector &_F, const Vector &_Mu)
{
	if((_F.length()==3)&&(_Mu.length()==3))
	{
		F = _F;
		Mu = _Mu;
		return true;
	}
	else
	{
		F.resize(3);	F.zero();
		Mu.resize(3);	Mu.zero();
	
		if(verbose)
        {
            fprintf(stderr,"FinalLinkNewtonEuler error: could not set F/Mu due to wrong dimensions: (%ld,%ld) instead of (3,3).",_F.length(),_Mu.length());
			fprintf(stderr," Default is set. \n");
        }
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string BaseLinkNewtonEuler::toString() const
{
	string ret = "[Base link]: " + info + " [Mode]: " + NewEulMode_s[mode];

	char buffer[300]; int j=0;		  
	if(verbose)
	{
		j=sprintf(buffer," [w]  : %.3f,%.3f,%.3f",w(0),w(1),w(2));
		j+=sprintf(buffer+j," [dw] : %.3f,%.3f,%.3f",dw(0),dw(1),dw(2));
		j+=sprintf(buffer+j," [ddp]: %.3f,%.3f,%.3f",ddp(0),ddp(1),ddp(2));
	}
	ret.append(buffer);
	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	 //~~~~~~~~~~~~~~~~~~~~~~
	 //   get methods
	 //~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	BaseLinkNewtonEuler::getAngVel()		const	{return w;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	BaseLinkNewtonEuler::getAngAcc()		const	{return dw;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	BaseLinkNewtonEuler::getAngAccM()		const	{return dw;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	BaseLinkNewtonEuler::getLinAcc()		const	{return ddp; }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	BaseLinkNewtonEuler::getLinAccC()		const	{return getLinAcc();}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	BaseLinkNewtonEuler::getForce()	const	{return F;}
Vector	BaseLinkNewtonEuler::getMoment(bool isBase)const	{ if(isBase==false) return Mu; else return H0.submatrix(0,2,0,2).transposed() * Mu;}
double	BaseLinkNewtonEuler::getTorque()const	{return Tau;}
Matrix	BaseLinkNewtonEuler::getR()				{Matrix ret(3,3); ret.eye(); return ret;}
Matrix	BaseLinkNewtonEuler::getRC()			{Matrix ret(3,3); ret.eye(); return ret;}
double	BaseLinkNewtonEuler::getIm()	const	{return 0.0;}
double	BaseLinkNewtonEuler::getFs()	const	{return 0.0;}
double	BaseLinkNewtonEuler::getFv()	const	{return 0.0;}
double	BaseLinkNewtonEuler::getD2q()	const	{return 0.0;}
double	BaseLinkNewtonEuler::getDq()	const	{return 0.0;}
double	BaseLinkNewtonEuler::getKr()	const	{return 0.0;}
double	BaseLinkNewtonEuler::getMass()	const	{return 0.0;}
Matrix	BaseLinkNewtonEuler::getInertia()const	{Matrix ret(3,3); ret.zero(); return ret;}
Vector	BaseLinkNewtonEuler::getr(bool proj) {Vector v(3); v.zero(); return v;}
Vector	BaseLinkNewtonEuler::getrC(bool proj){Vector v(3); v.zero(); return v;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool BaseLinkNewtonEuler::setForce(const Vector &_F)	
{
	if(_F.length()==3)
	{
		F=H0.submatrix(0,2,0,2)*_F;
		return true;
	}
	else
	{
		if(verbose)
			fprintf(stderr,"BaseLink error, could not set force due to wrong dimension: %ld instead of 3.\n",_F.length());
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool BaseLinkNewtonEuler::setMoment(const Vector &_Mu)	
{
	if(_Mu.length()==3)
	{
		Mu=H0.submatrix(0,2,0,2)*_Mu;
		return true;
	}
	else
	{
		if(verbose)
			fprintf(stderr,"BaseLink error, could not set moment due to wrong dimension: %ld instead of 3.\n",_Mu.length());
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void BaseLinkNewtonEuler::setTorque(const double _Tau)		{Tau=_Tau;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool BaseLinkNewtonEuler::setAngVel(const Vector &_w)
{
	if(_w.length()==3)
	{
		w = H0.submatrix(0,2,0,2).transposed()*_w; 
		return true;
	}
	else
	{
		if(verbose)
			fprintf(stderr,"BaseLink error, could not set w due to wrong size: %ld instead of 3. \n",_w.length() );
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool BaseLinkNewtonEuler::setAngAcc(const Vector &_dw)
{
	if(_dw.length()==3)
	{
		dw = H0.submatrix(0,2,0,2).transposed()*_dw; 
		return true;
	}
	else
	{
		if(verbose)
			fprintf(stderr,"BaseLink error, could not set dw due to wrong size: %ld instead of 3. \n",_dw.length());
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool BaseLinkNewtonEuler::setLinAcc(const Vector &_ddp)
{
	if(_ddp.length()==3)
	{
		ddp = H0.submatrix(0,2,0,2).transposed()*_ddp; 
		return true;
	}
	else
	{
		if(verbose)
			fprintf(stderr,"BaseLink error, could not set ddp due to wrong size: %ld instead of 3. \n",_ddp.length());
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool BaseLinkNewtonEuler::setLinAccC(const Vector &_ddpC)
{
	if(verbose)
		fprintf(stderr,"BaseLink error: no ddpC existing \n");
	return false;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool BaseLinkNewtonEuler::setAngAccM(const Vector &_dwM)
{
	if(verbose)
		fprintf(stderr,"BaseLink error: no dwM existing \n");
	return false;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//================================
//
//		FINAL LINK NEWTON EULER
//
//================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
FinalLinkNewtonEuler::FinalLinkNewtonEuler(const NewEulMode _mode, unsigned int verb)
: OneLinkNewtonEuler(_mode,verb,NULL)
{
	info = "final";
	F.resize(3);	F.zero();
	Mu.resize(3);	Mu.zero();
	w.resize(3);	w.zero();
	dw.resize(3);	dw.zero();
	ddp.resize(3);	ddp.zero();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
FinalLinkNewtonEuler::FinalLinkNewtonEuler(const Vector &_F, const Vector &_Mu, const NewEulMode _mode, unsigned int verb)
: OneLinkNewtonEuler(_mode,verb,NULL)
{
	info = "final";
	F.resize(3);	F.zero();
	Mu.resize(3);	Mu.zero();
	w.resize(3);	w.zero();
	dw.resize(3);	dw.zero();
	ddp.resize(3);	ddp.zero();
	setAsFinal(_F,_Mu);
			
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool FinalLinkNewtonEuler::setAsFinal(const Vector &_w, const Vector &_dw, const Vector &_ddp)
{
	if((_w.length()==3)&&(_dw.length()==3)&&(_ddp.length()==3))
	{
		w = _w;
		dw = _dw;
		ddp = _ddp;
		return true;
	}
	else
	{
		w.resize(3);	w.zero();
		dw.resize(3);	dw.zero();
		ddp.resize(3);	ddp.zero();
	
		if(verbose)
        {
            fprintf(stderr,"FinalLinkNewtonEuler error: could not set w/dw/ddp due to wrong dimensions: (%ld,%ld,%ld) instead of (3,3,3).",	_w.length(),_dw.length(),_ddp.length());
			fprintf(stderr," Default is set. \n");
        }
		return false;
	}
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool FinalLinkNewtonEuler::setAsFinal(const Vector &_F, const Vector &_Mu)
{
	if((_F.length()==3)&&(_Mu.length()==3))
	{
		F = _F;
		Mu = _Mu;
		return true;
	}
	else
	{
		F.resize(3);	F.zero();
		Mu.resize(3);	Mu.zero();
	
		if(verbose)
        {
            fprintf(stderr,"FinalLinkNewtonEuler error: could not set F/Mu due to wrong dimensions: (%ld,%ld) instead of (3,3).",_F.length(),_Mu.length());
			fprintf(stderr," Default is set. \n");
        }
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string FinalLinkNewtonEuler::toString() const
{
	string ret = "[Final link]: " + info + " [Mode]: " + NewEulMode_s[mode];

	char buffer[300]; int j=0;		  
	if(verbose)
	{
		j=sprintf(   buffer," [F]  : %.3f,%.3f,%.3f",F(0),F(1),F(2));
		j+=sprintf(buffer+j," [Mu] : %.3f,%.3f,%.3f",Mu(0),Mu(1),Mu(2));
	}
	ret.append(buffer);
	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	 //~~~~~~~~~~~~~~~~~~~~~~
	 //   get methods
	 //~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	FinalLinkNewtonEuler::getForce()		        const	{return F;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	FinalLinkNewtonEuler::getMoment(bool isBase)	const	{return Mu;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	FinalLinkNewtonEuler::getAngVel()	const	{return w;}
Vector	FinalLinkNewtonEuler::getAngAcc()	const	{return dw;}
Vector	FinalLinkNewtonEuler::getAngAccM()	const	{Vector v(3); v.zero(); return v;}
Vector	FinalLinkNewtonEuler::getLinAcc()	const	{return ddp;}
Vector	FinalLinkNewtonEuler::getLinAccC()	const	{Vector v(3); v.zero(); return v;}
double	FinalLinkNewtonEuler::getTorque()	const	{return 0.0;}
Matrix	FinalLinkNewtonEuler::getR()				{Matrix ret(3,3); ret.eye(); return ret;}	
Matrix	FinalLinkNewtonEuler::getRC()				{Matrix ret(3,3); ret.eye(); return ret;}
double	FinalLinkNewtonEuler::getIm()		const	{return 0.0;}
double	FinalLinkNewtonEuler::getFs()		const	{return 0.0;}
double	FinalLinkNewtonEuler::getFv()		const	{return 0.0;}
double	FinalLinkNewtonEuler::getD2q()		const	{return 0.0;}
double	FinalLinkNewtonEuler::getDq()		const	{return 0.0;}
double	FinalLinkNewtonEuler::getKr()		const	{return 0.0;}
double	FinalLinkNewtonEuler::getMass()		const	{return 0.0;}
Matrix	FinalLinkNewtonEuler::getInertia()const		{Matrix ret(3,3); ret.zero(); return ret;}
Vector	FinalLinkNewtonEuler::getr(bool proj)		{Vector v(3); v.zero(); return v;}
Vector	FinalLinkNewtonEuler::getrC(bool proj)		{Vector v(3); v.zero(); return v;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool	FinalLinkNewtonEuler::setForce(const Vector &_F)
{
	if(_F.length()==3)
	{
		F=_F;
		return true;
	}
	else
	{
		if(verbose)
			fprintf(stderr,"FinalLink error, could not set force due to wrong dimension: %ld instead of 3.\n",_F.length());
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool	FinalLinkNewtonEuler::setMoment(const Vector &_Mu)
{
	if(_Mu.length()==3)
	{
		Mu=_Mu;
		return true;
	}
	else
	{
		if(verbose)
			fprintf(stderr,"FinalLink error, could not set moment due to wrong dimension: %ld instead of 3.\n",_Mu.length());
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void	FinalLinkNewtonEuler::setTorque(const double _Tau){}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool FinalLinkNewtonEuler::setAngVel(const Vector &_w)
{
	/*if(verbose)
		fprintf(stderr,"FinalLink error: no w existing \n");
	return false;*/
	w=_w;
	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool FinalLinkNewtonEuler::setAngAcc(const Vector &_dw)
{
	//if(verbose)
	//	fprintf(stderr,"FinalLink error: no dw existing \n");
	//return false;
	dw=_dw;	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool FinalLinkNewtonEuler::setLinAcc(const Vector &_ddp)
{
	//if(verbose)
	//	fprintf(stderr,"FinalLink error: no ddp existing \n");
	//return false;
	ddp=_ddp;	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool FinalLinkNewtonEuler::setLinAccC(const Vector &_ddpC)
{
	if(verbose)
		fprintf(stderr,"FinalLink error: no ddpC existing \n");
	return false;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool FinalLinkNewtonEuler::setAngAccM(const Vector &_dwM)
{
	if(verbose)
		fprintf(stderr,"FinalLink error: no dwM existing \n");
	return false;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//================================
//
//		SENSOR LINK NEWTON EULER
//
//================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
SensorLinkNewtonEuler::SensorLinkNewtonEuler(const NewEulMode _mode, unsigned int verb)
: OneLinkNewtonEuler(_mode,verb,NULL)
{
	info = "sensor";
	F.resize(3);	F.zero();
	Mu.resize(3);	Mu.zero();
	w.resize(3);	w.zero();
	dw.resize(3);	dw.zero();
	ddp.resize(3);	ddp.zero();
	ddpC.resize(3);	ddpC.zero();
	H.resize(4,4); H.eye();
	COM.resize(4,4); COM.eye();
	I.resize(3,3); I.zero();
	m=0.0;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
SensorLinkNewtonEuler::SensorLinkNewtonEuler(const Matrix &_H, const Matrix &_COM, const double _m, const Matrix &_I, const NewEulMode _mode, unsigned int verb)
: OneLinkNewtonEuler(_mode,verb,NULL)
{
	info = "sensor";
	F.resize(3);	F.zero();
	Mu.resize(3);	Mu.zero();
	w.resize(3);	w.zero();
	dw.resize(3);	dw.zero();
	ddp.resize(3);	ddp.zero();
	ddpC.resize(3);	ddpC.zero();
	H.resize(4,4); H.eye();
	COM.resize(4,4); COM.eye();
	I.resize(3,3); I.zero();
	setSensor(_H,_COM,_m,_I);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool SensorLinkNewtonEuler::setMeasuredFMu(const Vector &_F, const Vector &_Mu)
{
	if((_F.length()==3)&&(_Mu.length()==3))
	{
		F = _F;
		Mu = _Mu;
		return true;
	}
	else
	{
		F.resize(3);	F.zero();
		Mu.resize(3);	Mu.zero();
	
		if(verbose)
			fprintf(stderr,"SensorLinkNewtonEuler error: could not set F/Mu due to wrong dimensions: (%ld,%ld) instead of (3,3). Default zero is set.\n",_F.length(),_Mu.length());
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool SensorLinkNewtonEuler::setSensor(const Matrix &_H, const Matrix &_COM, const double _m, const Matrix &_I)
{
	if((_COM.rows()==4)&&(_COM.cols()==4) && (_H.cols()==4) && (_H.rows()==4) && (_I.rows()==3) && (_I.cols()==3))
	{
		H.resize(4,4); COM.resize(4,4); I.resize(3,3);
		H = _H;
		COM = _COM;
		I = _I;
		m = _m;
		return true;
	}
	else
	{
		m = _m;
		H.resize(4,4); H.eye();
		COM.resize(4,4); COM.eye();
		I.resize(3,3); I.zero();
		if(verbose)
        {
            fprintf(stderr,"SensorLink error, could not set properly H,COM,I due to wrong dimensions: \n");
            fprintf(stderr,"    H:   (%ld,%ld) instead of (4,4) \n",_H.rows(),_H.cols());
            fprintf(stderr,"  COM:   (%ld,%ld) instead of (4,4) \n",_COM.rows(),_COM.cols()); 
            fprintf(stderr,"    I:   (%ld,%ld) instead of (3,3) \n",_I.rows(),_I.cols()); 
			fprintf(stderr,"Setting identities and zeros by default.");
        }
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string SensorLinkNewtonEuler::toString() const
{
	string ret = "[Sensor link]: " + info + " [Mode]: " + NewEulMode_s[mode];

	char buffer[300]; int j=0;		  
	if(verbose)
	{
		j=sprintf(   buffer," [F]  : %.3f,%.3f,%.3f",F(0),F(1),F(2));
		j+=sprintf(buffer+j," [Mu] : %.3f,%.3f,%.3f",Mu(0),Mu(1),Mu(2));
		j+=sprintf(buffer+j," [R sens] : %.3f,%.3f,%.3f ; %.3f,%.3f,%.3f ; %.3f,%.3f,%.3f ",H(0,0),H(0,1),H(0,2),H(1,0),H(1,1),H(1,2),H(2,0),H(2,1),H(2,2));
		j+=sprintf(buffer+j," [p sens] : %.3f,%.3f,%.3f",H(0,3),H(1,3),H(2,3));
	}
	ret.append(buffer);
	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensorLinkNewtonEuler::ForwardAttachToLink(iDynLink *link)
{
	computeAngVel(link);
	computeAngAcc(link);
	computeLinAcc(link);
	computeLinAccC();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensorLinkNewtonEuler::BackwardAttachToLink(iDynLink *link)
{
	computeForce(link);
	computeMoment(link);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensorLinkNewtonEuler::ForwardForcesMomentsToLink(iDynLink *link)
{
	computeForceToLink(link);
	computeMomentToLink(link);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	 //~~~~~~~~~~~~~~~~~~~~~~
	 //   get methods
	 //~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	SensorLinkNewtonEuler::getForce()	const	{ return F;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	SensorLinkNewtonEuler::getMoment(bool isBase)	const	{ return Mu;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	SensorLinkNewtonEuler::getAngVel()	const	{ return w;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	SensorLinkNewtonEuler::getAngAcc()	const	{ return dw;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	SensorLinkNewtonEuler::getLinAcc()	const	{ return ddp;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	SensorLinkNewtonEuler::getLinAccC()	const	{ return ddpC;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double	SensorLinkNewtonEuler::getMass()	const	{ return m;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix	SensorLinkNewtonEuler::getInertia()	const	{ return I;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix	SensorLinkNewtonEuler::getR()				{ return H.submatrix(0,2,0,2);}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix	SensorLinkNewtonEuler::getRC()				{ return COM.submatrix(0,2,0,2);}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	SensorLinkNewtonEuler::getr(bool proj)		
{
	if(proj==false)
		return H.submatrix(0,2,0,3).getCol(3);
	else
		return (-1.0 * getR().transposed() * (H.submatrix(0,2,0,3)).getCol(3));
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	SensorLinkNewtonEuler::getrC(bool proj)	
{
	if(proj==false)
		return COM.submatrix(0,2,0,3).getCol(3);
	else
		return (-1.0 * getR().transposed() * COM.submatrix(0,2,0,3).getCol(3));
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double  SensorLinkNewtonEuler::getIm()		const	{return 0.0;}
double	SensorLinkNewtonEuler::getFs()		const	{return 0.0;}
double	SensorLinkNewtonEuler::getFv()		const	{return 0.0;}
double	SensorLinkNewtonEuler::getD2q()		const	{return 0.0;}
double	SensorLinkNewtonEuler::getDq()		const	{return 0.0;}
double	SensorLinkNewtonEuler::getKr()		const	{return 0.0;}
Vector	SensorLinkNewtonEuler::getAngAccM()	const	{return Vector(0);}
double	SensorLinkNewtonEuler::getTorque()	const	{return 0.0;}
Matrix	SensorLinkNewtonEuler::getH()		const	{return H;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool	SensorLinkNewtonEuler::setForce		(const Vector &_F)
{
	if(_F.length()==3)
	{
		F=_F;
		return true;
	}
	else
	{
		if(verbose)
			fprintf(stderr,"SensorLink error, could not set force due to wrong dimension: %ld instead of 3.\n",_F.length());
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool	SensorLinkNewtonEuler::setMoment	(const Vector &_Mu)
{
	if(_Mu.length()==3)
	{
		Mu=_Mu;
		return true;
	}
	else
	{
		if(verbose)
			fprintf(stderr,"SensorLink error, could not set moment due to wrong dimension: %ld instead of 3.\n",_Mu.length());
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void	SensorLinkNewtonEuler::setTorque	(const double _Tau)		{}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool	SensorLinkNewtonEuler::setAngVel	(const Vector &_w)
{
	if(_w.length()==3)
	{
		w = _w; 
		return true;
	}
	else
	{
		if(verbose)
			fprintf(stderr,"SensorLink error, could not set w due to wrong size: %ld instead of 3. \n",_w.length());
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool	SensorLinkNewtonEuler::setAngAcc	(const Vector &_dw)
{
	if(_dw.length()==3)
	{
		dw = _dw; 
		return true;
	}
	else
	{
		if(verbose)
			fprintf(stderr,"SensorLink error, could not set dw due to wrong size: %ld instead of 3. \n",_dw.length());
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool	SensorLinkNewtonEuler::setLinAcc	(const Vector &_ddp)
{
	if(_ddp.length()==3)
	{
		ddp = _ddp; 
		return true;
	}
	else
	{
		if(verbose)
			fprintf(stderr,"SensorLink error, could not set ddp due to wrong size: %ld instead of 3. \n",_ddp.length());
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool	SensorLinkNewtonEuler::setLinAccC	(const Vector &_ddpC)
{
	if(_ddpC.length()==3)
	{
		ddpC = _ddpC; 
		return true;
	}
	else
	{
		if(verbose)
			fprintf(stderr,"SensorLink error, could not set ddp due to wrong size: %ld instead of 3. \n",_ddpC.length());
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool	SensorLinkNewtonEuler::setAngAccM	(const Vector &_dwM)
{
	if(verbose)
		fprintf(stderr,"SensorLink error: no dwM existing \n");
	return false;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	 //~~~~~~~~~~~~~~~~~~~~~~
	 //  redefined methods
	 //~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensorLinkNewtonEuler::computeAngVel( iDynLink *link)
{
	w = link->getW();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensorLinkNewtonEuler::computeAngAcc( iDynLink *link)
{
	dw = link->getdW();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensorLinkNewtonEuler::computeLinAcc( iDynLink *link)
{
	switch(mode)
	{
	case DYNAMIC:
	case DYNAMIC_CORIOLIS_GRAVITY:
	case DYNAMIC_W_ROTOR:
		ddp = getR().transposed() * link->getLinAcc() 
			+ cross(dw,getr(true))
			+ cross(w,cross(w,getr(true)));
		break;
	case STATIC:
		ddp = getR().transposed() * link->getLinAcc();
		break;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensorLinkNewtonEuler::computeLinAccC()
{
	switch(mode)
	{
	case DYNAMIC:
	case DYNAMIC_CORIOLIS_GRAVITY:
	case DYNAMIC_W_ROTOR:
		ddpC = getLinAcc() + cross(getAngAcc(),getrC()) + cross(getAngVel(),cross(getAngVel(),getrC()));
		break;
	case STATIC:
		ddpC = ddp;
		break;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensorLinkNewtonEuler::computeForce(iDynLink *link)
{
	F = getR().transposed() * link->getForce() + m * getLinAccC()  ;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensorLinkNewtonEuler::computeForceToLink ( iDynLink *link)
{
	link->setForce( getR()*( F - m * getLinAccC() ) );
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensorLinkNewtonEuler::computeMomentToLink( iDynLink *link)
{
	switch(mode)
	{
	case DYNAMIC_CORIOLIS_GRAVITY:
	case DYNAMIC:	
		link->setMoment( getR()*( Mu - cross(getr(true),getR().transposed()*link->getForce())
			- cross(getrC(),(m * getLinAccC()))
			- getInertia() * getR().transposed() * getAngAcc() 
			- cross( getR().transposed() * getAngVel() , getInertia() * getR().transposed() * getAngVel())
			));
		break;
	case DYNAMIC_W_ROTOR:
		link->setMoment( getR()*( Mu - cross(getr(true),getR().transposed()*link->getForce())
			- cross(getrC(),(m * getLinAccC()))
			- getInertia() * getR().transposed() * getAngAcc() 
			- cross( getR().transposed() * getAngVel() , getInertia() * getR().transposed() * getAngVel())
			- link->getKr() * link->getD2Ang() * link->getIm() * getZM()
			- link->getKr() * link->getDAng() * link->getIm() * cross(getAngVel(),getZM())
			));
		break;
	case STATIC:
		link->setMoment( getR() * ( Mu - cross(getr(true),getR().transposed()*link->getForce()) )			
			- cross(getrC(), m*getLinAccC()) );
	break;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensorLinkNewtonEuler::computeMoment(iDynLink *link)
{
	switch(mode)
	{
	case DYNAMIC_CORIOLIS_GRAVITY:
	case DYNAMIC:		
	Mu = cross(getr(true),getR().transposed()*link->getForce()) 
			+ cross(getrC(),(m * getLinAccC()))  
			+ getR().transposed() * link->getMoment()
			+ getInertia() * getR().transposed() * getAngAcc() 
			+ cross( getR().transposed() * getAngVel() , getInertia() * getR().transposed() * getAngVel());
		break;
	case DYNAMIC_W_ROTOR:
	Mu = cross(getr(true),getR().transposed()*link->getForce()) 
			+ cross(getrC(),(m * getLinAccC()))  
			+ getR().transposed() * link->getMoment()
			+ getInertia() * getR().transposed() * getAngAcc() 
			+ cross( getR().transposed() * getAngVel() , getInertia() * getR().transposed() * getAngVel())
			+ link->getKr() * link->getD2Ang() * link->getIm() * getZM()
			+ link->getKr() * link->getDAng() * link->getIm() * cross(getAngVel(),getZM()) ;

		break;
	case STATIC:
		Mu = cross(getr(true),getR().transposed()*link->getForce()) 
			+ cross(getrC(), m*getLinAccC()) 
			+ getR().transposed() * link->getMoment();
	break;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector SensorLinkNewtonEuler::getForceMoment() const
{
	Vector ret(6); ret.zero();
	ret[0]=F[0]; ret[1]=F[1]; ret[2]=F[2];
	ret[3]=Mu[0]; ret[4]=Mu[1]; ret[5]=Mu[2];
	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string SensorLinkNewtonEuler::getType() const 
{
	return "no type for the basic sensor - only iCub sensors have type";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




//================================
//
//		ONE CHAIN NEWTON EULER
//
//================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
OneChainNewtonEuler::OneChainNewtonEuler(iDynChain *_c, string _info, const NewEulMode _mode, unsigned int verb)
{
	//with pointers!
	chain = _c;

	string descript;
	char buffer[100]; int j=0;		  

	mode = _mode;
	verbose = verb;
	info = _info;

	//set the manipulator chain
	nLinks = _c->getN();

	//nLinks+base+final
	neChain = new OneLinkNewtonEuler*[nLinks+2];

	//first the base frame, for the forward - insertion as first
	neChain[0] = new BaseLinkNewtonEuler(chain->getH0(),mode,verbose);
	 	
	//then the link frames (the true ones)
	for(unsigned int i=0; i<nLinks; i++)
	{	
		//OneLinkNewtonEuler * tmp 
		neChain[i+1] = new OneLinkNewtonEuler(mode,verbose,chain->refLink(i));
		//insertion in the chain
		descript.clear();
		j=sprintf(buffer,"link <%d>",i);
		descript.append(buffer);
		neChain[i+1]->setInfo(descript);
	}
	//then the final frame, for the backward - insertion as last
	neChain[nLinks+1] = new FinalLinkNewtonEuler(mode,verbose);

	//the end effector is the last (nLinks+2-1 because it's an index)
	nEndEff = nLinks+1;

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
OneChainNewtonEuler::~OneChainNewtonEuler()
{
	for(unsigned int i=0;i<=nEndEff;i++)
		delete neChain[i];
	delete [] neChain;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string OneChainNewtonEuler::toString() const
{
	string ret;
	ret = "[Chain]: " + info;
	for(unsigned int i=0;i<=nEndEff;i++)
	{
		ret.append("\n");
		ret.append(neChain[i]->toString());
	}
	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneChainNewtonEuler::getVelAccAfterForward(unsigned int i, Vector &w, Vector &dw, Vector &dwM, Vector &ddp, Vector &ddpC) const
{
	if((i>=0)&&(i<=nEndEff))
	{
		w = neChain[i]->getAngVel();
		dw = neChain[i]->getAngAcc();
		dwM = neChain[i]->getAngAccM();
		ddp = neChain[i]->getLinAcc();
		ddpC = neChain[i]->getLinAccC();
		return true;
	}
	else
	{
		if(verbose)
			fprintf(stderr,"OneChain error, impossible to retrieve vel/acc due to out of range index: %ld > %ld \n", i, nEndEff);
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneChainNewtonEuler::getWrenchAfterForward(unsigned int i, Vector &F, Vector &Mu) const
{
	if((i>=0)&&(i<=nEndEff))
	{
		F = neChain[i]->getForce();
		Mu = neChain[i]->getMoment(false);
		return true;
	}
	else
	{
		if(verbose)
			fprintf(stderr,"OneChain error, impossible to retrieve vel/acc due to out of range index: %ld > %ld \n",i,nEndEff);
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneChainNewtonEuler::getVelAccBase(Vector &w, Vector &dw,Vector &ddp) const
{
	w = neChain[0]->getAngVel();
	dw = neChain[0]->getAngAcc();
	ddp = neChain[0]->getLinAcc();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneChainNewtonEuler::getVelAccEnd(Vector &w, Vector &dw,Vector &ddp) const
{
	w = neChain[nEndEff]->getAngVel();
	dw = neChain[nEndEff]->getAngAcc();
	ddp = neChain[nEndEff]->getLinAcc();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneChainNewtonEuler::getWrenchBase(Vector &F, Vector &Mu) const
{
	F = neChain[0]->getForce();
	Mu = neChain[0]->getMoment(false);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneChainNewtonEuler::getWrenchEnd(Vector &F, Vector &Mu) const
{
	F = neChain[nEndEff]->getForce();
	Mu = neChain[nEndEff]->getMoment(false);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	//~~~~~~~~~~~~~~~~~~~~~~
	//   set methods
	//~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneChainNewtonEuler::setVerbose(unsigned int verb)					
{
	verbose=verb;
	for(unsigned int i=0;i<=nEndEff;i++)
		neChain[i]->setVerbose(verb);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneChainNewtonEuler::setMode(const NewEulMode _mode)		
{
	mode=_mode;
	for(unsigned int i=0;i<=nEndEff;i++)
		neChain[i]->setMode(_mode);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneChainNewtonEuler::setInfo(const string _info)			
{
	info=_info;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneChainNewtonEuler::initKinematicBase(const Vector &w0,const Vector &dw0,const Vector &ddp0)
{
	return neChain[0]->setAsBase(w0,dw0,ddp0);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneChainNewtonEuler::initKinematicEnd(const Vector &w0,const Vector &dw0,const Vector &ddp0)
{
	return neChain[nEndEff]->setAsBase(w0,dw0,ddp0);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneChainNewtonEuler::initWrenchEnd(const Vector &Fend,const Vector &Muend)
{
	return neChain[nEndEff]->setAsFinal(Fend,Muend);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneChainNewtonEuler::initWrenchBase(const Vector &Fend,const Vector &Muend)
{
	return neChain[0]->setAsFinal(Fend,Muend);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




	//~~~~~~~~~~~~~~~~~~~~~~
	//   get methods
	//~~~~~~~~~~~~~~~~~~~~~~

string		OneChainNewtonEuler::getInfo()		const		{return info;}
NewEulMode	OneChainNewtonEuler::getMode()		const		{return mode;}

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//   main computation methods
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneChainNewtonEuler::ForwardKinematicFromBase()
{
	for(unsigned int i=1;i<nEndEff;i++)
	{
		neChain[i]->ForwardKinematics(neChain[i-1]);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneChainNewtonEuler::ForwardKinematicFromBase(const Vector &_w, const Vector &_dw, const Vector &_ddp)
{
	//set initial values on the base frame
	neChain[0]->setAsBase(_w,_dw,_ddp);
	//finally forward
	ForwardKinematicFromBase();	
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneChainNewtonEuler::BackwardKinematicFromEnd()
{
	for(unsigned int i=nEndEff;i>=1;i--)
	{
		neChain[i-1]->BackwardKinematics(neChain[i]);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneChainNewtonEuler::BackwardKinematicFromEnd(const Vector &_w, const Vector &_dw, const Vector &_ddp)
{
	//set initial values on the base frame
	neChain[nEndEff]->setAsFinal(_w,_dw,_ddp);
	//finally forward
	BackwardKinematicFromEnd();	
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneChainNewtonEuler::BackwardWrenchFromEnd()
{
	
	for(int i=nEndEff-1; i>=0; i--)
		neChain[i]->BackwardWrench(neChain[i+1]);
	for(int i=nEndEff-1; i>0; i--)
		neChain[i]->computeTorque(neChain[i-1]);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneChainNewtonEuler::BackwardWrenchFromEnd(const Vector &F, const Vector &Mu)
{
	
	//set initial values on the end-effector frame
	neChain[nEndEff]->setAsFinal(F,Mu);
	//then backward
	BackwardWrenchFromEnd();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneChainNewtonEuler::ForwardWrenchFromBase()
{
    fprintf(stderr,"ForwardWrenchFromBase: not implemented yet \n");
	/*for(int i=nEndEff-1; i>=0; i--)
		neChain[i]->BackwardWrench(neChain[i+1]);
	for(int i=nEndEff-1; i>0; i--)
		neChain[i]->computeTorque(neChain[i-1]);   TO BE IMPLEMENTED*/
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void OneChainNewtonEuler::ForwardWrenchFromBase(const Vector &F, const Vector &Mu)
{
    fprintf(stderr,"ForwardWrenchFromBase: not implemented yet \n");
	/*for(int i=nEndEff-1; i>=0; i--)
		neChain[i]->BackwardWrench(neChain[i+1]);
	for(int i=nEndEff-1; i>0; i--)
		neChain[i]->computeTorque(neChain[i-1]);   TO BE IMPLEMENTED*/
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneChainNewtonEuler::ForwardWrenchToEnd(unsigned int lSens)
{
	if(lSens<nLinks)
	{
		// lSens is the sensor link index
		// link lSens --> neChain[lSens+1]
		// since ForwardWrench takes the previous, we start with the OneLink
		// indexed lSens+2 = lSens + baseLink + the next one
		// that's because link lSens = neChain[lSens+1] is already set before
		// with a specific sensor method
		for(unsigned int i=lSens+2; i<nEndEff; i++)
			neChain[i]->ForwardWrench(neChain[i-1]);
		return true;
	}
	else
	{
		fprintf(stderr,"OneChainNewtonEuler error, could not perform ForwardWrenchToEnd because of out of range index: %ld >= %ld \n",lSens,nLinks);
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool OneChainNewtonEuler::BackwardWrenchToBase(unsigned int lSens)
{
	if(lSens<nLinks)
	{
		// lSens is the sensor link index
		// link lSens --> neChain[lSens+1]
		// since BackwardWrench takes the next, we start with the OneLink
		// indexed lSens = lSens + baseLink - the previous one
		// that's because link lSens = neChain[lSens+1] is already set before
		// with a specific sensor method
		for(int i=lSens; i>=0; i--)
			neChain[i]->BackwardWrench(neChain[i+1]);
		// now we can compute all torques
		// we also compute the one of the sensor link, since we needed the 
		// previous link done
		for(int i=lSens+1; i>0; i--)
			neChain[i]->computeTorque(neChain[i-1]);
		return true;
	}
	else
	{
		fprintf(stderr,"OneChainNewtonEuler error, could not perform ForwardWrenchToEnd because of out of range index: %ld >= %ld \n",lSens,nLinks);
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//======================================
//
//			  iDYN INV SENSOR
//
//======================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynInvSensor::iDynInvSensor(iDynChain *_c, const string &_info, const NewEulMode _mode, unsigned int verb)
{
	chain = _c;
	info = _info + "chain with sensor: unknown";
	mode = _mode;
	verbose = verb;
	//unknown sensor
	lSens = 0;
	sens = NULL;
    //no contact
    lCont = 0;
    contact = NULL;

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynInvSensor::iDynInvSensor(iDynChain *_c, unsigned int i, const Matrix &_H, const Matrix &_HC, const double _m, const Matrix &_I, const string &_info, const NewEulMode _mode, unsigned int verb)
{
	chain = _c;
	info = _info + "chain with sensor: attached";
	mode = _mode;
	verbose = verb;
	// new sensor
	lSens = i;
	sens = new SensorLinkNewtonEuler(_H,_HC,_m,_I,_mode,verb);
    //no contact
    lCont = 0;
    contact = NULL;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynInvSensor::~iDynInvSensor()
{
	if(sens!=NULL)
		delete sens;
	sens=NULL;

    if(contact!=NULL)
        delete contact;
    contact=NULL;

	//do not delete the chain! only stop pointing at it!
	chain=NULL;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string iDynInvSensor::toString() const
{
	string ret;
	ret = "[Chain]: " + info + "\n[Sensor]: yes. ";
	char buffer[60];
	int j = sprintf(buffer,"[before link]: %d",lSens);
	ret.append(buffer);
	if(sens != NULL)
	ret = ret + sens->toString();
	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynInvSensor::setSensor(unsigned int i, const Matrix &_H, const Matrix &_HC, const double _m, const Matrix &_I)
{
	if(i<chain->getN())
	{
		lSens = i;
        if(sens==NULL)
        {
            sens = new SensorLinkNewtonEuler(_H,_HC,_m,_I,mode,verbose);
            return true;
        }
        else
            return sens->setSensor(_H,_HC,_m,_I); 
	}
	else
	{
		if(verbose)
            fprintf(stderr,"iDynInvSensor error: could not set FT Sensor inside the dynamic chain due to out of range index: %ld >= %ld \n",i,chain->getN());
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynInvSensor::computeSensorForceMoment()
{
    if(sens != NULL)
    {
	    sens->ForwardAttachToLink(chain->refLink(lSens));
	    sens->BackwardAttachToLink(chain->refLink(lSens));
    }
    else
    {
        fprintf(stderr,"iDynInvSensor error: could not make computations, the sensor is not set yet.\n");
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	iDynInvSensor::getSensorForce()	const   
{
    if(sens != NULL)
        return sens->getForce();
    else
    {
        if(verbose)
            fprintf(stderr,"iDynInvSensor error: could not get FT Sensor force, because the sensor is not set yet.\n");
        return Vector(0);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	iDynInvSensor::getSensorMoment()const   
{
    if(sens != NULL)    
        return sens->getMoment(false);
    else
    {
        if(verbose)
            fprintf(stderr,"iDynInvSensor error: could not get FT Sensor moment, because the sensor is not set yet.\n");
        return Vector(0);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynInvSensor::getTorques()      const	
{
    return chain->getTorques();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	//~~~~~~~~~~~~~~
	// set methods
	//~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynInvSensor::setMode(const NewEulMode _mode)
{
	mode = _mode;
	if(sens != NULL) sens->setMode(_mode);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynInvSensor::setVerbose(unsigned int verb)
{
	verbose = verb;
	if(sens != NULL) sens->setVerbose(verb);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynInvSensor::setInfo(const string &_info)
{
	info = _info;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynInvSensor::setSensorInfo(const string &_info)
{
	if(sens != NULL) sens->setInfo(_info);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	//~~~~~~~~~~~~~~
	// get methods
	//~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string iDynInvSensor::getInfo()					const	{return info;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynInvSensor::getH()					const	
{
    if(sens != NULL) 
        return sens->getH();
    else
    {
        if(verbose)
            fprintf(stderr,"iDynInvSensor error: could not get H of the FT sensor, because the sensor is not set yet.\n");
        return Matrix(0,0);
    }

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string iDynInvSensor::getSensorInfo()			const	
{
    if(sens != NULL) 
        return sens->getInfo();
    else
    {
        if(verbose)
            fprintf(stderr,"iDynInvSensor error: could not get info from the FT sensor, because the sensor is not set yet.\n");
        return "FT sensor is not set";
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynInvSensor::getSensorForceMoment()	const	
{
    if(sens != NULL) 
        return sens->getForceMoment(); 
    else
    {
        if(verbose)
            fprintf(stderr,"iDynInvSensor error: could not get force-moment of the FT sensor, because the sensor is not set yet.\n");
        return Vector(0);
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
unsigned int iDynInvSensor::getSensorLink()	    const	{return lSens; }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//======================================
//
//		 iCUB ARM SENSOR LINK
//
//======================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubArmSensorLink::iCubArmSensorLink(const string &_type, const NewEulMode _mode, unsigned int verb)
:SensorLinkNewtonEuler(_mode,verb)
{
	// the arm type determines the sensor properties
	type = _type;	
	//now setting inertia, mass and COM specifically for each link
	H.resize(4,4); COM.resize(4,4); I.resize(3,3);
	if(type=="left")
	{
		H.zero(); H(0,0) = 1.0; H(2,1) = 1.0; H(1,2) = -1.0; H(1,3) = 0.08428; H(3,3) = 1.0;
		COM.eye(); COM(0,3) = -1.56e-04; COM(1,3) = -9.87e-05;  COM(2,3) = 2.98e-2; 
		I.zero(); I(0,0) = 4.08e-04; I(0,1) = I(1,0) = -1.08e-6; I(0,2) = I(2,0) = -2.29e-6;
		I(1,1) = 3.80e-04; I(1,2) = I(2,1) =  3.57e-6; I(2,2) = 2.60e-4;
		m = 7.2784301e-01; 
	}
	else
	{ 
		if(!(type =="right"))
		{
			if(verbose)
				fprintf(stderr,"iCubArmSensorLink error: type is not left/right: assuming right.\n");
			type = "right";
		}

		H.zero(); H(0,0) = -1.0; H(2,1) = 1.0; H(1,2) = 1.0; H(1,3) = -0.08428; H(3,3) = 1.0;
		COM.eye(); COM(0,3) = -1.5906019e-04; COM(1,3) =   8.2873258e-05; COM(2,3) =  2.9882773e-02;
		I.zero(); I(0,0) = 4.08e-04; I(0,1) = I(1,0) = -1.08e-6; I(0,2) = I(2,0) = -2.29e-6;
		I(1,1) = 3.80e-04; I(1,2) = I(2,1) =  3.57e-6; I(2,2) = 2.60e-4;
		m = 7.29e-01; 
	
	}
	//then the sensor information
	info.clear(); info = "FT sensor " + type + " arm";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string iCubArmSensorLink::getType() const
{ 
	return type;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//======================================
//
//		 iDYN INV SENSOR ARM
//
//======================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynInvSensorArm::iDynInvSensorArm(iCubArmDyn *_c, const NewEulMode _mode, unsigned int verb)
:iDynInvSensor(_c->asChain(),_c->getType(),_mode,verb)
{
	// FT sensor is in position 5 in the kinematic chain in both arms
	lSens = 5;
	// the arm type determines the sensor properties
	if( !((_c->getType()=="left")||(_c->getType()=="right"))  )
	{
		if(verbose)
        {
            fprintf(stderr,"iDynInvSensorArm error: type is not left/right. iCub only has a left and a right arm, it is not an octopus :) \n");
			fprintf(stderr,"iDynInvSensorArm: assuming right arm. \n");
        }
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
iDynInvSensorArm::iDynInvSensorArm(iDynChain *_c, const string _type, const NewEulMode _mode, unsigned int verb)
:iDynInvSensor(_c,_type,_mode,verb)
{
	// FT sensor is in position 5 in the kinematic chain in both arms
	lSens = 5;
	// the arm type determines the sensor properties
	if( !((_type=="left")||(_type=="right"))  )
	{
		if(verbose)
        {
            fprintf(stderr,"iDynInvSensorArm error: type is not left/right. iCub only has a left and a right arm, it is not an octopus :) \n");
			fprintf(stderr,"iDynInvSensorArm: assuming right arm.\n");
        }
		// set the sensor with the default value
		sens = new iCubArmSensorLink("right",mode,verbose);
	}
	else
	{
		// set the sensor properly
		sens = new iCubArmSensorLink(_type,mode,verbose);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string iDynInvSensorArm::getType() const
{ 
	return sens->getType();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//=========================================
//
//		 iDYN INV SENSOR ARM NO TORSO
//
//=========================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynInvSensorArmNoTorso::iDynInvSensorArmNoTorso(iCubArmNoTorsoDyn *_c, const NewEulMode _mode, unsigned int verb)
:iDynInvSensor(_c->asChain(),_c->getType(),_mode,verb)
{
	// FT sensor is in position 2 in the kinematic chain in both arms
	// note: it's 5 if arm with torso, 2 if arm without torso
	lSens = 2;
	// the arm type determines the sensor properties
	if( !((_c->getType()=="left")||(_c->getType()=="right"))  )
	{
		if(verbose)
        {
            fprintf(stderr,"iDynInvSensorArm error: type is not left/right. iCub only has a left and a right arm, it is not an octopus :) \n");
			fprintf(stderr,"iDynInvSensorArm: assuming right arm. \n");
        }
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
iDynInvSensorArmNoTorso::iDynInvSensorArmNoTorso(iDynChain *_c, const string _type, const NewEulMode _mode, unsigned int verb)
:iDynInvSensor(_c,_type,_mode,verb)
{
	// FT sensor is in position 2 in the kinematic chain in both arms
	// note: it's 5 if arm with torso, 2 if arm without torso
	lSens = 2;
	// the arm type determines the sensor properties
	if( !((_type=="left")||(_type=="right"))  )
	{
		if(verbose)
        {
            fprintf(stderr,"iDynInvSensorArm error: type is not left/right. iCub only has a left and a right arm, it is not an octopus :) \n");
			fprintf(stderr,"iDynInvSensorArm: assuming right arm. \n");
        }
		// set the sensor with the default value
		sens = new iCubArmSensorLink("right",mode,verbose);
	}
	else
	{
		// set the sensor properly
		sens = new iCubArmSensorLink(_type,mode,verbose);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string iDynInvSensorArmNoTorso::getType() const
{ 
	return sens->getType();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//======================================
//
//		 iCUB LEG SENSOR LINK
//
//======================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubLegSensorLink::iCubLegSensorLink(const string _type, const NewEulMode _mode, unsigned int verb)
:SensorLinkNewtonEuler(_mode,verb)
{
	// the arm type determines the sensor properties
	type = _type;
	//now setting inertia, mass and COM specifically for the link
	H.resize(4,4); COM.resize(4,4); I.resize(3,3);
	if(type=="left")
	{
		H.zero(); H(0,1) = -1.0; H(1,0) = -1.0; H(2,2) = -1.0; H(2,3) = -0.0665; H(3,3) = 1.0;
		COM.zero();
		I.zero(); 
		
		m = 0.0;
	}
	else
	{ 
		if(!(type =="right"))
		{
			if(verbose)
				fprintf(stderr,"iCubLegSensorLink error: type is not left/right: assuming right. \n");
			type = "right";
		}

		
		H.zero(); H(0,1) = 1.0; H(1,0) = -1.0; H(2,2) = 1.0; H(2,3) = 0.0665; H(3,3) = 1.0;
		COM.zero(); 
		I.zero(); 
		
		m = 0.0;
	
	}
	//then the sensor information
	info.clear(); info = "FT sensor " + type + " leg";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string iCubLegSensorLink::getType() const
{ 
	return type;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//======================================
//
//		 iDYN INV SENSOR LEG
//
//======================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynInvSensorLeg::iDynInvSensorLeg(iCubLegDyn *_c, const NewEulMode _mode, unsigned int verb)
:iDynInvSensor(_c->asChain(),_c->getType(),_mode,verb)
{
	// FT sensor is in position 1 in the kinematic chain in both legs
	lSens = 1;
	// the leg type determines the sensor properties
	if( !((_c->getType()=="left")||(_c->getType()=="right"))  )
	{
		if(verbose)
        {
            fprintf(stderr,"iDynInvSensorLeg error: type is not left/right. iCub only has a left and a right leg, it is not a millipede :) \n");
			fprintf(stderr,"iDynInvSensorLeg: assuming right leg.\n");
        }
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
iDynInvSensorLeg::iDynInvSensorLeg(iDynChain *_c, const string _type, const NewEulMode _mode, unsigned int verb)
:iDynInvSensor(_c,_type,_mode,verb)
{
	// FT sensor is in position 2 in the kinematic chain in both legs
	lSens = 1;
	// the leg type determines the sensor properties
	if( !((_type=="left")||(_type=="right"))  )
	{
		if(verbose)
        {
            fprintf(stderr,"iDynInvSensorLeg error: type is not left/right. iCub only has a left and a right leg, it is not a millipede :) \n");
            fprintf(stderr,"iDynInvSensorLeg: assuming right leg.\n");
        }
		// set the sensor with the default value
		sens = new iCubLegSensorLink("right",mode,verbose);
	}
	else
	{
		// set the sensor properly
		sens = new iCubLegSensorLink(_type,mode,verbose);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string iDynInvSensorLeg::getType() const
{ 
	return sens->getType();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




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
        {
            fprintf(stderr,"iDynSensor error: could not set sensor measures due to wrong sized vector: %ld instead of 6 (3+3). \n",FM.length());
        }
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//   main computation methods
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynSensor::computeFromSensorNewtonEuler()
{
	//first forward all the quantities w,dw,.. in the chain
	//just in case it was not initialized before
	if(chain->NE == NULL)		
	{
		chain->prepareNewtonEuler(mode);
		chain->initNewtonEuler();
	}
	//the iDynChain independently solve the forward phase of the limb
	//setting w,dw,ddp,ddpC
	chain->computeKinematicNewtonEuler();
	sens->ForwardAttachToLink(chain->refLink(lSens));
	//the sensor does not need to retrieve w,dw,ddp,ddpC in this case 
	//then propagate forces and moments
	//from sensor to lSens
	sens->ForwardForcesMomentsToLink(chain->refLink(lSens));
	//from lSens to End
	chain->NE->ForwardWrenchToEnd(lSens);
	//from lSens to Base
	chain->NE->BackwardWrenchToBase(lSens);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynSensor::computeWrenchFromSensorNewtonEuler()
{
    // this must be done
    sens->ForwardAttachToLink(chain->refLink(lSens));
	//propagate forces and moments
	//from sensor to lSens
	sens->ForwardForcesMomentsToLink(chain->refLink(lSens));
	//from lSens to End
	chain->NE->ForwardWrenchToEnd(lSens);
	//from lSens to Base
	chain->NE->BackwardWrenchToBase(lSens);
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
        {
            fprintf(stderr,"iDynSensorArm error: type is not left/right. iCub only has a left and a right arm, it is not an octopus :) \n");
            fprintf(stderr,"iDynSensorArm: assuming right arm.\n");
        }
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
//		 iDYN SENSOR ARM NO TORSO
//
//======================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynSensorArmNoTorso::iDynSensorArmNoTorso(iCubArmNoTorsoDyn *_c, const NewEulMode _mode, unsigned int verb)
:iDynSensor(_c->asChain(),_c->getType(),_mode,verb)
{
	// FT sensor is in position 2 in the kinematic chain in both arms
	// note position 5 if with torso, 2 without torso
	lSens = 2;
	// the arm type determines the sensor properties
	if( !((_c->getType()=="left")||(_c->getType()=="right"))  )
	{
		if(verbose)
        {
            fprintf(stderr,"iDynSensorArm error: type is not left/right. iCub only has a left and a right arm, it is not an octopus :) \n");
			fprintf(stderr,"iDynSensorArm: assuming right arm.\n");
        }
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
string iDynSensorArmNoTorso::getType() const
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
	lSens = 1;
	// the leg type determines the sensor properties
	if( !((_c->getType()=="left")||(_c->getType()=="right"))  )
	{
		if(verbose)
        {
            fprintf(stderr,"iDynSensorLeg error: type is not left/right. iCub only has a left and a right leg, it is not a millipede :) \n");
			fprintf(stderr,"iDynSensorLeg: assuming right leg.\n");
        }
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


