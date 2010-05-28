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

#include <iCub/iDyn.h>
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

//================================
//
//		I DYN LINK
//
//================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynLink::iDynLink(double _A, double _D, double _Alpha, double _Offset, double _Min, double _Max)
: iKinLink( _A,  _D,  _Alpha,  _Offset,  _Min,  _Max)
{
	zero();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynLink::iDynLink(const double _m, const Matrix &_HC, const Matrix &_I, double _A, double _D, double _Alpha, double _Offset, double _Min, double _Max)
: iKinLink( _A,  _D,  _Alpha,  _Offset,  _Min,  _Max)
{
	zero();
	m = _m;
	setInertia(_I);
	setCOM(_HC);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynLink::iDynLink(const double _m, const Vector &_C, const Matrix &_I, double _A, double _D, double _Alpha, double _Offset, double _Min, double _Max)
: iKinLink( _A,  _D,  _Alpha,  _Offset,  _Min,  _Max)
{
	zero();
	m = _m;
	setInertia(_I);
	setCOM(_C);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynLink::iDynLink(const double _m, const double _rCx, const double _rCy, const double _rCz, const double Ixx, const double Ixy, const double Ixz, const double Iyy, const double Iyz, const double Izz, double _A, double _D, double _Alpha, double _Offset, double _Min, double _Max)
: iKinLink( _A,  _D,  _Alpha,  _Offset,  _Min,  _Max)
{
	zero();
	m = _m;
	setInertia(Ixx,Ixy,Ixz,Iyy,Iyz,Izz);
	setCOM(_rCx,_rCy,_rCz);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynLink::iDynLink(const iDynLink &c)
: iKinLink(c)
{
	clone(c);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynLink &iDynLink::operator=(const iDynLink &c)
{
	clone(c);
	return *this;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynLink::clone(const iDynLink &c)
{
	iKinLink::clone(c);

	m = c.getMass();
	I = c.getInertia();
	HC = c.getCOM();
	Im = c.getIm();
	Fv = c.getFv();
	Fs = c.getFs();
	kr = c.getKr();
	dq = c.getDAng();
	ddq = c.getD2Ang();
	w = c.getW();
	dw = c.getdW();
	dwM = c.getdWM();
	dp = c.getLinVel();
	dpC = c.getLinVelC();
	ddp = c.getLinAcc();
	ddpC = c.getLinAccC();
	F = c.getForce();
	Mu = c.getMoment();
	Tau = c.getTorque();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


	//~~~~~~~~~~~~~~~~~~~~~~
	//   set methods
	//~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynLink::setDynamicParameters(const double _m, const yarp::sig::Matrix &_HC, const yarp::sig::Matrix &_I, const double _kr, const double _Fv, const double _Fs, const double _Im)
{
	m = _m;
	kr = _kr;
	Fv = _Fv;
	Fs = _Fs;
	Im = _Im;
	return setInertia(_I) && setCOM(_HC);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynLink::setDynamicParameters(const double _m, const yarp::sig::Matrix &_HC, const yarp::sig::Matrix &_I)
{
	m = _m;
	return setInertia(_I) && setCOM(_HC);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynLink::setStaticParameters(const double _m,  const yarp::sig::Matrix &_HC)
{
	m = _m;
	return setCOM(_HC);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynLink::setInertia(const yarp::sig::Matrix &_I)
{
	if( (_I.rows()==3)&&(_I.cols()==3) )
	{
		I = _I;
		return true;
	}
	else
	{
		I.resize(3,3); I.zero();
		if(verbose)
			cerr<<"iDynLink: error in setting Inertia due to wrong matrix size: ("
			<<_I.rows()<<","<<_I.cols()<<") instead of (3,3). Inertia matrix now set automatically to zero."<<endl;
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynLink::setInertia(const double Ixx, const double Ixy, const double Ixz, const double Iyy, const double Iyz, const double Izz)
{
	I.resize(3,3); I.zero();
	I(0,0) = Ixx;
	I(0,1) = I(1,0) = Ixy;
	I(0,2) = I(2,0) = Ixz;
	I(1,1) = Iyy;
	I(1,2) = I(2,1) = Iyz;
	I(2,2) = Izz;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynLink::setMass(const double _m)
{
	m = _m;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynLink::setDAng(const double _dteta)
{
	dq = _dteta;
	return dq;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynLink::setD2Ang(const double _ddteta)
{
	ddq = _ddteta;
	return ddq;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynLink::setAngPosVelAcc(const double _teta,const double _dteta,const double _ddteta)
{
	setAng(_teta);
	setDAng(_dteta);
	setD2Ang(_ddteta);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynLink::setCOM(const yarp::sig::Matrix &_HC)
{
	if((_HC.rows()==4) && (_HC.cols()==4))
	{
		HC = _HC;
		return true;
	}
	else
	{
		HC.resize(4,4); HC.eye();
		if(verbose)
			cerr<<"iDynLink: error in setting COM roto-translation due to wrong matrix size: ("
			<<_HC.rows()<<","<<_HC.cols()<<") instead of (4,4). HC matrix now set automatically as eye."<<endl;
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynLink::setCOM(const yarp::sig::Vector &_rC)
{
	if(_rC.length()==3)
	{
		HC.resize(4,4); HC.eye();
		HC(0,3) = _rC(0);
		HC(1,3) = _rC(1);
		HC(2,3) = _rC(2);
		return true;
	}
	else	
	{
		if(verbose)
			cerr<<"iDynLink error, cannot set distance from COM due to wrong sized vector: "<<_rC.length()<<" instead of 3"<<endl;
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynLink::setCOM(const double _rCx, const double _rCy, const double _rCz)
{
	HC.resize(4,4); HC.eye();
	HC(0,3) = _rCx;
	HC(1,3) = _rCy;
	HC(2,3) = _rCz;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynLink::setForce(const yarp::sig::Vector &_F)
{
	if( _F.length()==3)	
	{
		F = _F;
		return true;
	}
	else
	{
		if(verbose)
			cerr<<"iDynLink error, cannot set forces due to wrong size: "<<_F.length()<<" instead of 3"<<endl;
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynLink::setMoment(const yarp::sig::Vector &_Mu)
{
	if(_Mu.length()==3)
	{
		Mu = _Mu;
		return true;
	}
	else
	{
		if(verbose)
			cerr<<"iDynLink error, cannot set moments due to wrong size: "<<_Mu.length()<<" instead of 3"<<endl;
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynLink::setForceMoment(const yarp::sig::Vector &_F, const yarp::sig::Vector &_Mu)
{
	return setForce(_F) && setMoment(_Mu);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynLink::setTorque(const double _Tau)
{
	Tau = _Tau;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynLink::zero()
{
	m = 0.0;
	dq = 0.0; ddq = 0.0;
	I.resize(3,3);	I.zero();
	w.resize(3);	w.zero();
	dw.resize(3);	dw.zero();
	dwM.resize(3);	dwM.zero();
	dp.resize(3);	dp.zero();
	dpC.resize(3);	dpC.zero();
	ddp.resize(3);	ddp.zero();
	ddpC.resize(3);	ddpC.zero();
	F.resize(3);	F.zero();
	Mu.resize(3);	Mu.zero();
	Tau = 0.0;
	Im = 0.0; kr = 0.0;	Fv = 0.0;	Fs = 0.0;	
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	 //~~~~~~~~~~~~~~~~~~~~~~
	 //   get methods
	 //~~~~~~~~~~~~~~~~~~~~~~
	     
Matrix	iDynLink::getInertia()	const	{return I;}
double	iDynLink::getMass()		const	{return m;}
double	iDynLink::getIm()		const	{return Im;}
double	iDynLink::getKr()		const	{return kr;}
double	iDynLink::getFs()		const	{return Fs;}
double	iDynLink::getFv()		const	{return Fv;}
Matrix	iDynLink::getCOM()		const	{return HC;}
double	iDynLink::getDAng()		const	{return dq;}
double	iDynLink::getD2Ang()	const	{return ddq;}
Vector	iDynLink::getW()		const	{return w;}
Vector	iDynLink::getdW()		const	{return dw;}
Vector	iDynLink::getdWM()		const	{return dwM;}
Vector  iDynLink::getLinVel()   const   {return dp;}
Vector  iDynLink::getLinVelC()	const	{return dpC;}
Vector	iDynLink::getLinAcc()	const	{return ddp;}
Vector	iDynLink::getLinAccC()	const	{return ddpC;}
Vector	iDynLink::getForce()	const	{return F;}
Vector	iDynLink::getMoment()	const	{return Mu;}
double	iDynLink::getTorque()	const	{return Tau;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix	iDynLink::getR()			{return (getH(true).submatrix(0,2,0,2));}
Matrix	iDynLink::getRC()			{return getCOM().submatrix(0,2,0,2);}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	iDynLink::getr(bool proj) 	
{
	if(proj==false)
		return getH(true).submatrix(0,2,0,3).getCol(3);
	else
		return (-1.0 * getR().transposed() * getH(true).submatrix(0,2,0,3).getCol(3));
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector	iDynLink::getrC(bool proj) 	
{
	if(proj==false)
		return getCOM().submatrix(0,2,0,3).getCol(3);
	else
		return (-1.0 * getRC().transposed() * getCOM().submatrix(0,2,0,3).getCol(3));
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//================================
//
//		I DYN CHAIN
//
//================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynChain::iDynChain()
: iKinChain()
{
	NE=NULL;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynChain::iDynChain(const Matrix &_H0)
:iKinChain(_H0)
{
	NE=NULL;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynChain::clone(const iDynChain &c)
{
	iKinChain::clone(c);
	curr_dq = c.curr_dq;
	curr_ddq = c.curr_ddq;
	NE = c.NE;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynChain::build()
{
	iKinChain::build();
	if(DOF)
	{
		curr_dq = getDAng();
		curr_ddq = getD2Ang();
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynChain::iDynChain(const iDynChain &c)
{
    clone(c);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynChain &iDynChain::operator=(const iDynChain &c)
{
    clone(c);
    return *this;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynChain::setDAng(const Vector &dq)
{
    if(!DOF)
        return Vector(0);

    curr_dq.resize(DOF);

    if(dq.length()>=(int)DOF)
	{
		for(unsigned int i=0; i<DOF; i++)
            curr_dq[i]=quickList[hash_dof[i]]->setDAng(dq[i]);
	}
    else 
		if(verbose)
        cerr<<"setVel() failed: " << DOF << " joint angles needed" <<endl;

    return curr_dq;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynChain::setD2Ang(const Vector &ddq)
{
    if(!DOF)
        return Vector(0);

    curr_ddq.resize(DOF);

    if(ddq.length()>=(int)DOF)      
	{
		for(unsigned int i=0; i<DOF; i++)
            curr_ddq[i]=quickList[hash_dof[i]]->setD2Ang(ddq[i]);
	}
    else 
		if(verbose)
        cerr<<"setVel() failed: " << DOF << " joint angles needed" <<endl;

    return curr_ddq;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynChain::getDAng()
{
    if(!DOF)
        return Vector(0);

    curr_dq.resize(DOF);

    for(unsigned int i=0; i<DOF; i++)
        curr_dq[i]=quickList[hash_dof[i]]->getDAng();

    return curr_dq;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynChain::getD2Ang()
{
	if(!DOF)
       return Vector(0);

    curr_ddq.resize(DOF);

    for(unsigned int i=0; i<DOF; i++)
        curr_ddq[i]=quickList[hash_dof[i]]->getD2Ang();

    return curr_ddq;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynChain::setDAng(const unsigned int i, double _dq)
{
    if(i<N)
        return allList[i]->setDAng(_dq);
    else 
	{	if(verbose)
		{
			cerr<<"setVel() failed due to out of range index: ";
			cerr<<i<<">="<<N<<endl;
		}
		return 0.0;
		
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynChain::setD2Ang(const unsigned int i, double _ddq)
{
    if(i<N)
         return allList[i]->setD2Ang(_ddq);
    else 
    {
		if(verbose)
		{
			cerr<<"setAcc() failed due to out of range index: ";
			cerr<<i<<">="<<N<<endl;
		}
		return 0.0;
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynChain::getDAng(const unsigned int i)
{
    if(i<N)
        return allList[i]->getDAng();
    else 
    {
		if(verbose)
		{
			cerr<<"getVel() failed due to out of range index: ";
			cerr<<i<<">="<<N<<endl;
		}
		return 0.0;
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynChain::getD2Ang(const unsigned int i)
{
    if(i<N)
        return allList[i]->getD2Ang();
    else 
    {
		if(verbose)
		{
			cerr<<"getAcc() failed due to out of range index: ";
			cerr<<i<<">="<<N<<endl;
		}
		return 0.0;
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynChain::getLinAcc(const unsigned int i) const
{
    if(i<N)
        return allList[i]->getLinAcc();
    else 
    {
		if(verbose)
		{
			cerr<<"getLinAcc() failed due to out of range index: ";
			cerr<<i<<">="<<N<<endl;
		}
		return Vector(0);
    }	
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynChain::getLinAccCOM(const unsigned int i) const
{
    if(i<N)
		return allList[i]->getLinAccC();
    else 
    {
		if(verbose)
		{
			cerr<<"getLinAccCOM() failed due to out of range index: ";
			cerr<<i<<">="<<N<<endl;
		}
		return Vector(0);
    }	
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynLink * iDynChain::refLink(const unsigned int i)
{
	if(i<N)
		return dynamic_cast<iDynLink *>(allList[i]);
	else 
	{
		if(verbose)
		{
			cerr<<"getAcc() failed due to out of range index: ";
			cerr<<i<<">="<<N<<endl;
		}
		return NULL;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynChain::getForce(const unsigned int iLink)	const		
{
	if(iLink<N)
		return allList[iLink]->getForce();
	else 
	{
		if(verbose)
		{
			cerr<<"getForce() failed due to out of range index: "
				<<iLink<<">="<<N<<endl;
		}
		return Vector(0);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynChain::getMoment(const unsigned int iLink) const	
{
	if(iLink<N)
		return allList[iLink]->getMoment();
	else 
	{
		if(verbose)
		{
			cerr<<"getMoment() failed due to out of range index: ";
			cerr<<iLink<<">="<<N<<endl;
		}
		return Vector(0);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynChain::getTorque(const unsigned int iLink) const	
{
	if(iLink<N)
		return allList[iLink]->getTorque();
	else 
	{
		if(verbose)
		{
			cerr<<"getTorque() failed due to out of range index: ";
			cerr<<iLink<<">="<<N<<endl;
		}
		return 0.0;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynChain::getMasses() const
{
	Vector ret(N); ret.zero();	
	for(unsigned int i=0;i<N;i++)
		ret[i] = allList[i]->getMass();
	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynChain::setMasses(Vector _m) 
{
	if(_m.length()==N)
	{
		for(unsigned int i=0; i<N; i++)
			allList[i]->setMass(_m[i]);
		return true;
	}
	else
	{
		if(verbose)
			cerr<<"iDynChain: setMasses() failed due to wrong vector size: "
				<<_m.length()<<" instead of "<<N<<endl;
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynChain::getMass(const unsigned int i) const
{
	if(i<N)
		return allList[i]->getMass();
	else
	{
		if(verbose)
			cerr<<"iDynChain: getMass() failed due to out of range index: "
				<<i<<">="<<N<<endl;
		return 0.0;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynChain::setMass(const unsigned int i, const double _m)
{
	if(i<N)
	{
		allList[i]->setMass(_m);
		return true;
	}
	else
	{
		if(verbose)
			cerr<<"iDynChain: setMass() failed due to out of range index: "
				<<i<<">="<<N<<endl;
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynChain::getForces() const
{
	Matrix ret(3,N);
	for(unsigned int i=0;i<N;i++)
		ret.setCol(i,allList[i]->getForce());
	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynChain::getMoments() const
{
	Matrix ret(3,N);
	for(unsigned int i=0;i<N;i++)
		ret.setCol(i,allList[i]->getMoment());
	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynChain::getTorques() const
{
	Vector ret(N);
	for(unsigned int i=0;i<N;i++)
		ret[i]= allList[i]->getTorque();
	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynChain::setDynamicParameters(const unsigned int i, const double _m, const Matrix &_HC, const Matrix &_I, const double _kr, const double _Fv, const double _Fs, const double _Im)
{
	if(i<N)
		return 	allList[i]->setDynamicParameters(_m,_HC,_I,_kr,_Fv,_Fs,_Im);	
	else
	{
		if(verbose)
		{
			cerr<<"iDynChain: setDynamicParameters() failed due to out of range index: "
				<<i<<">="<<N<<endl;
		}
		return false;
	}

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynChain::setDynamicParameters(const unsigned int i, const double _m, const Matrix &_HC, const Matrix &_I)
{
	if(i<N)
		return allList[i]->setDynamicParameters(_m,_HC,_I);
	else
	{
		if(verbose)
		{
			cerr<<"iDynChain: setDynamicParameters() failed due to out of range index: "
				<<i<<">="<<N<<endl;
		}
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynChain::setStaticParameters(const unsigned int i, const double _m, const Matrix &_HC)
{
	if(i<N)
		return allList[i]->setStaticParameters(_m,_HC);
	else
	{
		if(verbose)
		{
			cerr<<"iDynChain: setStaticParameters() failed due to out of range index: "
				<<i<<">="<<N<<endl;
		}
		return false;
	}

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynChain::prepareNewtonEuler(const NewEulMode ne_mode)
{
	string info;
	info = "[Chain] ";
	char buffer[60]; 
	int j = sprintf(buffer,"DOF=%d N=%d",DOF,N);
	info.append(buffer);

	NE = new OneChainNewtonEuler(const_cast<iDynChain *>(this),info,ne_mode,verbose);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynChain::computeNewtonEuler(const Vector &w0, const Vector &dw0, const Vector &ddp0, const Vector &Fend, const Vector &Muend )
{ 
	if((w0.length()==3)&&(dw0.length()==3)&&(ddp0.length()==3)&&(Fend.length()==3)&&(Muend.length()==3))
	{
		NE->ForwardFromBase(w0,dw0,ddp0);
		NE->BackwardFromEnd(Fend,Muend);
		return true;
	}
	else
	{
		if(verbose)
		{
			cerr<<"iDynChain error: could not compute with Newton Euler due to wrong sized initializing vectors: "
				<<" w0,dw0,ddp0,Fend,Muend have size "
				<< w0.length() <<","<< dw0.length() <<","
				<< ddp0.length() <<","<< Fend.length() <<","<< Muend.length() <<","
				<<" instead of 3,3,3,3,3"<<endl;
		}
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynChain::computeNewtonEuler()
{ 
	NE->ForwardFromBase();
	NE->BackwardFromEnd();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynChain::initNewtonEuler(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0, const yarp::sig::Vector &Fend, const yarp::sig::Vector &Muend)
{
	if((w0.length()==3)&&(dw0.length()==3)&&(ddp0.length()==3)&&(Fend.length()==3)&&(Muend.length()==3))
	{
		return NE->initNewtonEuler(w0,dw0,ddp0,Fend,Muend);
	}
	else
	{
		if(verbose)
		{
			cerr<<"iDynChain error: could not initialize Newton Euler due to wrong sized initializing vectors: "
				<<" w0,dw0,ddp0,Fend,Muend have size "
				<< w0.length() <<","<< dw0.length() <<","
				<< ddp0.length() <<","<< Fend.length() <<","<< Muend.length() <<","
				<<" instead of 3,3,3,3,3"<<endl;
		}
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynChain::setModeNewtonEuler(const NewEulMode ne_mode)
{
	NE->setMode(ne_mode);
	if(verbose)
		cerr<<"iDynChain: Newton-Euler mode set to "
			<<NE_Mode[ne_mode]<<endl;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

			//~~~~~~~~~~~~~~
			//	basic get
			//~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynChain::getForcesNewtonEuler() const
{
	Matrix ret(3,N+2);
	for(unsigned int i=0;i<N+2;i++)
		ret.setCol(i,NE->neChain[i]->getForce());
	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynChain::getMomentsNewtonEuler() const
{
	Matrix ret(3,N+2);
	for(unsigned int i=0;i<N+2;i++)
		ret.setCol(i,NE->neChain[i]->getMoment());
	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynChain::getTorquesNewtonEuler() const
{
	Vector ret(N);
	for(unsigned int i=0;i<N;i++)
		ret[i] = NE->neChain[i+1]->getTorque();
	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




//================================
//
//		I DYN LIMB
//
//================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynLimb::iDynLimb()
{
    allocate("right");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynLimb::iDynLimb(const string &_type)
{
    allocate(_type);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynLimb::iDynLimb(const iDynLimb &limb)
{
    clone(limb);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynLimb::iDynLimb(const Property &option)
{
    fromLinksProperties(option);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynLimb::fromLinksProperties(const Property &option)
{
    Property &opt=const_cast<Property&>(option);
    dispose();   
	int i,j;
	Matrix I(3,3);
	Matrix HC(4,4);

	// type: left/right
    type=opt.check("type",Value("right")).asString().c_str();
    if(type!="right" && type!="left")
    {
        cerr<<"Error: invalid handedness type specified!" <<endl;
        return false;
    }

	// H0 matrix
    if(Bottle *bH0=opt.find("H0").asList())
    {
        i=0;
        j=0;
        H0.zero();
        for(int cnt=0; (cnt<bH0->size()) && (cnt<H0.rows()*H0.cols()); cnt++)
        {    
            H0(i,j)=bH0->get(cnt).asDouble();
            if(++j>=H0.cols())
            {
                i++;
                j=0;
            }
        }
    }

	//number of links
    int numLinks=opt.check("numLinks",Value(0)).asInt();
    if(numLinks==0)
    {
        cerr<<"Error: invalid number of links specified!" <<endl;
        type="right";
        H0.eye();
        return false;
    }
    linkList.resize(numLinks,NULL);

    for(int iLink=0; iLink<numLinks; iLink++)
    {
        char link[255];
        sprintf(link,"link_%d",iLink);
		//look for link_i into the property parameters
        Bottle &bLink=opt.findGroup(link);
        if(bLink.isNull())
        {
            cerr<<"Error: " << link << " is missing!" <<endl;
            type="right";
            H0.eye();
            dispose();
            return false;
        }

		//kinematics parameters
        double A=bLink.check("A",Value(0.0)).asDouble();
        double D=bLink.check("D",Value(0.0)).asDouble();
        double alpha=CTRL_DEG2RAD*bLink.check("alpha",Value(0.0)).asDouble();
        double offset=CTRL_DEG2RAD*bLink.check("offset",Value(0.0)).asDouble();
        double min=CTRL_DEG2RAD*bLink.check("min",Value(0.0)).asDouble();
        double max=CTRL_DEG2RAD*bLink.check("max",Value(0.0)).asDouble();
		
		//dynamic parameters
		//mass
		double mass=bLink.check("mass",Value(0.0)).asDouble();
		//inertia
		if(Bottle *bI=opt.find("Inertia").asList())
		{
			i=0; j=0;
			I.zero(); 
			for(int cnt=0; (cnt<bI->size()) && (cnt<I.rows()*I.cols()); cnt++)
			{    
				I(i,j)=bI->get(cnt).asDouble();
				if(++j>=I.cols())
				{
					i++;
					j=0;
				}
			}
		}
		//HC
		if(Bottle *bHC=opt.find("H_COM").asList())
		{
			i=0; j=0;
			HC.zero(); 
			for(int cnt=0; (cnt<bHC->size()) && (cnt<HC.rows()*HC.cols()); cnt++)
			{    
				HC(i,j)=bHC->get(cnt).asDouble();
				if(++j>=HC.cols())
				{
					i++;
					j=0;
				}
			}
		}
		//create iDynLink from parameters
		linkList[iLink] = new iDynLink(mass,HC,I,A,D,alpha,offset,min,max);
		//insert the link in the list
        *this<<*linkList[iLink];

        if(bLink.check("blocked"))
            blockLink(iLink,CTRL_DEG2RAD*bLink.find("blocked").asDouble());
    }

    return configured=true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynLimb &iDynLimb::operator=(const iDynLimb &limb)
{
    dispose();	
    clone(limb);

    return *this;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynLimb::~iDynLimb()
{
    dispose();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynLimb::allocate(const string &_type)
{
    type=_type;

    if(type!="right" && type!="left")
        type="right";

    configured=true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynLimb::clone(const iDynLimb &limb)
{
    type=limb.type;
	verbose = limb.verbose;
	N = limb.N;
	DOF = limb.DOF;

	H0=limb.H0;
	curr_q = limb.curr_q;
	curr_dq = limb.curr_dq;
	curr_ddq = limb.curr_ddq;

	unsigned int n,i;
	
	//now copy all lists

    if(n=limb.linkList.size())
    {
        linkList.resize(n);
        for(i=0; i<n; i++)
        {
            linkList[i]=new iDynLink(*limb.linkList[i]);
			//also push into allList
            *this << *linkList[i];

			//push into quick list and hash lists (see iKinChain::buildChain())
			quickList.push_back(allList[i]);
            hash_dof.push_back(quickList.size()-1);
            hash.push_back(i);
        }
    }
    configured=limb.configured;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynLimb::dispose()
{
    if(unsigned int n=linkList.size())
    {
        for(unsigned int i=0; i<n; i++)
            if(linkList[i])
                delete linkList[i];

        linkList.clear();
    }
	dispose();

    configured=false;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~





//======================================
//
//			  ICUB ARM DYN
//
//======================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubArmDyn::iCubArmDyn()
{
    allocate("right");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubArmDyn::iCubArmDyn(const string &_type)
{
    allocate(_type);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubArmDyn::iCubArmDyn(const iCubArmDyn &arm)
{
    clone(arm);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCubArmDyn::allocate(const string &_type)
{
    iDynLimb::allocate(_type);

    H0.zero();
    H0(0,1)=-1;	H0(1,2)=-1;
    H0(2,0)=1;	H0(3,3)=1;

    linkList.resize(10);

	//now set the parameters properly according to the part
    if (type=="right")
    {
		//set mass, inertia and COM

		//create iDynLink from parameters calling
		//          iDynLink(mass, rC (3x1), I(6x1),            A,         D,       alfa,            offset,         min,               max);
        linkList[0]=new iDynLink(0,					0,		  0,		 0,				0,			0,			0,			0,			0,			0,	      0.032,      0.0,  M_PI/2.0,                 0.0, -22.0*CTRL_DEG2RAD,  84.0*CTRL_DEG2RAD);
        linkList[1]=new iDynLink(0,					0,		  0,		 0,				0,			0,			0,			0,			0,			0,	        0.0,      0.0,  M_PI/2.0,           -M_PI/2.0, -39.0*CTRL_DEG2RAD,  39.0*CTRL_DEG2RAD);
        linkList[2]=new iDynLink(0,					0,		  0,		 0,				0,			0,			0,			0,			0,			0,   -0.0233647,  -0.1433,  M_PI/2.0, -105.0*CTRL_DEG2RAD, -59.0*CTRL_DEG2RAD,  59.0*CTRL_DEG2RAD);
        linkList[3]=new iDynLink(0.189,		 0.005e-3,  18.7e-3,   1.19e-3,		 123.0e-6,   0.021e-6,  -0.001e-6,    24.4e-6,    4.22e-6,   113.0e-6,			0.0, -0.10774,  M_PI/2.0,           -M_PI/2.0, -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD);
        linkList[4]=new iDynLink(0.179,		-0.094e-3, -6.27e-3,  -16.6e-3,		 137.0e-6, -0.453e-06,  0.203e-06,    83.0e-6,    20.7e-6,    99.3e-6,			0.0,      0.0, -M_PI/2.0,           -M_PI/2.0,                0.0, 160.8*CTRL_DEG2RAD);
        linkList[5]=new iDynLink(0.884,		  1.79e-3, -62.9e-3, 0.064e-03,		 743.0e-6,    63.9e-6,  0.851e-06,   336.0e-6,   -3.61e-6,   735.0e-6, 			0.0, -0.15228, -M_PI/2.0, -105.0*CTRL_DEG2RAD, -37.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD);
        linkList[6]=new iDynLink(0.074,		 -13.7e-3, -3.71e-3,   1.05e-3,		  28.4e-6,  -0.502e-6,  -0.399e-6,    9.24e-6,  -0.371e-6,    29.9e-6,		  0.015,      0.0,  M_PI/2.0,                 0.0,   5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD);
        linkList[7]=new iDynLink(0.525,		-0.347e-3,  71.3e-3,  -4.76e-3,		 766.0e-6,    5.66e-6,    1.40e-6,   164.0e-6,    18.2e-6,   699.0e-6,	        0.0,  -0.1373,  M_PI/2.0,           -M_PI/2.0, -90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD);
        linkList[8]=new iDynLink(	 0,			    0,        0,         0,		 	    0,		    0,		    0,			0,			0,		    0,	        0.0,      0.0,  M_PI/2.0,            M_PI/2.0, -90.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD);
        linkList[9]=new iDynLink(0.213,		  7.73e-3, -8.05e-3,  -9.00e-3,		 154.0e-6,	  12.6e-6,   -6.08e-6,   250.0e-6,    17.6e-6,   378.0e-6,	     0.0625,    0.016,       0.0,                M_PI, -20.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD);
    }
    else
    {
        linkList[0]=new iDynLink(0,				0,		   0,		  0,				0,			0,			0,			0,			0,			0,		  0.032,     0.0,		M_PI/2.0,                 0.0,	-22.0*CTRL_DEG2RAD,  84.0*CTRL_DEG2RAD);
        linkList[1]=new iDynLink(0,				0,		   0,		  0,				0,			0,			0,			0,			0,			0,		    0.0,	 0.0,		M_PI/2.0,           -M_PI/2.0,	-39.0*CTRL_DEG2RAD,  39.0*CTRL_DEG2RAD);
        linkList[2]=new iDynLink(0,				0,		   0,		  0,				0,			0,			0,			0,			0,			0,	  0.0233647, -0.1433,	   -M_PI/2.0,  105.0*CTRL_DEG2RAD,  -59.0*CTRL_DEG2RAD,  59.0*CTRL_DEG2RAD);
        linkList[3]=new iDynLink(0.13,	-0.004e-3, 14.915e-3, -0.019e-3,		54.421e-6,   0.009e-6,     0.0e-6,   9.331e-6,  -0.017e-6,  54.862e-6,			0.0, 0.10774,	   -M_PI/2.0,            M_PI/2.0,  -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD);
        linkList[4]=new iDynLink(0.178,  0.097e-3,  -6.271e-3, 16.622e-3,		 137.2e-6,   0.466e-6,   0.365e-6,  82.927e-6, -20.524e-6,  99.274e-6,			0.0,	 0.0,		M_PI/2.0,           -M_PI/2.0,				   0.0,	160.8*CTRL_DEG2RAD);
        linkList[5]=new iDynLink(0.894, -1.769e-3, 63.302e-3, -0.084e-3,	   748.531e-6,  63.340e-6,  -0.903e-6, 338.109e-6,  -4.031e-6, 741.022e-6,			0.0, 0.15228,	   -M_PI/2.0,   75.0*CTRL_DEG2RAD,  -37.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD);
        linkList[6]=new iDynLink(0.074, 13.718e-3,  3.712e-3, -1.046e-3,		28.389e-6,  -0.515e-6,  -0.408e-6,   9.244e-6,  -0.371e-6,  29.968e-6,		 -0.015,     0.0,		M_PI/2.0,                 0.0,    0.0*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD);//5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD);
        linkList[7]=new iDynLink(0.525, 0.264e-3, -71.327e-3,  4.672e-3,	   765.393e-6,   4.337e-6,   0.239e-6, 164.578e-6,  19.381e-6, 698.060e-6,			0.0,  0.1373,		M_PI/2.0,           -M_PI/2.0,	-90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD);
        linkList[8]=new iDynLink(	 0,		   0,		   0,		  0,				0,		    0,	        0,		    0,		    0,		    0,			0.0,	 0.0,		M_PI/2.0,            M_PI/2.0,	-90.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD);
        linkList[9]=new iDynLink(0.214, 7.851e-3, -8.319e-3, 9.284e-3,		   157.143e-6,  12.780e-6,   4.823e-6, 247.995e-6, -18.188e-6, 380.535e-6,		 0.0625,  -0.016,			 0.0,                 0.0,	-20.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD);
    }
	//insert in the allList
    for(unsigned int i=0; i<linkList.size(); i++)
        *this << *linkList[i];

    //blockLink(0,0.0);
    //blockLink(1,0.0);
    //blockLink(2,0.0);

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubLegDyn::iCubLegDyn()
{
    allocate("right");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubLegDyn::iCubLegDyn(const string &_type)
{
    allocate(_type);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubLegDyn::iCubLegDyn(const iCubLegDyn &leg)
{
    clone(leg);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCubLegDyn::allocate(const string &_type)
{
    iDynLimb::allocate(_type);

    H0.zero();
    H0(0,0)=1;	    H0(1,2)=1;    
    H0(2,1)=-1;		H0(2,3)=-0.1199;
    H0(3,3)=1;

    linkList.resize(6);

	//dynamical parameters: inertia and COM are matrices, they must be initialized before
	// mass
	Vector m(6);
	// inertia and COM
	deque<Matrix> HC;  
	deque<Matrix> I; 
	Matrix HCtmp(4,4); HCtmp.eye();
	Matrix Itmp(3,3); Itmp.zero();
	for(int i=0;i<6; i++)
	{
		HC.push_back(HCtmp);
		I.push_back(Itmp);
	}

    if(type=="right")
    {
        H0(1,3)=0.0681;

		m=0;

		//create iDynLink from parameters calling
		//linkList[i] = new iDynLink(mass,HC,I,A,D,alfa,offset,min,max);

        linkList[0]=new iDynLink(m[0],	HC[0],	I[0],   0.0,     0.0,  M_PI/2.0,  M_PI/2.0,  -44.0*CTRL_DEG2RAD, 132.0*CTRL_DEG2RAD);
        linkList[1]=new iDynLink(m[1],	HC[1],	I[1],   0.0,     0.0,  M_PI/2.0,  M_PI/2.0, -119.0*CTRL_DEG2RAD,  17.0*CTRL_DEG2RAD);
        linkList[2]=new iDynLink(m[2],	HC[2],	I[2],   0.0,  0.2236, -M_PI/2.0, -M_PI/2.0,  -79.0*CTRL_DEG2RAD,  79.0*CTRL_DEG2RAD);
        linkList[3]=new iDynLink(m[3],	HC[3],	I[3],-0.213,     0.0,      M_PI,  M_PI/2.0, -125.0*CTRL_DEG2RAD,  23.0*CTRL_DEG2RAD);
        linkList[4]=new iDynLink(m[4],	HC[4],	I[4],   0.0,     0.0,  M_PI/2.0,       0.0,  -42.0*CTRL_DEG2RAD,  21.0*CTRL_DEG2RAD);
        linkList[5]=new iDynLink(m[5],	HC[5],	I[5], -0.041,     0.0,      M_PI,       0.0,  -24.0*CTRL_DEG2RAD,  24.0*CTRL_DEG2RAD);
    }
    else
    {
        H0(1,3)=-0.0681;

		m=0;

		//create iDynLink from parameters calling
		//linkList[i] = new iDynLink(mass,HC,I,A,D,alfa,offset,min,max);

        linkList[0]=new iDynLink(m[0],	HC[0],	I[0],   0.0,     0.0, -M_PI/2.0,  M_PI/2.0,  -44.0*CTRL_DEG2RAD, 132.0*CTRL_DEG2RAD);
        linkList[1]=new iDynLink(m[1],	HC[1],	I[1],   0.0,     0.0, -M_PI/2.0,  M_PI/2.0, -119.0*CTRL_DEG2RAD,  17.0*CTRL_DEG2RAD);
        linkList[2]=new iDynLink(m[2],	HC[2],	I[2],   0.0, -0.2236,  M_PI/2.0, -M_PI/2.0,  -79.0*CTRL_DEG2RAD,  79.0*CTRL_DEG2RAD);
        linkList[3]=new iDynLink(m[3],	HC[3],	I[3],-0.213,     0.0,      M_PI,  M_PI/2.0, -125.0*CTRL_DEG2RAD,  23.0*CTRL_DEG2RAD);
        linkList[4]=new iDynLink(m[4],	HC[4],	I[4],   0.0,     0.0, -M_PI/2.0,       0.0,  -42.0*CTRL_DEG2RAD,  21.0*CTRL_DEG2RAD);
        linkList[5]=new iDynLink(m[5],	HC[5],	I[5],-0.041,     0.0,       0.0,       0.0,  -24.0*CTRL_DEG2RAD,  24.0*CTRL_DEG2RAD);
    }

    for(unsigned int i=0; i<linkList.size(); i++)
        *this << *linkList[i];
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubEyeDyn::iCubEyeDyn()
{
    allocate("right");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubEyeDyn::iCubEyeDyn(const string &_type)
{
    allocate(_type);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubEyeDyn::iCubEyeDyn(const iCubEyeDyn &eye)
{
    clone(eye);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCubEyeDyn::allocate(const string &_type)
{
    iDynLimb::allocate(_type);

    H0.zero();
    H0(0,1)=-1;
    H0(1,2)=-1;
    H0(2,0)=1;
    H0(3,3)=1;

    linkList.resize(8);

	//dynamical parameters: inertia and COM are matrices, they must be initialized before
	// mass
	Vector m(8);
	// inertia and COM
	deque<Matrix> HC;  
	deque<Matrix> I; 
	Matrix HCtmp(4,4); HCtmp.eye();
	Matrix Itmp(3,3); Itmp.zero();
	for(int i=0;i<8; i++)
	{
		HC.push_back(HCtmp);
		I.push_back(Itmp);
	}

    if(type=="right")
    {
		 m=0;

		//create iDynLink from parameters calling
		//linkList[i] = new iDynLink(mass,HC,I,A,D,alfa,offset,min,max);

        linkList[0]=new iDynLink(m[0],	HC[0],	I[0],   0.032,    0.0,  M_PI/2.0,       0.0, -22.0*CTRL_DEG2RAD, 84.0*CTRL_DEG2RAD);
        linkList[1]=new iDynLink(m[1],	HC[1],	I[1],     0.0,    0.0,  M_PI/2.0, -M_PI/2.0, -39.0*CTRL_DEG2RAD, 39.0*CTRL_DEG2RAD);
        linkList[2]=new iDynLink(m[2],	HC[2],	I[2], 0.00231,-0.1933, -M_PI/2.0, -M_PI/2.0, -59.0*CTRL_DEG2RAD, 59.0*CTRL_DEG2RAD);
        linkList[3]=new iDynLink(m[3],	HC[3],	I[3],   0.033,    0.0,  M_PI/2.0,  M_PI/2.0, -40.0*CTRL_DEG2RAD, 30.0*CTRL_DEG2RAD);
        linkList[4]=new iDynLink(m[4],	HC[4],	I[4],     0.0,    0.0,  M_PI/2.0,  M_PI/2.0, -70.0*CTRL_DEG2RAD, 60.0*CTRL_DEG2RAD);
        linkList[5]=new iDynLink(m[5],	HC[5],	I[5],  -0.054, 0.0825, -M_PI/2.0, -M_PI/2.0, -55.0*CTRL_DEG2RAD, 55.0*CTRL_DEG2RAD);
        linkList[6]=new iDynLink(m[6],	HC[6],	I[6],     0.0,  0.034, -M_PI/2.0,       0.0, -35.0*CTRL_DEG2RAD, 15.0*CTRL_DEG2RAD);
        linkList[7]=new iDynLink(m[7],	HC[7],	I[7],     0.0,    0.0,  M_PI/2.0, -M_PI/2.0, -50.0*CTRL_DEG2RAD, 50.0*CTRL_DEG2RAD);
    }
    else
    {
		 m=0;

		//create iDynLink from parameters calling
		//linkList[i] = new iDynLink(mass,HC,I,A,D,alfa,offset,min,max);

        linkList[0]=new iDynLink(m[0],	HC[0],	I[0],   0.032,    0.0,  M_PI/2.0,       0.0, -22.0*CTRL_DEG2RAD, 84.0*CTRL_DEG2RAD);
        linkList[1]=new iDynLink(m[1],	HC[1],	I[1],     0.0,    0.0,  M_PI/2.0, -M_PI/2.0, -39.0*CTRL_DEG2RAD, 39.0*CTRL_DEG2RAD);
        linkList[2]=new iDynLink(m[2],	HC[2],	I[2], 0.00231,-0.1933, -M_PI/2.0, -M_PI/2.0, -59.0*CTRL_DEG2RAD, 59.0*CTRL_DEG2RAD);
        linkList[3]=new iDynLink(m[3],	HC[3],	I[3],   0.033,    0.0,  M_PI/2.0,  M_PI/2.0, -40.0*CTRL_DEG2RAD, 30.0*CTRL_DEG2RAD);
        linkList[4]=new iDynLink(m[4],	HC[4],	I[4],     0.0,    0.0,  M_PI/2.0,  M_PI/2.0, -70.0*CTRL_DEG2RAD, 60.0*CTRL_DEG2RAD);
        linkList[5]=new iDynLink(m[5],	HC[5],	I[5],  -0.054, 0.0825, -M_PI/2.0, -M_PI/2.0, -55.0*CTRL_DEG2RAD, 55.0*CTRL_DEG2RAD);
        linkList[6]=new iDynLink(m[6],	HC[6],	I[6],     0.0, -0.034, -M_PI/2.0,       0.0, -35.0*CTRL_DEG2RAD, 15.0*CTRL_DEG2RAD);
        linkList[7]=new iDynLink(m[7],	HC[7],	I[7],     0.0,    0.0,  M_PI/2.0, -M_PI/2.0, -50.0*CTRL_DEG2RAD, 50.0*CTRL_DEG2RAD);
    }

    for(unsigned int i=0; i<linkList.size(); i++)
        *this << *linkList[i];

    blockLink(0,0.0);
    blockLink(1,0.0);
    blockLink(2,0.0);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubEyeNeckRefDyn::iCubEyeNeckRefDyn()
{
    allocate("right");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubEyeNeckRefDyn::iCubEyeNeckRefDyn(const string &_type)
{
    allocate(_type);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubEyeNeckRefDyn::iCubEyeNeckRefDyn(const iCubEyeNeckRefDyn &eye)
{
    clone(eye);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCubEyeNeckRefDyn::allocate(const string &_type)
{
    rmLink(0);
    rmLink(0);
    rmLink(0);

    delete linkList[0];
    delete linkList[1];
    delete linkList[2];

    linkList.erase(linkList.begin(),linkList.begin()+2);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubInertialSensorDyn::iCubInertialSensorDyn()
{
    allocate("right");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubInertialSensorDyn::iCubInertialSensorDyn(const iCubInertialSensorDyn &sensor)
{
    clone(sensor);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCubInertialSensorDyn::allocate(const string &_type)
{
    iDynLimb::allocate(_type);

    H0.zero();
    H0(0,1)=-1;
    H0(1,2)=-1;
    H0(2,0)=1;
    H0(3,3)=1;

    linkList.resize(8);

	//dynamical parameters: inertia and COM are matrices, they must be initialized before
	// mass
	Vector m(8);
	// inertia and COM
	deque<Matrix> HC; 
	deque<Matrix> I;
	Matrix HCtmp(4,4); HCtmp.eye();
	Matrix Itmp(3,3); Itmp.zero();
	for(int i=0;i<8; i++)
	{
		HC.push_back(HCtmp);
		I.push_back(Itmp);
	}

	m=0;

	//create iDynLink from parameters calling
	//linkList[i] = new iDynLink(mass,HC,I,A,D,alfa,offset,min,max);

    // links of torso and neck
    linkList[0]=new iDynLink(m[0],	HC[0],	I[0],    0.032,       0.0,  M_PI/2.0,       0.0, -22.0*CTRL_DEG2RAD, 84.0*CTRL_DEG2RAD);
    linkList[1]=new iDynLink(m[1],	HC[1],	I[1],      0.0,       0.0,  M_PI/2.0, -M_PI/2.0, -39.0*CTRL_DEG2RAD, 39.0*CTRL_DEG2RAD);
    linkList[2]=new iDynLink(m[2],	HC[2],	I[2],  0.00231,   -0.1933, -M_PI/2.0, -M_PI/2.0, -59.0*CTRL_DEG2RAD, 59.0*CTRL_DEG2RAD);
    linkList[3]=new iDynLink(m[3],	HC[3],	I[3],    0.033,       0.0,  M_PI/2.0,  M_PI/2.0, -40.0*CTRL_DEG2RAD, 30.0*CTRL_DEG2RAD);
    linkList[4]=new iDynLink(m[4],	HC[4],	I[4],      0.0,       0.0,  M_PI/2.0,  M_PI/2.0, -70.0*CTRL_DEG2RAD, 60.0*CTRL_DEG2RAD);
    linkList[5]=new iDynLink(m[5],	HC[5],	I[5],   -0.054,    0.0825, -M_PI/2.0, -M_PI/2.0, -55.0*CTRL_DEG2RAD, 55.0*CTRL_DEG2RAD);

    // virtual links that describe T_nls (see http://eris.liralab.it/wiki/ICubInertiaSensorKinematics)
    linkList[6]=new iDynLink(m[6],	HC[6],	I[6], 0.013250,  0.008538,  0.785721,       0.0,              0.0,             0.0);
    linkList[7]=new iDynLink(m[7],	HC[7],	I[7], 0.013250, -0.026861,  0.785075,       0.0,              0.0,             0.0);

    for(unsigned int i=0; i<linkList.size(); i++)
        *this << *linkList[i];

    // block virtual links
    blockLink(6,0.0);
    blockLink(7,0.0);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iFakeDyn::iFakeDyn()
{
    allocate("right");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iFakeDyn::iFakeDyn(const string &_type)
{
    allocate(_type);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iFakeDyn::iFakeDyn(const iFakeDyn &arm)
{
    clone(arm);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iFakeDyn::allocate(const string &_type)
{
    iDynLimb::allocate(_type);

    H0.eye();
    linkList.resize(1);

	//dynamical parameters: inertia and COM are matrices, they must be initialized before
	// mass
	Vector m(1);
	// inertia and COM
	deque<Matrix> HC;
	deque<Matrix> I;
	Matrix HCtmp(4,4); HCtmp.eye();
	Matrix Itmp(3,3); Itmp.zero();
		HC.push_back(HCtmp);
		I.push_back(Itmp);

	//now set the parameters properly according to the part
    if(type=="right")
    {
		//set mass, inertia and COM
		 m=1.0; 

		//create iDynLink from parameters calling
		//linkList[i] = new iDynLink(mass,HC,I,A,D,alfa,offset,min,max);
		
        linkList[0]=new iDynLink(m[0],	HC[0],	I[0],		1.0,      1.0,  0.0,   0.0, -180.0*CTRL_DEG2RAD,  180.0*CTRL_DEG2RAD);
        }
    else
    {
		m=0;
        linkList[0]=new iDynLink(m[0],	HC[0],	I[0],     1.0,      1.0,  0.0,   0.0, -180.0*CTRL_DEG2RAD,  180.0*CTRL_DEG2RAD);
    }

	//insert in the allList
    for(unsigned int i=0; i<linkList.size(); i++)
        *this << *linkList[i];

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iFakeDyn2GdL::iFakeDyn2GdL()
{
    allocate("right");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iFakeDyn2GdL::iFakeDyn2GdL(const string &_type)
{
    allocate(_type);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iFakeDyn2GdL::iFakeDyn2GdL(const iFakeDyn2GdL &arm)
{
    clone(arm);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iFakeDyn2GdL::allocate(const string &_type)
{
    iDynLimb::allocate(_type);

    H0.eye();
    linkList.resize(2);

	//dynamical parameters: inertia and COM are matrices, they must be initialized before
	// mass
	Vector m(2);
	// inertia and COM
	deque<Matrix> HC;
	deque<Matrix> I;
	Matrix HCtmp(4,4); Matrix Itmp(3,3);
	// Parameter link 1
	m(0) = 1;
	HCtmp.eye(); HC.push_back(HCtmp);
	Itmp.zero(); I.push_back(Itmp);
	// Parameter link 2
	m(1) = 1;
	HCtmp.eye(); HC.push_back(HCtmp);
	Itmp.zero(); I.push_back(Itmp);
		
		

	//now set the parameters properly according to the part
    if(type=="right")
    {
		//create iDynLink from parameters calling
		//linkList[i] = new iDynLink(mass,HC,I,A,D,alfa,offset,min,max);
		
        linkList[0]=new iDynLink(m[0],	HC[0],	I[0],		1.0,      0.0,  0.0,   0.0, -180.0*CTRL_DEG2RAD,  180.0*CTRL_DEG2RAD);
        linkList[1]=new iDynLink(m[0],	HC[1],	I[1],		1.0,      0.0,  0.0,   0.0, -180.0*CTRL_DEG2RAD,  180.0*CTRL_DEG2RAD);
        }
    else
    {

        linkList[0]=new iDynLink(m[0],	HC[0],	I[0],     1.0,      0.0,  0.0,   0.0, -180.0*CTRL_DEG2RAD,  180.0*CTRL_DEG2RAD);
        linkList[1]=new iDynLink(m[1],	HC[1],	I[1],     1.0,      0.0,  0.0,   0.0, -180.0*CTRL_DEG2RAD,  180.0*CTRL_DEG2RAD);
    }

	//insert in the allList
    for(unsigned int i=0; i<linkList.size(); i++)
        *this << *linkList[i];

}