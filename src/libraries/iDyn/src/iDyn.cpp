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

#include <iCub/iDyn/iDyn.h>
#include <iCub/ctrl/math.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace iCub::iDyn;

//================================
//
//		I DYN HELPERS
//
//================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCub::iDyn::notImplemented(const unsigned int verbose)
{
    if(verbose) fprintf(stderr,"iDyn: error: not implemented \n");
}

void iCub::iDyn::notImplemented(const unsigned int verbose, const string &msg)
{
    if(verbose) fprintf(stderr,"iDyn: error: not implemented \n %s \n",msg.c_str());
}

void iCub::iDyn::workInProgress(const unsigned int verbose, const std::string &msg)
{
    if(verbose) fprintf(stderr,"iDyn: warning: this method/class is still under development. Please do not use it! \n %s \n",msg.c_str());
}

bool iCub::iDyn::asWrench(Vector &w, const Vector &f, const Vector &m)
{
    w.resize(6); w.zero();
    if((f.length()==3)||(m.length()==3))
    {
        w[0]=f[0]; w[1]=f[1]; w[2]=f[2];
        w[3]=m[0]; w[4]=m[1]; w[5]=m[2];
        return true;
    }
    else
    {
        fprintf(stderr,"iDyn: error in calling asWrench(), wrong sized vectors: (%d,%d) instead of (3,3). return wrench set automatically as zero.\n",f.length(),m.length());	
        return false;
    }
}

Vector iCub::iDyn::asWrench(const Vector &f, const Vector &m)
{
    Vector w(6); w.zero();
    if((f.length()==3)||(m.length()==3))
    {
        w[0]=f[0]; w[1]=f[1]; w[2]=f[2];
        w[3]=m[0]; w[4]=m[1]; w[5]=m[2];
    }
    else
    {
        fprintf(stderr,"iDyn: error in calling asWrench(), wrong sized vectors: (%d,%d) instead of (3,3). return wrench set automatically as zero.\n",f.length(),m.length());	
    }
    return w;
}

bool iCub::iDyn::asForceMoment(const Vector &w, Vector &f, Vector &m)
{
    f.resize(3); f.zero();
    m.resize(3); m.zero();

    if(w.length()==6)
    {
        f[0]=w[0]; f[1]=w[1]; f[2]=w[2];
        m[0]=w[3]; m[1]=w[4]; m[2]=w[5];  
        return true;
    }
    else
    {
        fprintf(stderr,"iDyn: error in calling asForceMoment(), wrong sized vector: (%d) instead of (6). return force/moment set automatically as zero.\n",w.length());	
        return false;
    }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


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
		if(verbose)	fprintf(stderr,"iDynLink: error in setting Inertia due to wrong matrix size: (%d,%d) instead of (3,3). Inertia matrix now set automatically to zero. \n",_I.rows(),_I.cols());
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
			fprintf(stderr,"iDynLink: error in setting COM roto-translation due to wrong matrix size: (%d,%d) instead of (4,4). HC matrix now set automatically as eye.\n",_HC.rows(),_HC.cols());
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
			fprintf(stderr,"iDynLink error, cannot set distance from COM due to wrong sized vector: %d instead of 3 \n",_rC.length());
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
            fprintf(stderr,"iDynLink error: cannot set forces due to wrong size: %d instead of 3.\n",_F.length());
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
			fprintf(stderr,"iDynLink error, cannot set moments due to wrong size: %d instead of 3. \n",_Mu.length());
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
	setIterMode(KINFWD_WREBWD);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynChain::iDynChain(const Matrix &_H0)
:iKinChain(_H0)
{
	NE=NULL;
	setIterMode(KINFWD_WREBWD);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynChain::clone(const iDynChain &c)
{
	iKinChain::clone(c);
	curr_dq = c.curr_dq;
	curr_ddq = c.curr_ddq;
	iterateMode_kinematics = c.iterateMode_kinematics;
	iterateMode_wrench = c.iterateMode_wrench;
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
void iDynChain::dispose()
{
	iKinChain::dispose();
	if(NE)
	{
		delete NE;
		NE=NULL;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynChain::~iDynChain()
{
	dispose();
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
            fprintf(stderr,"iDynChain error: setVel() failed: %d joint angles needed \n",DOF);

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
        fprintf(stderr,"iDynChain error: setVel() failed: %d joint angles needed \n",DOF);

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
	{	
        if(verbose) fprintf(stderr,"iDynChain error: setVel() failed due to out of range index: %d >= %d \n",i,N);
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
		if(verbose)	fprintf(stderr,"iDynChain error: setD2Ang() failed due to out of range index: %d >= %d \n",i,N);
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
		if(verbose)	fprintf(stderr,"iDynChain error: getDAng() failed due to out of range index: %d >= %d \n",i,N);
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
		if(verbose)	fprintf(stderr,"iDynChain error: getD2Ang() failed due to out of range index: %d >= %d \n",i,N);
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
		if(verbose)	fprintf(stderr,"iDynChain error: getLinAcc() failed due to out of range index: %d >= %d \n",i,N);
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
		if(verbose)	fprintf(stderr,"iDynChain error: getLinAccCOM() failed due to out of range index: %d >= %d \n",i,N);
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
		if(verbose)	fprintf(stderr,"iDynChain error: refLink() failed due to out of range index: %d >= %d \n",i,N);
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
		if(verbose) fprintf(stderr,"iDynChain error: getForce() failed due to out of range index: %d >= %d \n",iLink,N);
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
		if(verbose) fprintf(stderr,"iDynChain error: getMoment() failed due to out of range index: %d >= %d \n",iLink,N);
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
		if(verbose) fprintf(stderr,"iDynChain error: getTorque() failed due to out of range index: %d >= %d \n",iLink,N);
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
		if(verbose) fprintf(stderr,"iDynChain error: setMasses() failed due to wrong vector size: %d instead of %d",_m.length(),N);
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
		if(verbose) fprintf(stderr,"iDynChain error: getMass() failed due to out of range index: %d >= %d \n",i,N);
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
		if(verbose)	fprintf(stderr,"iDynChain error: setMass() failed due to out of range index: %d >= %d \n",i,N);
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
		if(verbose)	fprintf(stderr,"iDynChain error: setDynamicParameters() failed due to out of range index: %d >= %d \n",i,N);
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
		if(verbose)	fprintf(stderr,"iDynChain error: setDynamicParameters() failed due to out of range index: %d >= %d \n",i,N);
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
		if(verbose)	fprintf(stderr,"iDynChain error: setStaticParameters() failed due to out of range index: %d >= %d \n",i,N);
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynChain::prepareNewtonEuler(const NewEulMode NewEulMode_s)
{
	string info;
	info = "[Chain] ";
	char buffer[60]; 
	int j = sprintf(buffer,"DOF=%d N=%d",DOF,N);
	info.append(buffer);

	if( NE == NULL)
		NE = new OneChainNewtonEuler(const_cast<iDynChain *>(this),info,NewEulMode_s,verbose);
	else
	{
		delete NE;
		NE = new OneChainNewtonEuler(const_cast<iDynChain *>(this),info,NewEulMode_s,verbose);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynChain::computeNewtonEuler(const Vector &w0, const Vector &dw0, const Vector &ddp0, const Vector &F0, const Vector &Mu0 )
{ 
	if( NE == NULL)
	{
		if(verbose)
        {
            fprintf(stderr,"iDynChain error: trying to call computeNewtonEuler() without having prepared Newton-Euler method in the class. \n");
			fprintf(stderr,"iDynChain: prepareNewtonEuler() called autonomously in the default mode. \n");
        }
		prepareNewtonEuler();
	}

	if((w0.length()==3)&&(dw0.length()==3)&&(ddp0.length()==3)&&(F0.length()==3)&&(Mu0.length()==3))
	{
		if(iterateMode_kinematics == FORWARD)	
			NE->ForwardKinematicFromBase(w0,dw0,ddp0);
		else 
			NE->BackwardKinematicFromEnd(w0,dw0,ddp0);

		if(iterateMode_wrench == BACKWARD)	
			NE->BackwardWrenchFromEnd(F0,Mu0);
		else 
			NE->ForwardWrenchFromBase(F0,Mu0);
		return true;
	}
	else
	{
		if(verbose)
		{
            fprintf(stderr,"iDynChain error: could not compute with Newton Euler due to wrong sized initializing vectors: \n");
            fprintf(stderr," w0,dw0,ddp0,Fend,Muend have size %d,%d,%d,%d,%d instead of 3,3,3,3,3 \n",w0.length(),dw0.length(),ddp0.length(),F0.length(),Mu0.length());
		}
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynChain::computeNewtonEuler()
{ 
	if( NE == NULL)
	{
		if(verbose)
        {
            fprintf(stderr,"iDynChain error: trying to call computeNewtonEuler() without having prepared Newton-Euler method in the class. \n");
			fprintf(stderr,"iDynChain: prepareNewtonEuler() called autonomously in the default mode.  \n");
			fprintf(stderr,"iDynChain: initNewtonEuler() called autonomously with default values.  \n");
        }
		prepareNewtonEuler();
		initNewtonEuler();
	}

	if(iterateMode_kinematics == FORWARD)	
		NE->ForwardKinematicFromBase();
	else 
		NE->BackwardKinematicFromEnd();

	if(iterateMode_wrench == BACKWARD)	
		NE->BackwardWrenchFromEnd();
	else 
		NE->ForwardWrenchFromBase();

	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynChain::computeKinematicNewtonEuler()
{
	if(iterateMode_kinematics == FORWARD)	
		NE->ForwardKinematicFromBase();
	else 
		NE->BackwardKinematicFromEnd();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynChain::computeWrenchNewtonEuler()
{
	if(iterateMode_wrench == BACKWARD)	
		NE->BackwardWrenchFromEnd();
	else 
		NE->ForwardWrenchFromBase();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynChain::getKinematicNewtonEuler(Vector &w, Vector &dw, Vector &ddp)
{
	if( NE == NULL)
	{
		if(verbose)
        {
            fprintf(stderr,"iDynChain error: trying to call getKinematicNewtonEuler() without having prepared Newton-Euler method in the class. \n");
			fprintf(stderr,"iDynChain: prepareNewtonEuler() called autonomously in the default mode. \n");
			fprintf(stderr,"iDynChain: initNewtonEuler() called autonomously with default values. \n");
        }
		prepareNewtonEuler();
		initNewtonEuler();
	}
	
	w.resize(3); dw.resize(3); ddp.resize(3); w=dw=ddp=0.0;
	if(iterateMode_kinematics == FORWARD)	
	{
		//get kinematics from the end-effector
		NE->getVelAccEnd(w,dw,ddp);
	}
	else 
	{
		//get kinematics from the base
		NE->getVelAccBase(w,dw,ddp);
	}

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynChain::getFrameKinematic(unsigned int i, Vector &w, Vector &dw, Vector &ddp)
{
    if( NE == NULL)
	{
		if(verbose)
        {
            fprintf(stderr,"iDynChain error: trying to call getFrameKinematic() without having prepared Newton-Euler method in the class. \n");
			fprintf(stderr,"iDynChain: prepareNewtonEuler() called autonomously in the default mode. \n");
			fprintf(stderr,"iDynChain: initNewtonEuler() called autonomously with default values. \n");
        }
		prepareNewtonEuler();
		initNewtonEuler();
	}

	Vector dwM(3);dwM=0.0;
	Vector ddpC(3);ddpC=0.0;
	NE->getVelAccAfterForward(i,w,dw,dwM,ddp,ddpC);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynChain::getFrameWrench(unsigned int i, Vector &F, Vector &Mu)
{
    if( NE == NULL)
	{
		if(verbose)
        {
			fprintf(stderr,"iDynChain error: trying to call getFrameWrench() without having prepared Newton-Euler method in the class. \n");
			fprintf(stderr,"iDynChain: prepareNewtonEuler() called autonomously in the default mode. \n");
			fprintf(stderr,"iDynChain: initNewtonEuler() called autonomously with default values.  \n");
        }
		prepareNewtonEuler();
		initNewtonEuler();
	}

	NE->getWrenchAfterForward(i,F,Mu);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynChain::getWrenchNewtonEuler(Vector &F, Vector &Mu) 
{
	if( NE == NULL)
	{
		if(verbose)
        {
            fprintf(stderr,"iDynChain error: trying to call getWrenchNewtonEuler() without having prepared Newton-Euler method in the class. \n");
			fprintf(stderr,"iDynChain: prepareNewtonEuler() called autonomously in the default mode. \n");
			fprintf(stderr,"iDynChain: initNewtonEuler() called autonomously with default values. \n");
        }
		prepareNewtonEuler();
		initNewtonEuler();
	}
	F.resize(3); Mu.resize(3); F=Mu=0.0;
	if(iterateMode_wrench == BACKWARD)			
	{
		//get wrench from the base
		NE->getWrenchBase(F,Mu);
	}
	else
	{
		//get wrench from the end-effector
		NE->getWrenchEnd(F,Mu);
	}

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynChain::initNewtonEuler()
{
	Vector w0(3); w0.zero();
	Vector dw0(3); dw0.zero();
	Vector ddp0(3); ddp0.zero();
	Vector Fend(3); Fend.zero();
	Vector Muend(3); Muend.zero();

	return initNewtonEuler(w0,dw0,ddp0,Fend,Muend);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynChain::initNewtonEuler(const Vector &w0, const Vector &dw0, const Vector &ddp0, const Vector &Fend, const Vector &Muend)
{
	if( NE == NULL)
	{
		if(verbose)
        {
            fprintf(stderr,"iDynChain error: trying to call initNewtonEuler() without having prepared Newton-Euler method in the class. \n");
			fprintf(stderr,"iDynChain: prepareNewtonEuler() called autonomously in the default mode.  \n");
        }
		prepareNewtonEuler();
	}

	if((w0.length()==3)&&(dw0.length()==3)&&(ddp0.length()==3)&&(Fend.length()==3)&&(Muend.length()==3))
	{
		bool ret=true;

		if(iterateMode_kinematics == FORWARD)	
			ret = ret && NE->initKinematicBase(w0,dw0,ddp0);
		else 
			ret = ret && NE->initKinematicEnd(w0,dw0,ddp0);

		if(iterateMode_wrench == BACKWARD)	
			ret = ret && NE->initWrenchEnd(Fend,Muend);
		else 
			ret = ret && NE->initWrenchBase(Fend,Muend);

		return ret;
	}
	else
	{
		if(verbose)
		{
			fprintf(stderr,"iDynChain error: could not initialize Newton Euler due to wrong sized initializing vectors: ");
			fprintf(stderr," w0,dw0,ddp0,Fend,Muend have size %d,%d,%d,%d,%d instead of 3,3,3,3,3 \n",w0.length(),dw0.length(),ddp0.length(),Fend.length(),Muend.length());
		}
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynChain::initKinematicNewtonEuler(const Vector &w0, const Vector &dw0, const Vector &ddp0)
{
	if(NE == NULL)
	{
		if(verbose)
        {
            fprintf(stderr,"iDynChain error: trying to call initKinematicNewtonEuler() without having prepared Newton-Euler method in the class. \n");
			fprintf(stderr,"iDynChain: prepareNewtonEuler() called autonomously in the default mode.  \n");
        }
		prepareNewtonEuler();
	}

	if((w0.length()==3)&&(dw0.length()==3)&&(ddp0.length()==3))
	{
		if(iterateMode_kinematics == FORWARD)	
			return NE->initKinematicBase(w0,dw0,ddp0);
		else 
			return NE->initKinematicEnd(w0,dw0,ddp0);
	}
	else
	{
		if(verbose)
		{
			fprintf(stderr,"iDynChain error: could not initialize Newton Euler due to wrong sized initializing vectors: ");
			fprintf(stderr," w0,dw0,ddp0 have size %d,%d,%d instead of 3,3,3 \n",w0.length(),dw0.length(),ddp0.length());
		}
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynChain::initWrenchNewtonEuler(const Vector &Fend, const Vector &Muend)
{
	if( NE == NULL)
	{
		if(verbose)
        {
            fprintf(stderr,"iDynChain error: trying to call initWrenchNewtonEuler() without having prepared Newton-Euler method in the class. \n");
			fprintf(stderr,"iDynChain: prepareNewtonEuler() called autonomously in the default mode. \n");
        }
		prepareNewtonEuler();
	}

	if((Fend.length()==3)&&(Muend.length()==3))
	{
		if(iterateMode_wrench == BACKWARD)	
			return NE->initWrenchEnd(Fend,Muend);
		else 
			return NE->initWrenchBase(Fend,Muend);
	}
	else
	{
		if(verbose)
		{
			fprintf(stderr,"iDynChain error: could not initialize Newton Euler due to wrong sized initializing vectors: ");
			fprintf(stderr," Fend,Muend have size %d,%d instead of 3,3 \n",Fend.length(),Muend.length());
		}
		return false;
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynChain::setModeNewtonEuler(const NewEulMode mode)
{
	if( NE == NULL)
	{
		if(verbose)
        {
            fprintf(stderr,"iDynChain error: trying to call setModeNewtonEuler() without having prepared Newton-Euler method in the class. \n");
			fprintf(stderr,"iDynChain: prepareNewtonEuler() called autonomously in the default mode. \n");
        }
		prepareNewtonEuler();
	}

	NE->setMode(mode);
	if(verbose) fprintf(stderr,"iDynChain: Newton-Euler mode set to %s \n",NewEulMode_s[mode].c_str());
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

			//~~~~~~~~~~~~~~
			//	plus get
			//~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynChain::getForcesNewtonEuler() const
{
	Matrix ret(3,N+2); ret.zero();
	if( NE != NULL)
	{
		for(unsigned int i=0;i<N+2;i++)
			ret.setCol(i,NE->neChain[i]->getForce());
	}
	else
	{
		if(verbose)	fprintf(stderr,"iDynChain error: trying to call getForcesNewtonEuler() without having prepared Newton-Euler. \n");
	}

	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynChain::getMomentsNewtonEuler() const
{
	Matrix ret(3,N+2); ret.zero();
	if( NE != NULL)
	{
		for(unsigned int i=0;i<N+2;i++)
			ret.setCol(i,NE->neChain[i]->getMoment());
	}
	else
	{
		if(verbose)	fprintf(stderr,"iDynChain error: trying to call getMomentsNewtonEuler() without having prepared Newton-Euler. \n");
	}
	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynChain::getTorquesNewtonEuler() const
{
	Vector ret(N); ret.zero();
	if( NE != NULL)
	{
		for(unsigned int i=0;i<N;i++)
			ret[i] = NE->neChain[i+1]->getTorque();
	}
	else
	{
		if(verbose)	fprintf(stderr,"iDynChain error: trying to call getTorquesNewtonEuler() without having prepared Newton-Euler. \n");
	}
	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iDynChain::getForceMomentEndEff() const
{
	Vector ret(6); ret.zero();
	Vector f = allList[N-1]->getForce();
	Vector m = allList[N-1]->getMoment();
	ret[0]=f[0]; ret[1]=f[1]; ret[2]=f[2];
	ret[3]=m[0]; ret[4]=m[1]; ret[5]=m[2];
	return ret;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynChain::setIterModeKinematic(const ChainIterationMode _iterateMode_kinematics) 
{
	iterateMode_kinematics = _iterateMode_kinematics;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynChain::setIterModeWrench(const ChainIterationMode _iterateMode_wrench) 
{
	iterateMode_wrench = _iterateMode_wrench;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ChainIterationMode iDynChain::getIterModeKinematic() const
{
	return iterateMode_kinematics;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ChainIterationMode iDynChain::getIterModeWrench() const
{
	return iterateMode_wrench;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynChain::setIterMode(const ChainComputationMode mode)
{
	switch(mode)
	{
	case KINFWD_WREBWD: setIterModeKinematic(FORWARD); setIterModeWrench(BACKWARD);
		break;
	case KINFWD_WREFWD: setIterModeKinematic(FORWARD); setIterModeWrench(FORWARD);
		break;
	case KINBWD_WREBWD: setIterModeKinematic(BACKWARD); setIterModeWrench(BACKWARD);
		break;
	case KINBWD_WREFWD: setIterModeKinematic(BACKWARD); setIterModeWrench(FORWARD);
		break;
	default:
		if(verbose)	fprintf(stderr,"iDynChain error: in setIterMode() could not set iteration mode due to unexisting mode \n"); 
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	//-----------
	//  jacobian
	//-----------

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynChain::computeGeoJacobian(const unsigned int iLinkN, const Matrix &Pn)
{
	if(DOF==0)
    {
		if(verbose) fprintf(stderr,"iDynChain: computeGeoJacobian() failed since DOF==0 \n");
        return Matrix(0,0);
    }
    if(iLinkN>=N)
    {
		if(verbose) fprintf(stderr,"iDynChain: computeGeoJacobian() failed due to out of range indexes: from 0 to %d >= %d \n", iLinkN, N);
        return Matrix(0,0);
    }

	// the jacobian size is linkN+1: eg, index=2, Njoints=0,1,2=3
    Matrix J(6, iLinkN+1 );J.zero();
    Matrix Z;
    Vector w;

    deque<Matrix> intH;
    intH.push_back(H0);
    for (unsigned int i=0; i<iLinkN; i++)
        intH.push_back(intH[i]*allList[i]->getH(true));

    for (unsigned int i=0; i<iLinkN; i++)
    {
		unsigned int j=hash[i];
        Z=intH[j];
        w=cross(Z,2,Pn-Z,3,verbose);
        J(0,j)=w[0];
        J(1,j)=w[1];
        J(2,j)=w[2];
        J(3,j)=Z(0,2);
        J(4,j)=Z(1,2);
        J(5,j)=Z(2,2);
    }
    return J;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynChain::computeGeoJacobian(const unsigned int iLinkN, const Matrix &Pn, const Matrix &_H0)
{
	if(DOF==0)
    {
		if(verbose)fprintf(stderr,"iDynChain: computeGeoJacobian() failed since DOF==0 \n");
        return Matrix(0,0);
    }
    if(iLinkN>=N)
    {
		if(verbose) fprintf(stderr,"iDynChain: computeGeoJacobian() failed due to out of range indexes: from 0 to %d >= %d \n",iLinkN,N);
        return Matrix(0,0);
    }

	// the jacobian size is linkN+1: eg, index=2, Njoints=0,1,2=3
    Matrix J(6, iLinkN+1 );J.zero();
    Matrix Z;
    Vector w;

    deque<Matrix> intH;
    intH.push_back(_H0);
    for (unsigned int i=0; i<iLinkN; i++)
        intH.push_back(intH[i]*allList[i]->getH(true));

    for (unsigned int i=0; i<iLinkN; i++)
    {
		unsigned int j=hash[i];
        Z=intH[j];
        w=cross(Z,2,Pn-Z,3,verbose);
        J(0,j)=w[0];
        J(1,j)=w[1];
        J(2,j)=w[2];
        J(3,j)=Z(0,2);
        J(4,j)=Z(1,2);
        J(5,j)=Z(2,2);
    }
    return J;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynChain::computeGeoJacobian(const Matrix &Pn)
{
	if (DOF==0)
    {
		if(verbose) fprintf(stderr,"iDynChain: computeGeoJacobian() failed since DOF==0 \n");
        return Matrix(0,0);
    }

	//the jacobian is the same of iKin, but Pn is an input
	Matrix J(6,DOF); J.zero();
    Matrix Z;
    Vector w;

    deque<Matrix> intH;
    intH.push_back(H0);
    for(unsigned int i=0; i<N; i++)
        intH.push_back(intH[i]*allList[i]->getH(true));

    for(unsigned int i=0; i<DOF; i++)
    {
        unsigned int j=hash[i];
        Z=intH[j];
        w=cross(Z,2,Pn-Z,3,verbose);
        J(0,i)=w[0];
        J(1,i)=w[1];
        J(2,i)=w[2];
        J(3,i)=Z(0,2);
        J(4,i)=Z(1,2);
        J(5,i)=Z(2,2);
    }
    return J;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynChain::computeGeoJacobian(const Matrix &Pn, const Matrix &_H0)
{
	if (DOF==0)
    {
		if(verbose) fprintf(stderr,"iDynChain: computeGeoJacobian() failed since DOF==0 \n");
        return Matrix(0,0);
    }

	//the jacobian is the same of iKin, but Pn is an input
	Matrix J(6,DOF); J.zero();
    Matrix Z;
    Vector w;

    deque<Matrix> intH;
    intH.push_back(_H0);
    for(unsigned int i=0; i<N; i++)
        intH.push_back(intH[i]*allList[i]->getH(true));

    for(unsigned int i=0; i<DOF; i++)
    {
        unsigned int j=hash[i];
        Z=intH[j];
        w=cross(Z,2,Pn-Z,3,verbose);
        J(0,i)=w[0];
        J(1,i)=w[1];
        J(2,i)=w[2];
        J(3,i)=Z(0,2);
        J(4,i)=Z(1,2);
        J(5,i)=Z(2,2);
    }
    return J;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //---------------
	// jacobians COM
	//---------------

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynChain::computeCOMJacobian(const unsigned int iLink)
{
    if (iLink>=N)
    {
        if(verbose) fprintf(stderr,"computeCOMJacobian() failed due to out of range index: %d >= %d \n",iLink, N);
        return Matrix(0,0);
    }

    Matrix J(6,iLink+1);
    Matrix Pn,Z;
    Vector w;

    deque<Matrix> intH;
    intH.push_back(H0);

    for (unsigned int j=0; j<=iLink; j++)
        intH.push_back(intH[j]*allList[j]->getH(true));

    Pn=intH[iLink+1]*allList[iLink]->getCOM();

    for (unsigned int j=0; j<=iLink; j++)
    {
        Z=intH[j];
        w=cross(Z,2,Pn-Z,3,verbose);

        J(0,j)=w[0];
        J(1,j)=w[1];
        J(2,j)=w[2];
        J(3,j)=Z(0,2);
        J(4,j)=Z(1,2);
        J(5,j)=Z(2,2);
    }

    return J;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynChain::computeCOMJacobian(const unsigned int iLink, const Matrix &Pn)
{
    if (iLink>=N)
    {
        if(verbose) fprintf(stderr,"computeCOMJacobian() failed due to out of range index: %d >= %d \n", iLink,N);
        return Matrix(0,0);
    }

    Matrix J(6,iLink+1);
    Matrix Z;
    Vector w;

    deque<Matrix> intH;
    intH.push_back(H0);

    for (unsigned int j=0; j<=iLink; j++)
        intH.push_back(intH[j]*allList[j]->getH(true));

    for (unsigned int j=0; j<=iLink; j++)
    {
        Z=intH[j];
        w=cross(Z,2,Pn-Z,3,verbose);

        J(0,j)=w[0];
        J(1,j)=w[1];
        J(2,j)=w[2];
        J(3,j)=Z(0,2);
        J(4,j)=Z(1,2);
        J(5,j)=Z(2,2);
    }

    return J;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynChain::computeCOMJacobian(const unsigned int iLink, const Matrix &Pn, const Matrix &_H0)
{
    if (iLink>=N)
    {
        if(verbose) fprintf(stderr,"computeCOMJacobian() failed due to out of range index: %d >= %d \n", iLink,N);
        return Matrix(0,0);
    }

    Matrix J(6,iLink+1);
    Matrix Z;
    Vector w;

    deque<Matrix> intH;
    intH.push_back(_H0);

    for (unsigned int j=0; j<=iLink; j++)
        intH.push_back(intH[j]*allList[j]->getH(true));

    for (unsigned int j=0; j<=iLink; j++)
    {
        Z=intH[j];
        w=cross(Z,2,Pn-Z,3,verbose);

        J(0,j)=w[0];
        J(1,j)=w[1];
        J(2,j)=w[2];
        J(3,j)=Z(0,2);
        J(4,j)=Z(1,2);
        J(5,j)=Z(2,2);
    }

    return J;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynChain::getCOM(unsigned int iLink)
{
    if (iLink>=N)
    {
        if(verbose) fprintf(stderr,"iDynChain: error, getCOM() failed due to out of range index: %d >= %d \n", iLink,N);
        return Matrix(0,0);
    }
    return allList[iLink]->getCOM();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iDynChain::getHCOM(unsigned int iLink)
{
    if (iLink>=N)
    {
        if(verbose) fprintf(stderr,"iDynChain: error, getHCOM() failed due to out of range index: %d >= %d \n", iLink,N);
        return Matrix(0,0);
    }
    return getH(iLink,true) * allList[iLink]->getCOM();
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
void iDynLimb::pushLink(iDynLink *pl)
{
    linkList.push_back(pl);
    pushLink(*pl);
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
        fprintf(stderr,"Error: invalid handedness type specified! \n");
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
        fprintf(stderr,"Error: invalid number of links (0) specified! \n");
        type="right";
        H0.eye();
        return false;
    }

    for(int iLink=0; iLink<numLinks; iLink++)
    {
        char link[255];
        sprintf(link,"link_%d",iLink);
		//look for link_i into the property parameters
        Bottle &bLink=opt.findGroup(link);
        if(bLink.isNull())
        {
            fprintf(stderr,"Error: link %d is missing! \n",iLink);
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
		pushLink(new iDynLink(mass,HC,I,A,D,alpha,offset,min,max));

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

	//now copy all lists
    for(unsigned int i=0; i<limb.getN(); i++)
    {
        pushLink(new iDynLink(*(limb.linkList[i])));

        //push into quick list and hash lists (see iKinChain::buildChain())
        quickList.push_back(allList[i]);
        hash_dof.push_back(quickList.size()-1);
        hash.push_back(i);
    }
    configured=limb.configured;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynLimb::dispose()
{
    for(unsigned int i=0; i<linkList.size(); i++)
        if(linkList[i])
            delete linkList[i];
	
    linkList.clear();
	iDynChain::dispose();

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
	setIterMode(KINFWD_WREBWD);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubArmDyn::iCubArmDyn(const string &_type, const ChainComputationMode _mode)
{
    allocate(_type);
	setIterMode(_mode);
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

	Matrix H0(4,4);
    H0.zero();
    H0(0,1)=-1.0;
    H0(1,2)=-1.0;
    H0(2,0)=1.0;
    H0(3,3)=1.0;
	setH0(H0);

    if (getType()=="right")
    {
		//          iDynLink(mass, rC (3x1), I(6x1),            A,         D,       alfa,            offset,         min,               max);
        pushLink(new iDynLink(0,					0,		  0,		 0,				0,			0,			0,			0,			0,			0,	      0.032,      0.0,  M_PI/2.0,                 0.0, -22.0*CTRL_DEG2RAD,  84.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0,					0,		  0,		 0,				0,			0,			0,			0,			0,			0,	        0.0,      0.0,  M_PI/2.0,           -M_PI/2.0, -39.0*CTRL_DEG2RAD,  39.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0,					0,		  0,		 0,				0,			0,			0,			0,			0,			0,   -0.0233647,  -0.1433,  M_PI/2.0, -105.0*CTRL_DEG2RAD, -59.0*CTRL_DEG2RAD,  59.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.189,		 0.005e-3,  18.7e-3,   1.19e-3,		 123.0e-6,   0.021e-6,  -0.001e-6,    24.4e-6,    4.22e-6,   113.0e-6,			0.0, -0.10774,  M_PI/2.0,           -M_PI/2.0, -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.179,		-0.094e-3, -6.27e-3,  -16.6e-3,		 137.0e-6, -0.453e-06,  0.203e-06,    83.0e-6,    20.7e-6,    99.3e-6,			0.0,      0.0, -M_PI/2.0,           -M_PI/2.0,                0.0, 160.8*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.884,		  1.79e-3, -62.9e-3, 0.064e-03,		 743.0e-6,    63.9e-6,  0.851e-06,   336.0e-6,   -3.61e-6,   735.0e-6, 		 -0.015, -0.15228, -M_PI/2.0, -105.0*CTRL_DEG2RAD, -37.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.074,		 -13.7e-3, -3.71e-3,   1.05e-3,		  28.4e-6,  -0.502e-6,  -0.399e-6,    9.24e-6,  -0.371e-6,    29.9e-6,		  0.015,      0.0,  M_PI/2.0,                 0.0,   5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.525,		-0.347e-3,  71.3e-3,  -4.76e-3,		 766.0e-6,    5.66e-6,    1.40e-6,   164.0e-6,    18.2e-6,   699.0e-6,	        0.0,  -0.1373,  M_PI/2.0,           -M_PI/2.0, -90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(	 0,			    0,        0,         0,		 	    0,		    0,		    0,			0,			0,		    0,	        0.0,      0.0,  M_PI/2.0,            M_PI/2.0, -90.0*CTRL_DEG2RAD,       0.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.213,		  7.73e-3, -8.05e-3,  -9.00e-3,		 154.0e-6,	  12.6e-6,   -6.08e-6,   250.0e-6,    17.6e-6,   378.0e-6,	     0.0625,    0.016,       0.0,                M_PI, -20.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD));
    }
    else
    {
        pushLink(new iDynLink(0,				0,		   0,		  0,				0,			0,			0,			0,			0,			0,		  0.032,     0.0,		M_PI/2.0,                 0.0,	-22.0*CTRL_DEG2RAD,  84.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0,				0,		   0,		  0,				0,			0,			0,			0,			0,			0,		    0.0,	 0.0,		M_PI/2.0,           -M_PI/2.0,	-39.0*CTRL_DEG2RAD,  39.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0,				0,		   0,		  0,				0,			0,			0,			0,			0,			0,	  0.0233647, -0.1433,	   -M_PI/2.0,  105.0*CTRL_DEG2RAD,  -59.0*CTRL_DEG2RAD,  59.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.13,	-0.004e-3, 14.915e-3, -0.019e-3,		54.421e-6,   0.009e-6,     0.0e-6,   9.331e-6,  -0.017e-6,  54.862e-6,			0.0, 0.10774,	   -M_PI/2.0,            M_PI/2.0,  -95.5*CTRL_DEG2RAD,       5.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.178,  0.097e-3,  -6.271e-3, 16.622e-3,		 137.2e-6,   0.466e-6,   0.365e-6,  82.927e-6, -20.524e-6,  99.274e-6,			0.0,	 0.0,		M_PI/2.0,           -M_PI/2.0,				   0.0,	    160.8*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.894, -1.769e-3, 63.302e-3, -0.084e-3,	   748.531e-6,  63.340e-6,  -0.903e-6, 338.109e-6,  -4.031e-6, 741.022e-6,		  0.015, 0.15228,	   -M_PI/2.0,   75.0*CTRL_DEG2RAD,  -37.0*CTRL_DEG2RAD,      90.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.074, 13.718e-3,  3.712e-3, -1.046e-3,		28.389e-6,  -0.515e-6,  -0.408e-6,   9.244e-6,  -0.371e-6,  29.968e-6,		 -0.015,     0.0,		M_PI/2.0,                 0.0,    5.5*CTRL_DEG2RAD,     106.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.525, 0.264e-3, -71.327e-3,  4.672e-3,	   765.393e-6,   4.337e-6,   0.239e-6, 164.578e-6,  19.381e-6, 698.060e-6,			0.0,  0.1373,		M_PI/2.0,           -M_PI/2.0,	-90.0*CTRL_DEG2RAD,      90.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(	 0,		   0,		   0,		  0,				0,		    0,	        0,		    0,		    0,		    0,			0.0,	 0.0,		M_PI/2.0,            M_PI/2.0,	-90.0*CTRL_DEG2RAD,       0.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.214, 7.851e-3, -8.319e-3, 9.284e-3,		   157.143e-6,  12.780e-6,   4.823e-6, 247.995e-6, -18.188e-6, 380.535e-6,		 0.0625,  -0.016,			 0.0,                 0.0,	-20.0*CTRL_DEG2RAD,      40.0*CTRL_DEG2RAD));
    }

    blockLink(0,0.0);
    blockLink(1,0.0);
    blockLink(2,0.0);

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCubArmDyn::alignJointsBounds(const deque<IControlLimits*> &lim)
{
    if (lim.size()<2)
        return false;

    IControlLimits &limTorso=*lim[0];
    IControlLimits &limArm  =*lim[1];

    unsigned int iTorso;
    unsigned int iArm;
    double min, max;

    for (iTorso=0; iTorso<3; iTorso++)
    {   
        if (!limTorso.getLimits(iTorso,&min,&max))
            return false;

        (*this)[2-iTorso].setMin(CTRL_DEG2RAD*min);
        (*this)[2-iTorso].setMax(CTRL_DEG2RAD*max);
    }

    for (iArm=0; iArm<getN()-iTorso; iArm++)
    {   
        if (!limArm.getLimits(iArm,&min,&max))
            return false;

        (*this)[iTorso+iArm].setMin(CTRL_DEG2RAD*min);
        (*this)[iTorso+iArm].setMax(CTRL_DEG2RAD*max);
    }

    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




//======================================
//
//	      ICUB ARM NO TORSO DYN
//
//======================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubArmNoTorsoDyn::iCubArmNoTorsoDyn()
{
    allocate("right");
	setIterMode(KINFWD_WREBWD);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubArmNoTorsoDyn::iCubArmNoTorsoDyn(const string &_type, const ChainComputationMode _mode)
{
    allocate(_type);
	setIterMode(_mode);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubArmNoTorsoDyn::iCubArmNoTorsoDyn(const iCubArmNoTorsoDyn &arm)
{
    clone(arm);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCubArmNoTorsoDyn::allocate(const string &_type)
{
    iDynLimb::allocate(_type);

	Matrix H0(4,4);
	H0.eye();
	setH0(H0);

    if (getType()=="right")
    {
        //note: the D value in joint 0 is different from the corresponding one in iCubArmDyn (and iKin::iCubArm)
        // in ikin: -0.10774   here: 0.0
        // see the RBT matrix of the right arm connected to the UpperTorso or UpperBody nodes
		pushLink(new iDynLink(0.189,		 0.005e-3,  18.7e-3,   1.19e-3,		 123.0e-6,   0.021e-6,  -0.001e-6,    24.4e-6,    4.22e-6,   113.0e-6,			0.0,     -0.0,  M_PI/2.0,           -M_PI/2.0, -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD)); 
		pushLink(new iDynLink(0.179,		-0.094e-3, -6.27e-3,  -16.6e-3,		 137.0e-6, -0.453e-06,  0.203e-06,    83.0e-6,    20.7e-6,    99.3e-6,			0.0,      0.0, -M_PI/2.0,           -M_PI/2.0,                0.0, 160.8*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.884,		  1.79e-3, -62.9e-3, 0.064e-03,		 743.0e-6,    63.9e-6,  0.851e-06,   336.0e-6,   -3.61e-6,   735.0e-6, 		 -0.015, -0.15228, -M_PI/2.0, -105.0*CTRL_DEG2RAD, -37.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.074,		 -13.7e-3, -3.71e-3,   1.05e-3,		  28.4e-6,  -0.502e-6,  -0.399e-6,    9.24e-6,  -0.371e-6,    29.9e-6,		  0.015,      0.0,  M_PI/2.0,                 0.0,   5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.525,		-0.347e-3,  71.3e-3,  -4.76e-3,		 766.0e-6,    5.66e-6,    1.40e-6,   164.0e-6,    18.2e-6,   699.0e-6,	        0.0,  -0.1373,  M_PI/2.0,           -M_PI/2.0, -90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(	 0,			    0,        0,         0,		 	    0,		    0,		    0,			0,			0,		    0,	        0.0,      0.0,  M_PI/2.0,            M_PI/2.0, -90.0*CTRL_DEG2RAD,       0.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.213,		  7.73e-3, -8.05e-3,  -9.00e-3,		 154.0e-6,	  12.6e-6,   -6.08e-6,   250.0e-6,    17.6e-6,   378.0e-6,	     0.0625,    0.016,       0.0,                M_PI, -20.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD));
 	}
    else
    {
        //note: the D value in joint 0 is different from the corresponding one in iCubArmDyn (and iKin::iCubArm)
        // in ikin:  +0.10774   here: 0.0
        // see the RBT matrix of the right arm connected to the UpperTorso or UpperBody nodes
        pushLink(new iDynLink(0.13,	-0.004e-3, 14.915e-3, -0.019e-3,		54.421e-6,   0.009e-6,     0.0e-6,   9.331e-6,  -0.017e-6,  54.862e-6,			0.0,	 0.0,	   -M_PI/2.0,            M_PI/2.0,  -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.178,  0.097e-3,  -6.271e-3, 16.622e-3,		 137.2e-6,   0.466e-6,   0.365e-6,  82.927e-6, -20.524e-6,  99.274e-6,			0.0,	 0.0,		M_PI/2.0,           -M_PI/2.0,				   0.0,	160.8*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.894, -1.769e-3, 63.302e-3, -0.084e-3,	   748.531e-6,  63.340e-6,  -0.903e-6, 338.109e-6,  -4.031e-6, 741.022e-6,		  0.015, 0.15228,	   -M_PI/2.0,   75.0*CTRL_DEG2RAD,  -37.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.074, 13.718e-3,  3.712e-3, -1.046e-3,		28.389e-6,  -0.515e-6,  -0.408e-6,   9.244e-6,  -0.371e-6,  29.968e-6,		 -0.015,     0.0,		M_PI/2.0,                 0.0,    5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.525, 0.264e-3, -71.327e-3,  4.672e-3,	   765.393e-6,   4.337e-6,   0.239e-6, 164.578e-6,  19.381e-6, 698.060e-6,			0.0,  0.1373,		M_PI/2.0,           -M_PI/2.0,	-90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(	 0,		   0,		   0,		  0,				0,		    0,	        0,		    0,		    0,		    0,			0.0,	 0.0,		M_PI/2.0,            M_PI/2.0,	-90.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.214, 7.851e-3, -8.319e-3, 9.284e-3,		   157.143e-6,  12.780e-6,   4.823e-6, 247.995e-6, -18.188e-6, 380.535e-6,		 0.0625,  -0.016,			 0.0,                 0.0,	-20.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD));
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCubArmNoTorsoDyn::alignJointsBounds(const deque<IControlLimits*> &lim)
{
    if (lim.size()<1)
        return false;

    IControlLimits &limArm  =*lim[0];

    unsigned int iArm;
    double min, max;

    for (iArm=0; iArm<getN(); iArm++)
    {   
        if (!limArm.getLimits(iArm,&min,&max))
            return false;

        (*this)[iArm].setMin(CTRL_DEG2RAD*min);
        (*this)[iArm].setMax(CTRL_DEG2RAD*max);
    }

    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//======================================
//
//			  ICUB TORSO DYN
//
//======================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubTorsoDyn::iCubTorsoDyn()
{
    allocate("lower");
	setIterMode(KINBWD_WREBWD);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubTorsoDyn::iCubTorsoDyn(const string &_type, const ChainComputationMode _mode)
{
	allocate(_type); 
	setIterMode(_mode);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubTorsoDyn::iCubTorsoDyn(const iCubTorsoDyn &torso)
{
    clone(torso);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCubTorsoDyn::allocate(const string &_type)
{
	type = _type;

    // note: H0 here is an identity, whereas in iCubArm and iCubArmDyn is different (0 -1 00; 00 -1 0; 1 000; 0001 by row)
    // the reason is that the torso limb is cursed, and its H0 is different if we consider it a standalone limb attached
    // to upper body or lower torso nodes. 
    // check the RBT matrix linking torso to each node, and set the H0 properly before performing computations involving it
    // like the Jacobians..
	Matrix H0(4,4);
	H0.eye();
	setH0(H0);

    //           iDynLink(   mass,					 rC (3x1),					  I(6x1),													                 A,          D,         alfa,     offset,                   min,               max);
//  These parameters are still under debug!
//#define TORSO_NO_WEIGHT
#ifdef TORSO_NO_WEIGHT
	pushLink(new iDynLink(0.00e-3,	  0.000e-3,  0.000e-3,  0.000e-3,		0.000e-6,   0.000e-6,   0.000e-6,   0.000e-6,   0.000e-6,   0.000e-6,	    32.0e-3,         0,	    M_PI/2.0,		 0.0,	-100.0*CTRL_DEG2RAD, 100.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.00e-3,	  0.000e-3,  0.000e-3,  0.000e-3, 	    0.000e-6,   0.000e-6,   0.000e-6,   0.000e-6,   0.000e-6,   0.000e-6,   	      0,   -5.5e-3,		M_PI/2.0,  -M_PI/2.0,	-100.0*CTRL_DEG2RAD, 100.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.00e-3,	  0.000e-3,  0.000e-3,  0.000e-3,	    0.000e-6,   0.000e-6,   0.000e-6,   0.000e-6,   0.000e-6,   0.000e-6,		2.31e-3, -193.3e-3,	   -M_PI/2.0,  -M_PI/2.0,	-100.0*CTRL_DEG2RAD, 100.0*CTRL_DEG2RAD));
#else
//  This version of the parameters has been taken from the CAD...
//  pushLink(new iDynLink(7.79e-1,	  3.120e-2,	 6.300e-4,  -9.758e-7,		4.544e-4,  -4.263e-5,  -3.889e-8,   1.141e-3,   0.000e-0,   1.236e-3,		32.0e-3,         0,     M_PI/2.0,        0.0,	 -22.0*CTRL_DEG2RAD,  84.0*CTRL_DEG2RAD));
//	pushLink(new iDynLink(5.77e-1,	  4.265e-2,	+4.296e-5,	-1.360e-3,		5.308e-4,  -1.923e-6,   5.095e-5,   2.031e-3,  -3.849e-7,   1.803e-3,		      0,   -5.5e-3,     M_PI/2.0,  -M_PI/2.0,	 -39.0*CTRL_DEG2RAD,  39.0*CTRL_DEG2RAD));
//	pushLink(new iDynLink(4.81e+0,	 -8.102e-5,  7.905e-3,	-1.183e-1,		7.472e-2,  -3.600e-6,  -4.705e-5,   8.145e-2,   4.567e-3,   1.306e-2,		2.31e-3, -193.3e-3,	   -M_PI/2.0,  -M_PI/2.0,	 -59.0*CTRL_DEG2RAD,  59.0*CTRL_DEG2RAD));
//  ...but they have still to be verified. In the meanwhile use this for debug:
    pushLink(new iDynLink(0,	      3.120e-2,	 6.300e-4,  -9.758e-7,		4.544e-4,  -4.263e-5,  -3.889e-8,   1.141e-3,   0.000e-0,   1.236e-3,		32.0e-3,         0,     M_PI/2.0,        0.0,	-100.0*CTRL_DEG2RAD, 100.0*CTRL_DEG2RAD));
	pushLink(new iDynLink(0,	      4.265e-2,	+4.296e-5,	-1.360e-3,		5.308e-4,  -1.923e-6,   5.095e-5,   2.031e-3,  -3.849e-7,   1.803e-3,		      0,   -5.5e-3,     M_PI/2.0,  -M_PI/2.0,	-100.0*CTRL_DEG2RAD, 100.0*CTRL_DEG2RAD));
	pushLink(new iDynLink(4.81e+0,	 -8.102e-5, -1.183e-1,   7.905e-3,		7.472e-2,  -3.600e-6,  -4.705e-5,   8.145e-2,   4.567e-3,   1.306e-2,		2.31e-3, -193.3e-3,	   -M_PI/2.0,  -M_PI/2.0,	-100.0*CTRL_DEG2RAD, 100.0*CTRL_DEG2RAD));
#endif  

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCubTorsoDyn::alignJointsBounds(const deque<IControlLimits*> &lim)
{
    if (lim.size()<1)
        return false;

    IControlLimits &limTorso=*lim[0];

    unsigned int iTorso;
    double min, max;

    for (iTorso=0; iTorso<3; iTorso++)
    {   
        if (!limTorso.getLimits(iTorso,&min,&max))
            return false;

        (*this)[2-iTorso].setMin(CTRL_DEG2RAD*min);
        (*this)[2-iTorso].setMax(CTRL_DEG2RAD*max);
    }

    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~






//======================================
//
//			  ICUB LEG DYN
//
//======================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

iCubLegDyn::iCubLegDyn()
{
    allocate("right");
	setIterMode(KINFWD_WREBWD);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubLegDyn::iCubLegDyn(const string &_type,const ChainComputationMode _mode)
{
    allocate(_type);
	setIterMode(_mode);
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

	Matrix H0(4,4);
    H0.eye();		
	setH0(H0);
	
#ifdef  LEGS_NO_WEIGHT
    if(getType()=="right")
    {
    	//create iDynLink from parameters calling
		//pushLink(new iDynLink(mass,HC,I,A,D,alfa,offset,min,max));
		//                    m            rcx        rcy        rcZ               I1          I2          I3          I4          I5          I6          A            D          alpha     offset                   
        pushLink(new iDynLink(0,         -0.0782,  -0.00637,  -0.00093,				0,			0,			0,			0,			0,			0,		   0.0,			0.0,	M_PI/2.0,	 M_PI/2.0,  -44.0*CTRL_DEG2RAD,  132.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0,         0.00296,  -0.00072,   0.03045,				0,			0,			0,			0,			0,			0,		   0.0,			0.0,	M_PI/2.0,	 M_PI/2.0,  -17.0*CTRL_DEG2RAD,  119.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0,         0.00144,   0.06417,   0.00039,				0,			0,			0,			0,			0,			0,		   0.0,		 0.2236,   -M_PI/2.0,	-M_PI/2.0,  -79.0*CTRL_DEG2RAD,   79.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0,          0.1059,   0.00182,  -0.00211,			    0,		    0,		    0,		    0,		    0,		    0,		-0.213,			0.0,		M_PI,    M_PI/2.0, -125.0*CTRL_DEG2RAD,   23.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0,         -0.0054,   0.00163,   -0.0172,				0,			0,			0,			0,			0,			0,		   0.0,			0.0,	M_PI/2.0,		  0.0,  -42.0*CTRL_DEG2RAD,   21.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0,			   0,		  0,  		 0,				0,			0,			0,			0,			0,			0,		-0.041,			0.0,		M_PI,		  0.0,  -24.0*CTRL_DEG2RAD,   24.0*CTRL_DEG2RAD));

	}
    else
    {
        pushLink(new iDynLink(0,         -0.0782, -0.00637,    0.00093,	   471.076e-6,   2.059e-6,   1.451e-6,  346.478e-6,   1.545e-6,510.315e-6,		   0.0,			0.0,   -M_PI/2.0,	M_PI/2.0,  -44.0*CTRL_DEG2RAD,     132.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0,         0.00296, -0.00072,   -0.03045,	  738.0487e-6,  -0.074e-6,  -0.062e-6,  561.583e-6,  10.835e-6,294.119e-6,		   0.0,			0.0,   -M_PI/2.0,   M_PI/2.0,  -17.0*CTRL_DEG2RAD,     119.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0,         0.00144,  0.06417,	  -0.00039,	  7591.073e-6, -67.260e-6,   2.267e-6,1423.0245e-6,36.37582e-6,7553.84e-6,		   0.0,		-0.2236,	M_PI/2.0,  -M_PI/2.0,  -79.0*CTRL_DEG2RAD,      79.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0,          0.1059,  0.00182,	   0.00211,    998.950e-6,-185.699e-6, -63.147e-6, 4450.537e-6,   0.786e-6,4207.65e-6,		-0.213,			0.0,		M_PI,   M_PI/2.0, -125.0*CTRL_DEG2RAD,      23.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0,         -0.0054,  0.00163,     0.0172,    633.230e-6,	-7.081e-6,  41.421e-6,  687.760e-6,  20.817e-6, 313.89e-6,		   0.0,			0.0,   -M_PI/2.0,        0.0,  -42.0*CTRL_DEG2RAD,      21.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0,			   0,		 0,	 	     0,		  	    0,			0,	        0, 			 0,			 0,		    0,		-0.041,			0.0,         0.0,        0.0,  -24.0*CTRL_DEG2RAD,      24.0*CTRL_DEG2RAD));
    }
#else
    if(getType()=="right")
    {
		//create iDynLink from parameters calling
		//pushLink(new iDynLink(mass,HC,I,A,D,alfa,offset,min,max));

        pushLink(new iDynLink(0.754,      -0.0782,  -0.00637, -0.00093,				0,			0,			0,			0,			0,			0,		   0.0,			0.0,	M_PI/2.0,	 M_PI/2.0,  -44.0*CTRL_DEG2RAD,     132.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.526,      0.00296,  -0.00072,  0.03045,				0,			0,			0,			0,			0,			0,		   0.0,			0.0,	M_PI/2.0,	 M_PI/2.0,  -17.0*CTRL_DEG2RAD,     119.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(2.175,      0.00144,   0.06417,  0.00039,				0,			0,			0,			0,			0,			0,		   0.0,		 0.2236,   -M_PI/2.0,	-M_PI/2.0,  -79.0*CTRL_DEG2RAD,      79.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(1.264,       0.1059,   0.00182, -0.00211,			  0.0,		  0.0,		  0.0,		  0.0,		  0.0,		  0.0,		-0.213,			0.0,		M_PI,    M_PI/2.0, -125.0*CTRL_DEG2RAD,      23.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.746,      -0.0054,   0.00163,  -0.0172,				0,			0,			0,			0,			0,			0,		   0.0,			0.0,	M_PI/2.0,		  0.0,  -42.0*CTRL_DEG2RAD,      21.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0,		  	    0,		   0,	 	 0,				0,			0,			0,			0,			0,			0,		-0.041,			0.0,		M_PI,		  0.0,  -24.0*CTRL_DEG2RAD,      24.0*CTRL_DEG2RAD));

	}
    else
    {
        pushLink(new iDynLink(0.754,      -0.0782, -0.00637,   0.00093,     471.076e-6,   2.059e-6,  1.451e-6,  346.478e-6,   1.545e-6, 510.315e-6,		   0.0,			0.0,   -M_PI/2.0,	M_PI/2.0,  -44.0*CTRL_DEG2RAD,     132.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.526,      0.00296, -0.00072,  -0.03045,    738.0487e-6,	 -0.074e-6, -0.062e-6,  561.583e-6,  10.835e-6, 294.119e-6,		   0.0,			0.0,   -M_PI/2.0,   M_PI/2.0,  -17.0*CTRL_DEG2RAD,     119.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(2.175,      0.00144,  0.06417,  -0.00039,    7591.073e-6, -67.260e-6,  2.267e-6,1423.0245e-6,36.37258e-6,7553.849e-6,		   0.0,		-0.2236,	M_PI/2.0,  -M_PI/2.0,  -79.0*CTRL_DEG2RAD,      79.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(1.264,       0.1059,  0.00182,   0.00211,     998.950e-6,-185.699e-6,-63.147e-6, 4450.537e-6,   0.786e-6,4207.657e-6,		-0.213,			0.0,		M_PI,   M_PI/2.0, -125.0*CTRL_DEG2RAD,      23.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.746,      -0.0054,  0.00163,    0.0172,     633.230e-6,	 -7.081e-6, 41.421e-6,  687.760e-6,	 20.817e-6, 313.897e-6,		   0.0,			0.0,   -M_PI/2.0,        0.0,  -42.0*CTRL_DEG2RAD,      21.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0,	 	 	    0,		  0,   	     0, 	    	 0,			 0,	 	    0,		     0,  		 0, 		 0,		-0.041,			0.0,         0.0,        0.0,  -24.0*CTRL_DEG2RAD,      24.0*CTRL_DEG2RAD));
    }
#endif

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCubLegDyn::alignJointsBounds(const deque<IControlLimits*> &lim)
{
    if (lim.size()<1)
        return false;

    IControlLimits &limLeg=*lim[0];

    unsigned int iLeg;
    double min, max;

    for (iLeg=0; iLeg<getN(); iLeg++)
    {   
        if (!limLeg.getLimits(iLeg,&min,&max))
            return false;

        (*this)[iLeg].setMin(CTRL_DEG2RAD*min);
        (*this)[iLeg].setMax(CTRL_DEG2RAD*max);
    }

    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

////////////////////////////////////////
//		ICUB INERTIAL SENSOR DYN            
////////////////////////////////////////

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubNeckInertialDyn::iCubNeckInertialDyn(const ChainComputationMode _mode)
{
    allocate("right");
	setIterMode(_mode);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubNeckInertialDyn::iCubNeckInertialDyn(const iCubNeckInertialDyn &sensor)
{
    clone(sensor);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCubNeckInertialDyn::allocate(const string &_type)
{
    iDynLimb::allocate(_type);

	Matrix H0(4,4);
    H0.eye();
	setH0(H0);

    pushLink(new iDynLink(0.27017604,	  -30.535917e-3,  2.5211768e-3, -0.23571261e-3, 	     100.46346e-6,   -0.17765781e-6,       0.44914333e-6, 45.425961e-6, -0.12682862e-6, 1.0145446e+02,		  0.033,       0.0,  M_PI/2.0,  M_PI/2.0, -40.0*CTRL_DEG2RAD,     30.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0.27230552,				0.0,  4.3752947e-3,   5.4544215e-3, 	     142.82339e-6, -0.0059261471e-6,    -0.0022006663e-6, 82.884917e-6,  -9.1321119e-6,  87.620338e-6,          0.0,     0.001, -M_PI/2.0, -M_PI/2.0, -70.0*CTRL_DEG2RAD,     60.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0,							  0,		     0,		         0,					    0,				  0,				   0,			 0,				 0,				0,		 0.0225,    0.1005, -M_PI/2.0,  M_PI/2.0, -55.0*CTRL_DEG2RAD, 55.0*CTRL_DEG2RAD));

    // virtual links that describe T_nls (see http://eris.liralab.it/wiki/ICubInertiaSensorKinematics)
    pushLink(new iDynLink(1.3368659,		  -11.811104e-3, -5.7800518e-3,  -11.685197e-3,			3412.8918e-06,  66.297315e-6, -153.07583e-6, 4693.0882e-6,  8.0646052e-6, 4153.4285e-6, 0.0,    0.0066,  M_PI/2.0,       0.0,                0.0,               0.0));

    blockLink(3,0.0);

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCubNeckInertialDyn::alignJointsBounds(const deque<IControlLimits*> &lim)
{
    if (lim.size()<1)
        return false;

    IControlLimits &limHead =*lim[0];

    unsigned int iHead;
    double min, max;

    // only the neck: the sensor is in a virtual link
    for (iHead=0; iHead<3; iHead++)
    {   
        if (!limHead.getLimits(iHead,&min,&max))
            return false;

        (*this)[iHead].setMin(CTRL_DEG2RAD*min);
        (*this)[iHead].setMax(CTRL_DEG2RAD*max);
    }

    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


////////////////////////////////////////
//		ICUB INERTIAL SENSOR DYN V2          
////////////////////////////////////////

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubNeckInertialDynV2::iCubNeckInertialDynV2(const ChainComputationMode _mode)
{
    allocate("right");
	setIterMode(_mode);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubNeckInertialDynV2::iCubNeckInertialDynV2(const iCubNeckInertialDynV2 &sensor)
{
    clone(sensor);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCubNeckInertialDynV2::allocate(const string &_type)
{
    iDynLimb::allocate(_type);

	Matrix H0(4,4);
    H0.eye();
	setH0(H0);

	//sorry: mass and dynamic parameters have still to be calculated for the V2 head
	pushLink(new iDynLink(0,  0,0,0,  0,0,0,0,0,0,      0.0095,  0.0,	  M_PI/2.0,  M_PI/2.0, -40.0*CTRL_DEG2RAD, 30.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0,  0,0,0,  0,0,0,0,0,0,      0.0,     0.0,	 -M_PI/2.0, -M_PI/2.0, -70.0*CTRL_DEG2RAD, 60.0*CTRL_DEG2RAD));
    pushLink(new iDynLink(0,  0,0,0,  0,0,0,0,0,0,      0.0185,  0.1108, -M_PI/2.0,  M_PI/2.0, -55.0*CTRL_DEG2RAD, 55.0*CTRL_DEG2RAD));

	// NOTE: VERIFY THE DYNAMIC PARAMETERS OF THE HEAD WRITTEN BELOW 
    // virtual links that describe T_nls (see http://eris.liralab.it/wiki/ICubInertiaSensorKinematics)
    pushLink(new iDynLink(1.3368659,		  -11.811104e-3, -5.7800518e-3,  -11.685197e-3,			3412.8918e-06,  66.297315e-6, -153.07583e-6, 4693.0882e-6,  8.0646052e-6, 4153.4285e-6, 0.0,    0.0066,  M_PI/2.0,       0.0,                0.0,               0.0));

    blockLink(3,0.0);

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCubNeckInertialDynV2::alignJointsBounds(const deque<IControlLimits*> &lim)
{
    if (lim.size()<1)
        return false;

    IControlLimits &limHead =*lim[0];

    unsigned int iHead;
    double min, max;

    // only the neck: the sensor is in a virtual link
    for (iHead=0; iHead<3; iHead++)
    {   
        if (!limHead.getLimits(iHead,&min,&max))
            return false;

        (*this)[iHead].setMin(CTRL_DEG2RAD*min);
        (*this)[iHead].setMax(CTRL_DEG2RAD*max);
    }

    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

