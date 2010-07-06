/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Matteo Fumagalli
* email:   matteo.fumagalli@iit.it
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
#include <iomanip>
#include <iCub/iDyn/iDynTransform.h>

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iKin;
using namespace iDyn;

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
iGenericFrame::iGenericFrame()
{
	initFTransform();
}
iGenericFrame::iGenericFrame(const Matrix &_R, double _x, double _y, double _z) 
{
	initFTransform();
	setP(_x,_y,_z);
	setR(_R);
	setH(R,p);
}
iGenericFrame::iGenericFrame(const Matrix &_R, const Vector &_p)
{
	initFTransform();
	setP(_p);
	setR(_R);
	setH(R,p);
}
void iGenericFrame::initFTransform()
{
	R.resize(3,3);
	H.resize(4,4);
	p.resize(3);
	FT.resize(6);
	p=0.0;
	R=eye(3,3);
	H=eye(4,4);
	FT=0.0;
}
void iGenericFrame::setR(const Matrix &_R)
{
	R=_R;
}
void iGenericFrame::setP(double _x, double _y, double _z)
{
	p(0)=_x;
	p(1)=_y;
	p(2)=_z;
}
void iGenericFrame::setP(const Vector &_p)
{
	p=_p;
}

void iGenericFrame::setPRH(const Matrix &_H)
{
	for(int i=0;i<3;i++)
		p(i)=_H(i,3);
	R=_H.submatrix(0,2,0,2);
	setH(_H);
}
void iGenericFrame::setPRH()
{
	for(int i=0;i<3;i++)
		p(i)=H(i,3);
	R=H.submatrix(0,2,0,2);
}
void iGenericFrame::setPRH(const Matrix &_R,const Vector &_p)
{
	setH(_R,_p);
	setPRH(H);
}
void iGenericFrame::setH(const Matrix &_R, double _x, double _y, double _z)
{
	H=eye(4,4);
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			H(i,j)=_R(i,j);
	H(0,3)=_x;
	H(1,3)=_y;
	H(2,3)=_z;
}
void iGenericFrame::setH(const Matrix &_R, const Vector &_p)
{
	H=eye(4,4);
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			H(i,j)=_R(i,j);
		}
		H(i,3)=_p(i);
	}
}
void iGenericFrame::setH(const Matrix &_H)
{
	R=_H.submatrix(0,2,0,2);
	for(int i=0;i<3;i++)
		p(i)=_H(i,3);
	H=_H;
}
yarp::sig::Vector iGenericFrame::setFT(const Vector &_FT)
{
		return FT=_FT;
}

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
// Definition of iFrameOnLink

iFrameOnLink::iFrameOnLink()
{
	l=0;
	initSFrame();
}

iFrameOnLink::iFrameOnLink(int _l)
{
	l=_l;
	initSFrame();	
}
void iFrameOnLink::initSFrame()
{
	H.resize(4,4);
	FT.resize(6);
	H=0.0;
	FT=0.0;
	Sensor=0;
	Limb=0;
	Link=new iGenericFrame();
}

void iFrameOnLink::setLink(int _l)
{
	l=_l;
}
void iFrameOnLink::setFT(const yarp::sig::Vector &_FT)
{
	FT=Sensor->setFT(_FT);
	//fprintf(stderr,"");
}

void iFrameOnLink::setSensorKin(int _l)
{
	Link->setPRH(Limb->getH(_l));
	H=Link->getH()*Sensor->getH();
}

void iFrameOnLink::setSensorKin()
{	
	Link->setPRH(Limb->getH(l));
	Matrix H1 = Link->getH();
	Matrix H2 = Sensor->getH();
	H=Link->getH()*Sensor->getH();
}

void iFrameOnLink::setSensorKin(const Matrix &_H)
{
	Link->setPRH(_H);
	H=_H*Sensor->getH();
}

void iFrameOnLink::setSensor(int _l, const yarp::sig::Vector &_FT)
{
	setSensorKin(_l);
	setFT(_FT);
}

void iFrameOnLink::setSensor(const yarp::sig::Matrix &_H, const yarp::sig::Vector &_FT)
{
	setSensorKin(_H);
	setFT(_FT);
}

void iFrameOnLink::setSensor(const yarp::sig::Vector &_FT)
{	
	setSensorKin();
	setFT(_FT);
}

Vector iFrameOnLink::getFT()
{
	return FT;
}
Matrix iFrameOnLink::getH()
{
	return H;
}

void iFrameOnLink::attach(iKinChain *_Limb)
{	
	Limb=_Limb;
}

void iFrameOnLink::attach(iGenericFrame *_Sensor)
{
	Sensor=_Sensor;
}

iFrameOnLink::~iFrameOnLink()
{
    if (Link)
        delete Link;
}

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////


iFTransformation::iFTransformation()
{
    ownLimb=true;
	Sensor=new iFrameOnLink();
	EndEffector=new iGenericFrame();
	Limb= new iDynChain();
	initiFTransformation();
}
iFTransformation::iFTransformation(int _l)
{
	l=_l;
    ownLimb=true;
	Sensor = new iFrameOnLink(l);
	Limb= new iKinChain();
	EndEffector=new iGenericFrame();	
	initiFTransformation();
	//Sensor->setLink(l);
}
iFTransformation::iFTransformation(iDynInvSensor *_iDynChainWithSensor)
{
    ownLimb=false;
	initiFTransformation();
	l=_iDynChainWithSensor->getSensorLink();
	Limb = _iDynChainWithSensor->chain;
	SensorFrame = new iGenericFrame(_iDynChainWithSensor->getH().submatrix(0,2,0,2),_iDynChainWithSensor->getH().submatrix(0,2,0,3).getCol(3));
	Sensor = new iFrameOnLink(l);
	EndEffector=new iGenericFrame();

	Sensor->attach(Limb);
	Sensor->attach(SensorFrame);
	//fprintf(stderr,"set up sensor transformation\n");
}
void iFTransformation::attach(iKin::iKinChain *_Limb)
{    
    if (Limb && ownLimb)
        delete Limb;

	Limb=_Limb;
	Sensor->attach(_Limb);
    ownLimb=false;
}
void iFTransformation::attach(iGenericFrame *_Sensor)
{    
    if (Sensor && ownSensor)
        //delete Sensor;

	Sensor->attach(_Sensor);
}
void iFTransformation::initiFTransformation()
{
	//Sensor=0;
	//Limb=0;
	//EndEffector=0;
	l=0;
	Hs.resize(4,4);
	Hs=0.0;
	Fs.resize(6);
	Fs=0.0;
	He.resize(4,4);
	He=0.0;
	Fe.resize(6);
	Fe=0.0;
	Tse.resize(6,6);
	Tse=0.0;
	Teb.resize(6,6);
	Teb=0.0;

	d.resize(3);
	d=0.0;
	S.resize(3,3);
	S=0.0;
	R.resize(3,3);
	R=0.0;

	SensorFrame=0;
}
void iFTransformation::setLink(int _l)
{
	Sensor->setLink(_l);
	l=_l;
}
void iFTransformation::setSensor(const Vector &_FT)
{
	Fs=_FT;	
	Sensor->setSensor(_FT);
	Hs=Sensor->getH();
}
void iFTransformation::setSensor(int _l, const Vector &_FT)
{
	Fs=_FT;
	l=_l;
	Sensor->setSensor(_l, _FT);
	Hs=Sensor->getH();
}
void iFTransformation::setSensor(const Matrix &_H, const Vector &_FT)
{
	Fs=_FT;
	Sensor->setSensor(_H, _FT);
	Hs=Sensor->getH();
}
void iFTransformation::setHe()
{
	EndEffector->setH(Limb->getH());
	He=EndEffector->getH();
}
void iFTransformation::setHe(int _l)
{
	EndEffector->setH(Limb->getH(_l));
	He=EndEffector->getH();
}
void iFTransformation::setHe(const Matrix &_H)
{
	EndEffector->setH(_H);
	He=EndEffector->getH();
}
void iFTransformation::setTeb()
{
	setHe();
	for(int i=0;i<3;i++)
	{
		for(int j=0; j<3; j++)
		{
			Teb(i,j)=He(i,j);
			Teb(i+3,j+3)=He(i,j);
		}
	}
}
yarp::sig::Vector iFTransformation::getEndEffWrench()
{
	setTse();
	setFe();
	return Fe;
}
yarp::sig::Vector iFTransformation::getEndEffWrench(const Vector &_FT)
{
	setSensor(_FT);
	return getEndEffWrench();
}
yarp::sig::Vector iFTransformation::getEndEffWrenchAsBase()
{
	getEndEffWrench();
	setTeb();
	return Teb*Fe;
}
yarp::sig::Vector iFTransformation::getEndEffWrenchAsBase(const Vector &_FT)
{
	setSensor(_FT);	
	return getEndEffWrenchAsBase();
}
void iFTransformation::setFe()
{
	Fe=Tse*Fs;
}
void iFTransformation::setTse()
{	
	S=0.0;
	Tse=0.0;
	R=0.0;
	for(int i=0; i<3;i++)
	{
		d(i)=(Hs(i,3)-He(i,3));
	}
	d=He.submatrix(0,2,0,2).transposed()*d;
	S(0,1)=-d(2);
	S(0,2)=d(1);
	S(1,0)=d(2);
	S(1,2)=-d(0);
	S(2,0)=-d(1);
	S(2,1)=d(0);

	R=He.submatrix(0,2,0,2).transposed()*Hs.submatrix(0,2,0,2);
	S=S*R;

	for(int i=0; i<3;i++)
	{
		for(int j=0; j<3;j++)
		{
			Tse(i,j)=R(i,j);
			Tse(i+3,j+3)=R(i,j);
			Tse(i+3,j)=S(i,j);
		}
		
	}

}
iFTransformation::~iFTransformation()
{
	if (Limb && ownLimb)
	{
		delete Limb;
		Limb = 0;
	}
	if (Sensor) 
	{
		delete Sensor;
		Sensor = 0;
	}
	if (EndEffector) 
	{
		delete EndEffector;
		EndEffector = 0;
	}
	if (SensorFrame) 
	{
		delete SensorFrame;
		SensorFrame = 0;
	}
}

