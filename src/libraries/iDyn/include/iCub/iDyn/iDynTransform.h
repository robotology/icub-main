/**
 * \defgroup iFC iFC
 *  
 * @ingroup iDyn
 *  
 * Classes for FT sensor transformation
 *  
 * \note SI units adopted: Newton for Forces and Newton-Meter for
 *       Torques.

 * \section dep_sec Dependencies 
 * - iKin
 *  
 * \author Matteo Fumagalli
 *  
 * \section intro_sec Description
 *
 * Classes for insertion of a FT sensor on a Link of a Chain and its prijection 
 * on the base frame.
 *
 * Date: first release 16/10/2009
 *
 *
 */

/* How to use the iFC library:

istantiate your limb (example: iCubArm *arm)
istantiate a iFrameOnLink Variable as pointer (iFrameOnLink *Sens) and a pointer to a variable of type iGenericFrame
(ex: iGenericFrame *sensore)
Sens need a iGenericFrame and an iKinLimb variables with some properties defining the location of the sensor over a link of the 
iKinLimb chain.

example code:

	Matrix Rs(3,3);
	Vector ps(3);
	Rs=...
	ps=...
	iCubArm *arm = new iCubArm("left");
	iKinChain *chain;
	iGenericFrame *sensore = new iGenericFrame(Rs,ps);
	iFrameOnLink *Sens = new iFrameOnLink(2);
	chain=arm->asChain();
	
	Sens->attach(sensore);
	Sens->attach(arm);
		
	Vector q(7);
	q=...
	Vector FT;
	FT.resize(6);
	FT = ReadFT(); //a function that reads the FT values from a sensor

	Vector FT_B; //will be the vector of forces and torqes in the base frame
	
	arm->setAng(q);
	sensore->setFT(FT);
	FT_B = Sens->getFT();
 *
 */ 

#ifndef __IFORCECONTROL_H__
#define __IFORCECONTROL_H__

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <iCub/ctrl/ctrlMath.h>
#include <iCub/iKin/iKinFwd.h>

#include <string>
//#include <deque>


namespace iDyn{

	class iDynLink;
	class iDynChain;
	class iDynLimb;
	class iCubArmDyn;
	class iCubLegDyn;
	class iCubEyeDyn;
	class iCubEyeNeckRefDyn;
	class iCubInertialSensorDyn;
	class iFakeDyn;
	class iFakeDyn2GdL;
	class iDynSensor;
	class iDynSensorLeg;
	class iDynSensorArm;
	class iDynInvSensor;
	class OneLinkNewtonEuler;
	class BaseLinkNewtonEuler;
	class FinalLinkNewtonEuler;
	class SensorLinkNewtonEuler;
	class OneChainNewtonEuler;
	class iGenericFrame;
    class iFrameOnLink;
	class iFTransformation;


/**
* \ingroup iFC
*
* A Base class for defining the Transformation of a Wrench from a frame to another.
*/
class iGenericFrame
{
private:
	    // Default constructor: not implemented.
	yarp::sig::Matrix R; // Rotation Matrix wrt the link reference frame on which the FT sensor is put 
	yarp::sig::Vector p; // Vector distance of the FT sensor wrt the link reference frame on which the FT sensor is put 
	yarp::sig::Matrix H; // Rototranslation matrix
	yarp::sig::Vector FT;
public:
	/* Default Constructor */
	iGenericFrame();
	/* 
	* Overload constructor
	* @param _R is the rotation matrix with respect to another reference frame
	* @param _x, _y, _z are the position vector of one reference frame with respect 
	* to another 
	*/
	iGenericFrame(const yarp::sig::Matrix &_R, double _x, double _y, double _z);
	/* 
	* Overload constructor
	* @param _R as previously
	* @param _p is a vector composed by {_x, _y, _z} of the previous definition 
	*/
	iGenericFrame(const yarp::sig::Matrix &_R, const yarp::sig::Vector &_p);

	/*Initializes the variables. Used in all the constructors*/
	void initFTransform();

	/*
	* Set the Rotation matrix among two reference Frame 
	* @param _R is the rotation matrix with respect to another reference frame
	*/
	void setR(const yarp::sig::Matrix &_R);

	/* 
	* Set the distance between the two reference Frame 
	* @param _x, _y, _z are the position vector of one reference frame with respect 
	* to another 
	*/
	void setP(double _x, double _y, double _z);
	/* 
	* Set the distance between the two reference Frame 
	* where _p is composed by {_x, _y, _z} of the previous definition 
	*/
	void setP(const yarp::sig::Vector &_p);

	/* Set the homogeneous transformation among two frames */
	/*
	* @param _R is the rotation matrix with respect to another reference frame
	* @param _x, _y, _z are the position vector of one reference frame with respect 
	* to another 
	*/
	void setH(const yarp::sig::Matrix &_R, double _x, double _y, double _z);
	/*
	* @param _R is the rotation matrix with respect to another reference frame
	* @param _p is composed by {_x, _y, _z} of the previous definition 
	*/
	void setH(const yarp::sig::Matrix &_R, const yarp::sig::Vector &_p);
	/*
	* @param _H is the homogeneous transformation matrix among 2 FoR
	*/
	void setH(const yarp::sig::Matrix &_H);

	/*set every variable (H, R, p, S, T) given H(4,4)
	* H is set as default or using previously a setH(...)
	*/
	void setPRH(void);
	/*
	* @param _R is the rotation matrix with respect to another reference frame
	* @param _p is the distance vector among 2 FoR
	*/
	void setPRH(const yarp::sig::Matrix &_R, const yarp::sig::Vector &_p);
	/*
	* @param _H is the homogeneous transformation matrix among 2 FoR
	*/
	void setPRH(const yarp::sig::Matrix &_H);

	/* Set the FT datas */
	/*
	* @param _FT is the wrench to be transformed, in its native FoR
	*/
	yarp::sig::Vector setFT(const yarp::sig::Vector &_FT);

	/* get the members defined above */
	yarp::sig::Vector getP(){return p;}
	yarp::sig::Matrix getR(){return R;}
	yarp::sig::Matrix getH(){return H;}

	/*return the Wrench set with setFT*/
	yarp::sig::Vector getFT(){return FT;}
	
	/* Destructor*/ 
	~iGenericFrame(){};
};

/**
* \ingroup iFC
*
* A Base class for defining the FT sensor over a generic link of a kinematic chain inherited by iKinLimb.
*/

class iFrameOnLink
{
private:
	int l;
	yarp::sig::Matrix H;
	yarp::sig::Vector FT;

	iGenericFrame *Sensore;
	iGenericFrame *Link;
	iKin::iKinChain *Limb;
	/*
	* initializes the iFrameOnLink members to zero. Used in the constructors.
	*/
	void initSFrame();

protected:

	//set transformation variables of/ the sensor:
	void setSensorKin(int _l);
	void setSensorKin(const yarp::sig::Matrix &_H);
	void setSensorKin();

	void setFT(const yarp::sig::Vector &_FT);
public:
	// Default constructor:
    iFrameOnLink();
	/* Overloaded constructor:
	* @param _l defines the link on which the FT sensor is attached
	*/
	iFrameOnLink(int _l);
	/*Destructor*/
	~iFrameOnLink()
	{
	};

	void setLink(int _l);
	//Set sensor variables:
	/* @param _Rs is the rotation matrix of the sensor FoR with respect to the one of the link the sensor is attached to.*/
	void setRs(const yarp::sig::Matrix &_Rs){	Sensore->setR(_Rs);}
	/* @param _ps is the position vector of the sensor FoR with respect to the one of the link the sensor is attached to.*/
	void setPs(const yarp::sig::Vector &_ps){Sensore->setP(_ps);}
	/* @param _Hs is the homogeneous transformation matrix of the sensor FoR with respect to the one
	* of the link the sensor is attached to. */
	void setHs(const yarp::sig::Matrix &_Hs){Sensore->setH(_Hs);}

	//get sensor members, previously defined
	yarp::sig::Matrix getRs(){return Sensore->getR();}
	yarp::sig::Vector getPs(){return Sensore->getP();}
	yarp::sig::Matrix getHs(){return Sensore->getH();}

	//get Limb variables
	/* @param _Rl is the rotation matrix of the link FoR with respect to the base FoR*/
	yarp::sig::Matrix getRl(){return Link->getR();}
	/* @param _pl is the position vector of the sensor FoR with respect to the base FoR	*/
	yarp::sig::Vector getPl(){return Link->getP();}
	/* @param _Hl is the homogeneous transformation matrix of the sensor FoR with respect to the base FoR	*/
	yarp::sig::Matrix getHl(){return Link->getH();}

	

	//set transformation variables of the sensor, with respect to a base frame:
	yarp::sig::Matrix getH();
	
	void setSensor(int _l, const yarp::sig::Vector &_FT);
	void setSensor(const yarp::sig::Matrix &_H, const yarp::sig::Vector &_FT);
	void setSensor(const yarp::sig::Vector &_FT);

	// get FT variables with respect to a fixed reference frame
	yarp::sig::Vector getFT();

	void attach(iKin::iKinChain *_Limb);
	void attach(iGenericFrame *_Sensore);
};


/**
* \ingroup iFC
*
*/
class iFTransformation
{
private:
	int l;
	yarp::sig::Matrix Hs;
	yarp::sig::Vector Fs;
	yarp::sig::Matrix He;
	yarp::sig::Vector Fe;
	yarp::sig::Matrix Tse;
	yarp::sig::Matrix Teb;

	iFrameOnLink* Sensore;
	iGenericFrame* EndEffector;
	iKin::iKinChain *Limb;

	yarp::sig::Vector d;
	yarp::sig::Matrix S;
	yarp::sig::Matrix R;

	void initiFTransformation();
public:
	iFTransformation();
	iFTransformation(int _l);

	~iFTransformation()
	{
		if (Limb)
		{
			delete Limb;
			Limb = 0;
		}
		if (Sensore) 
		{
			delete Sensore;
			Sensore = 0;
		}
		if (EndEffector) 
		{
			delete EndEffector;
			EndEffector = 0;
		}
	}
	void attach(iKin::iKinChain *_Limb);
	void attach(iGenericFrame *_sensore);

	void setLink(int _l);

	void setSensor(const yarp::sig::Vector &_FT);
	void setSensor(int _l, const yarp::sig::Vector &_FT);
	void setSensor(const yarp::sig::Matrix &_H, const yarp::sig::Vector &_FT);

	void setHe();
	void setHe(int _l);
	void setHe(const yarp::sig::Matrix &_H);

	void setTeb();
	void setTse();
	yarp::sig::Vector getFB();
	yarp::sig::Vector setFe();
	yarp::sig::Vector getFe();

	yarp::sig::Vector getFB(const yarp::sig::Vector &_FT);
	yarp::sig::Matrix getHs(){return Hs;}
	yarp::sig::Matrix getHe(){return He;}

	
};

}//end namespace

#endif
