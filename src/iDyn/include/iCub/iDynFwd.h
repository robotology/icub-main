/**
 * \defgroup iDynFwd iDynFwd 
 *    
 * @ingroup iDyn
 *  
 * Classes for link/joint force/moment/torque computation 
 * using Newton-Euler recursive formulation, given single 
 * FT sensor measurements.
 * 
 * \section intro_sec Description
 * 
 * iDynSensor is a class which attachs a FT sensor into a 
 * iDynChain; exploiting the sensor force/moment measurements, 
 * and applying the inverse formula of Newton-Euler recursive 
 * algorithm, it is possible to compute joint torques given
 * single FT measurements. The dynamical parameters necessary 
 * to the algorithm are read automatically from the iDynChain 
 * (consisting of iDynLinks), whereas the FT sensor must be 
 * set from iDynSensor constructor. 
 * 
 * \section tested_os_sec Tested OS
 * 
 * Windows
 *
 * \section example_sec Example
 *
 * See iDynChain examples.
 *
 * \author Serena Ivaldi, Matteo Fumagalli
 * 
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * 
 **/ 

#ifndef __IDYNFWD_H__
#define __IDYNFWD_H__

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <iCub/ctrlMath.h>
#include <iCub/iKinFwd.h>
#include <iCub/iDyn.h>
#include <deque>
#include <string>


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
	class OneLinkNewtonEuler;
	class BaseLinkNewtonEuler;
	class FinalLinkNewtonEuler;
	class SensorLinkNewtonEuler;
	class OneChainNewtonEuler;
	class iDynInvSensor;
	class iFTransform;
    class iSFrame;
	class iFB;


/**
* \ingroup iDynFwd
*
* A class for computing forces and torques in a iDynChain, 
* when a force/torque sensor is placed in the middle of the 
* kinematic chain and it is the only available sensor for 
* measuring forces and moments; the sensor position in the 
* chain must be set; the computation of joint forces, moments 
* and torques is performed by an Inverse Newton-Euler method.
* 
*/
class iDynSensor: public iDynInvSensor
{
protected:

	/**
     * Base function for forward of Newton-Euler, attaching the sensor in the forward process.
     */
	void ForwardFromBase();

	
	

public:

	/**
    * Constructor without FT sensor: the sensor must be set with setSensor()
	* @param _c a pointer to the iDynChain where the sensor is placed on
	* @param _info a string with information
	* @param _mode the analysis mode (static/dynamic)
	* @param verb flag for verbosity
    */
	iDynSensor(iDyn::iDynChain *_c, std::string _info, const NewEulMode _mode = NE_static, bool verb = false);


	/**
    * Constructor with FT sensor
	* @param _c a pointer to the iDynChain where the sensor is placed on
	* @param i the i-th link to whom the sensor is attached
	* @param _H the roto-traslational matrix from the reference frame of the i-th link to the sensor
	* @param _HC the roto-traslational matrix of the center of mass of the semi-link defined by the sensor in the i-th link
	* @param _m the mass of the semi-link
	* @param _I the inertia of the semi-link
	* @param _info a string with information
	* @param _mode the analysis mode (static/dynamic)
	* @param verb flag for verbosity
    */
	iDynSensor(iDyn::iDynChain *_c, unsigned int i, const yarp::sig::Matrix &_H, const yarp::sig::Matrix &_HC, const double _m, const yarp::sig::Matrix &_I, std::string _info, const NewEulMode _mode = NE_static, bool verb = false);

	/**
	 * Set the sensor measured force and moment
     * @param F the sensor force (3x1)
	 * @param Mu the sensor moment (3x1)
	 * @return true if the operation is successful, false otherwise (ie wrong vector size)
	 */
	bool setSensorMeasures(const yarp::sig::Vector &F, const yarp::sig::Vector &Mu);

	/**
	 * Set the sensor measured force and moment at once. The measure vector (6x1) is made
	 * of 0:2=force 3:5=moment
     * @param FM the sensor force and moment (6x1)
	 * @return true if the operation is successful, false otherwise (ie wrong vector size)
	 */
	bool setSensorMeasures(const yarp::sig::Vector &FM);

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//   main computation methods
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


	bool computeFromSensorNewtonEuler(const yarp::sig::Vector &F, const yarp::sig::Vector &Mu);

	void computeFromSensorNewtonEuler();

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//  get methods, overloaded from iDyn
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	/**
    * Returns the links forces as a matrix, where the i-th col is the i-th force
    * @return a 3xN matrix with forces, in the form: i-th col = F_i
    */
	yarp::sig::Matrix getForces() const;

	/**
    * Returns the links moments as a matrix, where the i-th col is the i-th moment
    * @return a 3xN matrix with moments, in the form: i-th col = Mu_i
    */
	yarp::sig::Matrix getMoments() const;

	/**
    * Returns the links torque as a vector
    * @return a vector with the torques
    */
	yarp::sig::Vector getTorques() const;

	/**
    * Returns the i-th link force
    * @return the i-th link force
    */
	yarp::sig::Vector getForce(const unsigned int iLink) const;

	/**
    * Returns the i-th link moment
    * @return the i-th link moment
    */
	yarp::sig::Vector getMoment(const unsigned int iLink) const;

	/**
    * Returns the i-th link torque
    * @return the i-th link torque
    */
	double getTorque(const unsigned int iLink) const;

	/**
    * Returns the links forces as a matrix, where the i-th col is the i-th force
    * @return a 3xN matrix with forces, in the form: i-th col = F_i
    */
	yarp::sig::Matrix getForcesNewtonEuler() const;

	/**
    * Returns the links moments as a matrix, where the i-th col is the i-th moment
    * @return a 3xN matrix with moments, in the form: i-th col = Mu_i
    */
	yarp::sig::Matrix getMomentsNewtonEuler() const;

	/**
    * Returns the links torque as a vector
    * @return a vector with the torques
    */
	yarp::sig::Vector getTorquesNewtonEuler() const;


};



}//end namespace

#endif