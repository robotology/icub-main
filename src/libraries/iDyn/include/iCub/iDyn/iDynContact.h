/*
 * Copyright (C) 2010-2011 RobotCub Consortium
 * Author: Andrea Del Prete
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/**
 * \defgroup iDynContact
 *    
 * @ingroup iDyn
 *  
 * Classes for external contact force/torque computation.
 * 
 * \section intro_sec Description
 * 
 * \section tested_os_sec Tested OS
 * 
 * Windows and Linux
 *
 *
 * \author Andrea Del Prete
 * 
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * 
 **/ 

#ifndef __IDYNCONT_H__
#define __IDYNCONT_H__

#include <iCub/iDyn/iDyn.h>
#include "iCub/skinDynLib/dynContactList.h"


namespace iCub
{

namespace iDyn
{

    // tollerance used to solve the linear system (all singular values < TOLLERANCE are considered zero)
    static double const TOLLERANCE = 10e-08;

/**
* \ingroup iDynContact
* This solver uses a modified version of the Newton-Euler algorithm to estimate both the external
* and internal forces/moments of a single kinematic chain.
* The solver assumes the contact locations are known and a F/T sensor measure is available.
*/
class iDynContactSolver : public iDynSensor
{
protected:        
    /// list of contacts acting on the link chain
    iCub::skinDynLib::dynContactList contactList;
    // empty list
    const iCub::skinDynLib::dynContactList nullList;
    // body part related to this solver
    iCub::skinDynLib::BodyPart      bodyPart;

    void findContactSubChain(unsigned int &firstLink, unsigned int &lastLink);
    yarp::sig::Matrix crossProductMatrix(const yarp::sig::Vector &v);
    
    yarp::sig::Matrix buildA(unsigned int firstContactLink, unsigned int lastContactLink);
    yarp::sig::Vector buildB(unsigned int firstContactLink, unsigned int lastContactLink);
    

public:
	

    /**
     * Constructor.
     * @param _c the robotic chain
     */
    iDynContactSolver(iDynChain *_c, const std::string &_info="", const NewEulMode _mode=DYNAMIC, 
        iCub::skinDynLib::BodyPart bodyPart = iCub::skinDynLib::UNKNOWN_BODY_PART, unsigned int verb=iCub::skinDynLib::NO_VERBOSE);
    
    /**
     * Constructor with F/T sensor.
     * @param _c the robotic chain
     * @param sensLink the index of the link containing the F/T sensor
     * @param sensor the F/T sensor
     */
    iDynContactSolver(iDynChain *_c, unsigned int sensLink, SensorLinkNewtonEuler *sensor, 
        const std::string &_info="", const NewEulMode _mode=DYNAMIC, iCub::skinDynLib::BodyPart bodyPart = iCub::skinDynLib::UNKNOWN_BODY_PART, 
        unsigned int verb=iCub::skinDynLib::NO_VERBOSE);

    /**
     * Constructor with F/T sensor information.
     * @param _c the robotic chain
     * @param sensLink the index of the link containing the F/T sensor
     * @param _H the homogeneous transformation matrix from the F/T sensor link to the F/T sensor sub-link
     * @param _H the homogeneous transformation matrix from the F/T sensor to the F/T sensor sub-link center of mass
     * @param _m mass of the F/T sensor sub-link
     * @param _I inertia of the F/T sensor sub-link
     */
    iDynContactSolver(iDynChain *_c, unsigned int sensLink, const yarp::sig::Matrix &_H, const yarp::sig::Matrix &_HC, 
        double _m, const yarp::sig::Matrix &_I, const std::string &_info="", const NewEulMode _mode=DYNAMIC, 
        iCub::skinDynLib::BodyPart bodyPart = iCub::skinDynLib::UNKNOWN_BODY_PART, unsigned int verb=iCub::skinDynLib::NO_VERBOSE);

    /**
     * Default destructor
     */
    ~iDynContactSolver();

    /**
     * Add a new element to the contact list. The content of this new element is initialized to a copy of "contact".
	 * @param contact the contact to add
	 * @return true if the operation is successful, false otherwise (eg index out of range)
     */
	bool addContact(const iCub::skinDynLib::dynContact &contact);

    /**
     * Add the specified elements to the contact list.
     * @param contacts the elements to add
     * @return true if operation succeeded, false otherwise
     */
    bool addContacts(const iCub::skinDynLib::dynContactList &contacts);

    /**
     * Clear the contact list.
     */
    void clearContactList();

    /**
     * Compute an estimate of the external contact wrenches.
     * @param FMsens the wrench measured by the F/T sensor
     * @return A copy of the external contact list
     */
    const iCub::skinDynLib::dynContactList& computeExternalContacts(const yarp::sig::Vector &FMsens);

    /**
     * Compute an estimate of the external contact wrenches 
     * (assuming the F/T sensor measure have already been set)
     * @return A copy of the external contact list
     */
    const iCub::skinDynLib::dynContactList& computeExternalContacts();

    /**
     * Compute an estimate of the external and internal contact wrenches (joint torques included).
     */
	void computeWrenchFromSensorNewtonEuler();

    //***************************************************************************************
    // GET METHODS
    //***************************************************************************************

    /**
     * @return A copy of the external contact list
     */
    const iCub::skinDynLib::dynContactList& getContactList() const;

    /**
	 * Returns the end effector force-moment as a single (6x1) vector
	 * @return a (6x1) vector, in the form 0:2=F 3:5=Mu 
	 */
    yarp::sig::Vector getForceMomentEndEff() const;

    /**
     * @return the number of unknowns in the current contact list
     */
    unsigned int getUnknownNumber() const;

	//***************************************************************************************
    // UTILITY METHODS
    //***************************************************************************************

	/**
	 * Compute the rototraslation matrix from frame a to frame b.
	 */
	yarp::sig::Matrix getHFromAtoB(unsigned int a, unsigned int b);

	/**
	 * Compute the wrench of the specified contact expressed w.r.t. the root reference
	 * frame of the chain (not the 0th frame, but the root).
	 */
	yarp::sig::Vector projectContact2Root(const iCub::skinDynLib::dynContact &c);

};


}

}//end namespace

#endif


