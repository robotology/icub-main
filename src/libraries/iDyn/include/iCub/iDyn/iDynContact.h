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
#include "iCub/skinDynLib/dynContact.h"


namespace iCub
{

namespace iDyn
{

/**
* \ingroup iDynContact
*/
class iDynContactSolver : public iDynSensor
{
protected:

    /// list of contacts acting on the link chain
    std::deque<iCub::skinDynLib::dynContact> contactList;

    void findContactSubChain(unsigned int &firstLink, unsigned int &lastLink);
    yarp::sig::Matrix crossProductMatrix(const yarp::sig::Vector &v);
    
    yarp::sig::Matrix buildA(unsigned int firstContactLink, unsigned int lastContactLink);
    yarp::sig::Vector buildB(unsigned int firstContactLink, unsigned int lastContactLink);
    

public:
	

    /**
     * Default constructor 
     */
    iDynContactSolver(iDynChain *_c, const std::string &_info="", const NewEulMode _mode=DYNAMIC, unsigned int verb=iCub::skinDynLib::NO_VERBOSE);
    
    iDynContactSolver(iDynChain *_c, unsigned int sensLink, SensorLinkNewtonEuler *sensor, 
        const std::string &_info="", const NewEulMode _mode=DYNAMIC, unsigned int verb=iCub::skinDynLib::NO_VERBOSE);
    /**
     * Constructor with F/T sensor information
     */
    iDynContactSolver(iDynChain *_c, unsigned int sensLink, const yarp::sig::Matrix &_H, const yarp::sig::Matrix &_HC, 
        double _m, const yarp::sig::Matrix &_I, const std::string &_info="", const NewEulMode _mode=DYNAMIC, 
        unsigned int verb=iCub::skinDynLib::NO_VERBOSE);
    /**
     * Default destructor
     */
    ~iDynContactSolver();

    /**
     * Add a new element to the contact list. The content of this new element is initialized to a copy of "contact".
	 * @param contact
	 * @return true if the operation is successful, false otherwise (eg index out of range)
     */
	bool addContact(const iCub::skinDynLib::dynContact &contact);
    bool addContacts(const std::deque<iCub::skinDynLib::dynContact> &contacts);
    bool removeContact(const int contactId);
    void clearContactList();

    /**
     * Compute an estimate of the external contact wrenches.
     * @return A copy of the external contact list
     */
    std::deque<iCub::skinDynLib::dynContact> computeExternalContacts(const yarp::sig::Vector &FMsens);

    std::deque<iCub::skinDynLib::dynContact> computeExternalContacts();

	void computeWrenchFromSensorNewtonEuler();

    //***************************************************************************************
    // GET METHODS
    //***************************************************************************************

    /**
     * @return A copy of the external contact list
     */
    std::deque<iCub::skinDynLib::dynContact> getContactList() const;
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


