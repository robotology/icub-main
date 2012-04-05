/*
 * Copyright (C) 2010-2011 RobotCub Consortium
 * Author: Andrea Del Prete
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/**
 * \defgroup skinDynLib
 *    
 * @ingroup iCubDev
 *  
 * Class representing a contact on the tactile sensor system (skin) of iCub.
 * 
 * \section intro_sec Description
 * 
 * \section tested_os_sec Tested OS
 * 
 * Windows
 *
 * \author Andrea Del Prete
 * 
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * 
 **/ 

#ifndef __SKINCONTLIST_H__
#define __SKINCONTLIST_H__

#include <vector>
#include <map>
#include <yarp/os/Portable.h>
#include "iCub/skinDynLib/skinContact.h"
#include "iCub/skinDynLib/dynContactList.h"

namespace iCub
{
namespace skinDynLib
{

/**
* Class representing a list of external contacts acting on the iCub' skin.
*/
class skinContactList  : public std::vector<skinContact>, public yarp::os::Portable
{
protected:
    
public:
    //~~~~~~~~~~~~~~~~~~~~~~
	//   CONSTRUCTORS
	//~~~~~~~~~~~~~~~~~~~~~~
    skinContactList();
    skinContactList(const size_type &n, const skinContact& value = skinContact());

    /**
    * Select all the contacts that have the specified body part.
    * @param bp the interested body part
    * @return a list containing only the skin contacts with the specified body part
    */
    virtual skinContactList filterBodyPart(const BodyPart &bp);

    /**
    * Split the list in N lists dividing the contacts per body part.
    */
    virtual std::map<BodyPart, skinContactList> splitPerBodyPart();


    //~~~~~~~~~~~~~~~~~~~~~~~~~
	//   SERIALIZATION methods
	//~~~~~~~~~~~~~~~~~~~~~~~~~
    /*
    * Read skinContactList from a connection.
    * return true iff a skinContactList was read correctly
    */
    virtual bool read(yarp::os::ConnectionReader& connection);

    /**
    * Write skinContactList to a connection.
    * return true iff a skinContactList was written correctly
    */
    virtual bool write(yarp::os::ConnectionWriter& connection);

    /**
     * Convert this skinContactList to a dynContactList casting all its elements
     * to dynContact.
     */
    virtual dynContactList toDynContactList() const;

    /**
    * Create a skinContactList starting from a dynContactList.
    * @return true if operation succeeded, false otherwise
    */
    //virtual bool fromDynContactList(dynContactList &l);
    
    /**
     * Useful to print some information.
     */
	virtual std::string toString() const;
};

}

}
#endif

