/*
 * Copyright (C) 2010-2011 RobotCub Consortium
 * Author: Andrea Del Prete
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/**
 * \defgroup iCubCommons
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
    skinContactList( size_type n, const skinContact& value = skinContact());


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
     * Useful to print some information.
     */
	virtual std::string toString() const;
};

}

}
#endif
