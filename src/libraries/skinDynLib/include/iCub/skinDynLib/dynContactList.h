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

#ifndef __DYNCONTLIST_H__
#define __DYNCONTLIST_H__

#include <vector>
#include <yarp/os/Portable.h>
#include "iCub/skinDynLib/dynContact.h"

namespace iCub
{
namespace skinDynLib
{

/**
* Class representing a list of external contacts.
*/
class dynContactList : public std::vector<dynContact>, public yarp::os::Portable
{
protected:
    
public:
    //~~~~~~~~~~~~~~~~~~~~~~
	//   CONSTRUCTORS
	//~~~~~~~~~~~~~~~~~~~~~~
    dynContactList();
    dynContactList( size_type n, const dynContact& value = dynContact());


    //~~~~~~~~~~~~~~~~~~~~~~~~~
	//   SERIALIZATION methods
	//~~~~~~~~~~~~~~~~~~~~~~~~~
    /*
    * Read dynContactList from a connection.
    * return true iff a dynContactList was read correctly
    */
    virtual bool read(yarp::os::ConnectionReader& connection);

    /**
    * Write dynContactList to a connection.
    * return true iff a dynContactList was written correctly
    */
    virtual bool write(yarp::os::ConnectionWriter& connection);

    
    /**
     * Useful to print some information.
     */
	virtual std::string toString() const;
};

}

}
#endif


