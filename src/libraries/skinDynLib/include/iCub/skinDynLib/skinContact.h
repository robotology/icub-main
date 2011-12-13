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

#ifndef __SKINCONT_H__
#define __SKINCONT_H__

#include <iostream>
#include <iomanip>
#include <yarp/sig/Vector.h>
#include "iCub/skinDynLib/dynContact.h"

namespace iCub
{

namespace skinDynLib
{

/**
* Class representing an external contact acting on the iCub' skin.
*/
class skinContact : public dynContact
{
protected:
    // id of the skin patch where the contact is applied
    SkinPart skinPart;
    // average pressure applied on the contact area 
    // (if the skin is not calibrated in pressure this is just the average taxel output)
    double pressure;
    // geometric center of the contact area expressed w.r.t. the reference frame of the link where the contact is applied
    yarp::sig::Vector geoCenter;
    // number of taxels activated by the contact
    unsigned int activeTaxels;
    
public:
    //~~~~~~~~~~~~~~~~~~~~~~
	//   CONSTRUCTORS
	//~~~~~~~~~~~~~~~~~~~~~~
    /**
    * Default contructor
    */
    skinContact();
    /**
    * 
    */
    skinContact(BodyPart _bodyPart, SkinPart _skinPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, 
        const yarp::sig::Vector &_geoCenter, unsigned int _activeTaxels, double _pressure);

    //~~~~~~~~~~~~~~~~~~~~~~
	//   GET methods
	//~~~~~~~~~~~~~~~~~~~~~~
	yarp::sig::Vector           getGeoCenter()          const;
    double	                    getPressure()   	    const;
	unsigned int                getActiveTaxels()       const;    
    SkinPart                    getSkinPart()           const;
    std::string                 getSkinPartName()       const;
 	
   
    //~~~~~~~~~~~~~~~~~~~~~~
	//   SET methods
	//~~~~~~~~~~~~~~~~~~~~~~    
    bool setGeoCenter(const yarp::sig::Vector &_geoCenter);
    bool setPressure(double _pressure);
    bool setActiveTaxels(unsigned int _activeTaxels);
    void setSkinPart(SkinPart _skinPart);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//   SERIALIZATION methods
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    /*
    * Read skinContact from a connection.
    * return true iff a skinContact was read correctly
    */
    virtual bool read(yarp::os::ConnectionReader& connection);
    /**
    * Write skinContact to a connection.
    * return true iff a skinContact was written correctly
    */
    virtual bool write(yarp::os::ConnectionWriter& connection);

    virtual std::string toString() const;
   
};


}

}//end namespace

#endif


