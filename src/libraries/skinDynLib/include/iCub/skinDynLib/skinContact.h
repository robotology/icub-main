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
    // contact area normal direction (link reference frame)
    yarp::sig::Vector normalDir;
    // number of taxels activated by the contact
    unsigned int activeTaxels;
    
public:
    //~~~~~~~~~~~~~~~~~~~~~~
	//   CONSTRUCTORS
	//~~~~~~~~~~~~~~~~~~~~~~
    /**
    * Empty contructor
    */
    skinContact();

    /**
    * Constructor
    * @param _bodyPart the part of the body
    * @param _skinPart the part of the skin
    * @param _linkNumber the link number relative to the specified body part
    * @param _CoP the center of pressure (link reference frame)
    * @param _geoCenter the geometric center of the contact area (link reference frame)
    * @param _activeTaxels number of taxels activated
    * @param _pressure average pressure applied on the contact area
    */
    skinContact(const BodyPart &_bodyPart, const SkinPart &_skinPart, const unsigned int &_linkNumber, const yarp::sig::Vector &_CoP, 
        const yarp::sig::Vector &_geoCenter, const unsigned int &_activeTaxels, const double &_pressure);

    /**
    * Constructor with contact surface normal.
    * @param _bodyPart the part of the body
    * @param _skinPart the part of the skin
    * @param _linkNumber the link number relative to the specified body part
    * @param _CoP the center of pressure (link reference frame)
    * @param _geoCenter the geometric center of the contact area (link reference frame)
    * @param _activeTaxels number of taxels activated
    * @param _pressure average pressure applied on the contact area
    * @param _normalDir contact area normal direction (link reference frame)
    */
    skinContact(const BodyPart &_bodyPart, const SkinPart &_skinPart, const unsigned int &_linkNumber, const yarp::sig::Vector &_CoP, 
        const yarp::sig::Vector &_geoCenter, const unsigned int &_activeTaxels, const double &_pressure, const yarp::sig::Vector &_normalDir);

    //~~~~~~~~~~~~~~~~~~~~~~
	//   GET methods
	//~~~~~~~~~~~~~~~~~~~~~~
	const yarp::sig::Vector&            getGeoCenter()          const;
    const yarp::sig::Vector&            getNormalDir()          const;
    const double&                       getPressure()   	    const;
	const unsigned int&                 getActiveTaxels()       const;    
    const SkinPart&                     getSkinPart()           const;
    std::string                         getSkinPartName()       const;
 	
   
    //~~~~~~~~~~~~~~~~~~~~~~
	//   SET methods
	//~~~~~~~~~~~~~~~~~~~~~~    
    bool setGeoCenter(const yarp::sig::Vector &_geoCenter);
    bool setNormalDir(const yarp::sig::Vector &_normalDir);
    bool setPressure(const double &_pressure);
    bool setActiveTaxels(const unsigned int &_activeTaxels);
    void setSkinPart(const SkinPart &_skinPart);

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

    virtual std::string toString(const int &precision=-1) const;
   
};


}

}//end namespace

#endif


