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
#include <vector>
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
    // list of active taxel ids
    std::vector<unsigned int> taxelList;
    
public:
    //~~~~~~~~~~~~~~~~~~~~~~
	//   CONSTRUCTORS
	//~~~~~~~~~~~~~~~~~~~~~~
    /**
    * Empty contructor
    */
    skinContact();

    /**
    * Create a skinContact starting from a dynContact.
    */
    skinContact(const dynContact &c);

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
    skinContact(const BodyPart &_bodyPart, const SkinPart &_skinPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, 
        const yarp::sig::Vector &_geoCenter, unsigned int _activeTaxels, double _pressure);

    /**
    * Constructor
    * @param _bodyPart the part of the body
    * @param _skinPart the part of the skin
    * @param _linkNumber the link number relative to the specified body part
    * @param _CoP the center of pressure (link reference frame)
    * @param _geoCenter the geometric center of the contact area (link reference frame)
    * @param _taxelList list of activated taxels
    * @param _pressure average pressure applied on the contact area
    */
    skinContact(const BodyPart &_bodyPart, const SkinPart &_skinPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, 
        const yarp::sig::Vector &_geoCenter, std::vector<unsigned int> _taxelList, double _pressure);

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
    skinContact(const BodyPart &_bodyPart, const SkinPart &_skinPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, 
        const yarp::sig::Vector &_geoCenter, unsigned int _activeTaxels, double _pressure, const yarp::sig::Vector &_normalDir);

    /**
    * Constructor with contact surface normal.
    * @param _bodyPart the part of the body
    * @param _skinPart the part of the skin
    * @param _linkNumber the link number relative to the specified body part
    * @param _CoP the center of pressure (link reference frame)
    * @param _geoCenter the geometric center of the contact area (link reference frame)
    * @param _taxelList list of activated taxels
    * @param _pressure average pressure applied on the contact area
    * @param _normalDir contact area normal direction (link reference frame)
    */
    skinContact(const BodyPart &_bodyPart, const SkinPart &_skinPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, 
        const yarp::sig::Vector &_geoCenter, std::vector<unsigned int> _taxelList, double _pressure, const yarp::sig::Vector &_normalDir);

    //~~~~~~~~~~~~~~~~~~~~~~
	//   GET methods
	//~~~~~~~~~~~~~~~~~~~~~~
    const yarp::sig::Vector&            getGeoCenter()          const{ return geoCenter; }
    const yarp::sig::Vector&            getNormalDir()          const{ return normalDir; }
    double                              getPressure()   	    const{ return pressure; }
    unsigned int                        getActiveTaxels()       const{ return activeTaxels; }
    SkinPart                            getSkinPart()           const{ return skinPart; }
    std::string                         getSkinPartName()       const{ return SkinPart_s[skinPart]; }
    std::vector<unsigned int>           getTaxelList()          const{ return taxelList; }
 	
   
    //~~~~~~~~~~~~~~~~~~~~~~
	//   SET methods
	//~~~~~~~~~~~~~~~~~~~~~~    
    bool setGeoCenter(const yarp::sig::Vector &_geoCenter);
    bool setNormalDir(const yarp::sig::Vector &_normalDir);
    bool setPressure(double _pressure);
    void setActiveTaxels(unsigned int _activeTaxels);
    void setSkinPart(SkinPart _skinPart);
    void setTaxelList(const std::vector<unsigned int> &list);

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

    virtual std::string toString(int precision=-1) const;
   
};


}

}//end namespace

#endif


