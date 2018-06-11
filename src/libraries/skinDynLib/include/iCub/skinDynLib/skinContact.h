/*
 * Copyright (C) 2010-2011 RobotCub Consortium
 * Author: Andrea Del Prete
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/**
 * \defgroup skinDynLib skinDynLib
 *  
 * @ingroup icub_libraries 
 *    
 * Class representing a contact on the tactile sensor system (skin) of iCub.
 * 
 * \section intro_sec Description
 * 
 * \section tested_os_sec Tested OS
 * 
 * Windows, Linux
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
* @ingroup skinDynLib 
*  
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
    * Empty contructor.
    */
    skinContact();

    /**
    * Create a skinContact starting from a dynContact.
    */
    skinContact(const dynContact &c);

    /**
    * Constructor.
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
    * Constructor with list of active taxels.
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
    * Constructor with contact surface normal and list of active taxels.
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
    
     /**
    * Constructor with contact surface normal, list of active taxels, direction of contact force, and moment applied at this contact.
    * @param _bodyPart the part of the body
    * @param _skinPart the part of the skin
    * @param _linkNumber the link number relative to the specified body part
    * @param _CoP the center of pressure (link reference frame)
    * @param _geoCenter the geometric center of the contact area (link reference frame)
    * @param _taxelList list of activated taxels
    * @param _pressure average pressure applied on the contact area
    * @param _normalDir contact area normal direction (link reference frame)
    * @param _F force applied at contact, expressed in link reference frame
    * @param _Mu the moment applied at this contact, expressed in link reference frame
    */
    skinContact(const BodyPart &_bodyPart, const SkinPart &_skinPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, 
        const yarp::sig::Vector &_geoCenter, std::vector<unsigned int> _taxelList, double _pressure, const yarp::sig::Vector &_normalDir,
    const yarp::sig::Vector &_F, const yarp::sig::Vector &_Mu);


    //~~~~~~~~~~~~~~~~~~~~~~
    //   GET methods
    //~~~~~~~~~~~~~~~~~~~~~~
    /**
    * Get the contact geometric center, expressed in the link reference frame.
    * @return a 3-dim vector
    */
    const yarp::sig::Vector&            getGeoCenter()          const{ return geoCenter; }
    /**
    * Get the direction normal to the contact area, expressed in the link reference frame.
    * @return a 3-dim vector
    */
    const yarp::sig::Vector&            getNormalDir()          const{ return normalDir; }
    /**
    * Get the average pressure measured at this contact.
    * @return the contact pressure
    */
    double                              getPressure()           const{ return pressure; }
    /**
    * Get the number of active taxels.
    * @return the number of taxels activated by this contact
    */
    unsigned int                        getActiveTaxels()       const{ return activeTaxels; }
    /**
    * Get the skin part on which this contact is applied.
    * @return the skin part
    */
    SkinPart                            getSkinPart()           const{ return skinPart; }
    /**
    * Get the name of the skin part on which this contact is applied.
    * @return the skin part name
    */
    std::string                         getSkinPartName()       const{ return SkinPart_s[skinPart]; }
    /**
    * Get the list of id's of the taxels activated by this contact
    * @return the list of taxels' id's
    */
    std::vector<unsigned int>           getTaxelList()          const{ return taxelList; }
    
   
    //~~~~~~~~~~~~~~~~~~~~~~
    //   SET methods
    //~~~~~~~~~~~~~~~~~~~~~~    
    /**
     * Set the geometric center of the contact area (link reference frame).
     * @param _geoCenter a 3-dim vector containing the geometric center of this contact area
     * @return true if the operation succeeded, false otherwise
     */
    bool setGeoCenter(const yarp::sig::Vector &_geoCenter);
    /**
     * Set the normal direction of the contact area, expressed in link reference frame.
     * @param _normalDir a 3-dim vector containing the normal to the contact area
     * @return true if the operation succeeded, false otherwise
     */
    bool setNormalDir(const yarp::sig::Vector &_normalDir);
    /**
     * Set the average contact pressure.
     * @param _pressure the mean contact pressure
     * @return true if the operation succeeded, false otherwise
     */
    bool setPressure(double _pressure);
    /**
     * Set the number of active taxels.
     * @param _activeTaxels the number of taxels activated by this contact
     * @return true if the operation succeeded, false otherwise
     */
    void setActiveTaxels(unsigned int _activeTaxels);
    /**
     * Set the skin part on which this contact is applied.
     * @param _skinPart the skin part
     * @return true if the operation succeeded, false otherwise
     */
    void setSkinPart(SkinPart _skinPart);
    /**
     * Set the list of taxels that are activated by this contact
     * @param list list of id's of the active taxels associated to this contact
     * @return true if the operation succeeded, false otherwise
     */
    void setTaxelList(const std::vector<unsigned int> &list);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //   SERIALIZATION methods
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    /*
    * Read skinContact from a connection.
    * @param connection connection to read from
    * @return true iff a skinContact was read correctly
    */
    virtual bool read(yarp::os::ConnectionReader& connection) override;

    /**
    * Write this skinContact to a connection.
    * The skinContact is represented as a list of 8 elements that are:
    * - a list of 4 int, i.e. contactId, bodyPart, linkNumber, skinPart
    * - a list of 3 double, i.e. the CoP
    * - a list of 3 double, i.e. the force
    * - a list of 3 double, i.e. the moment
    * - a list of 3 double, i.e. the geometric center
    * - a list of 3 double, i.e. the normal direction
    * - a list of N int, i.e. the active taxel ids
    * - a double, i.e. the pressure
    * @param connection connection to write to
    * @return true iff a skinContact was written correctly
    */
    virtual bool write(yarp::os::ConnectionWriter& connection) const override;

    /**
    * Convert this skinContact to a vector. The size of the vector is 21 plus
    * the number of active taxels. The vector contains this data, in this order:
    * 0: contactId; 
    * 1: body part id; 
    * 2: link number; 
    * 3: skin part; 
    * 4-6: center of pressure; 
    * 7-9: force; 
    * 10-12: moment; 
    * 13-15: geometric center; 
    * 16-18: surface normal direction; 
    * 19: number of active taxels;
    * 20-?: list of the id's of the activated taxels; 
    * ?: pressure;
    * @return a Vector representation of this skinContact
    */
    virtual yarp::sig::Vector toVector() const;

    /**
    * Convert the specified vector into a skinContact. The vector has to contain
    * the following data, in this specific order:
    * 0: contactId; 
    * 1: body part id; 
    * 2: link number; 
    * 3: skin part; 
    * 4-6: center of pressure; 
    * 7-9: force; 
    * 10-12: moment; 
    * 13-15: geometric center; 
    * 16-18: surface normal direction; 
    * 19: number of active taxels;
    * 20-?: list of the id's of the activated taxels; 
    * ?: pressure;
    * @param v the vector to convert into a skinContact
    * @return true iff operation succeeded, false otherwise
    */
    virtual bool fromVector(const yarp::sig::Vector &v);

    /**
     * Convert this skinContact into a string. Useful to print some information.
     * @param precision number of decimal digits to use in the string representation
     * @return a string representation of this contact
     */
    virtual std::string toString(int precision=-1) const override;
   
};


}

}//end namespace

#endif


