/*
 * Copyright (C) 2010-2011 RobotCub Consortium
 * Author: Andrea Del Prete
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/**
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

#ifndef __DINCONT_H__
#define __DINCONT_H__

#include <yarp/os/Portable.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include "iCub/skinDynLib/common.h"

namespace iCub
{
namespace skinDynLib
{

/** 
* @ingroup skinDynLib 
*  
* Class representing an external contact acting on a link of the robot body.
* A contact is identified by a unique id, a robot body part, a link number, 
* the center of pressure (CoP), the contact force and moment.
* All the vectors (i.e. CoP, force, moment) are expressed w.r.t. the reference frame of the link.
*
* This class is used in the process of estimating the contact wrenches.
* In the estimation process the link number and the contact position are supposed to be known.
* The force direction and the moment may be either known or unknown, depending on the sensor
* data.
*/
class dynContact : public yarp::os::Portable
{
protected:
    // static variable containing the id of the last contact created
    static unsigned long ID;
    // unique id of the contact
    unsigned long contactId;

    /// part of the body of the robot where the contact is applied
    BodyPart bodyPart;
    /// number of the link where the contact is applied
    unsigned int linkNumber;
    /// center of pressure of the contact expressed w.r.t. the reference frame
    yarp::sig::Vector CoP;
    ///contact force direction (unit vector)
    yarp::sig::Vector Fdir;
    ///contact force
    yarp::sig::Vector F;
    /// contact force module
    double Fmodule;
    /// contact moment
    yarp::sig::Vector Mu;

    /// True if the moment applied at the contact point is known
    bool muKnown;
    /// True if the direction of the force applied at the contact point is known
    bool fDirKnown;

    ///verbosity flag
    unsigned int verbose;

    /// name of the link on which the contact is applied
    std::string linkName;
    /// name of the frame on which all the vector quantities are expressed
    std::string frameName;

    void init(const BodyPart &_bodyPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, 
        const yarp::sig::Vector &_Mu=yarp::sig::Vector(0), const yarp::sig::Vector &_Fdir=yarp::sig::Vector(0));
    bool checkVectorDim(const yarp::sig::Vector &v, unsigned int dim, const std::string &descr="");

public:
    //~~~~~~~~~~~~~~~~~~~~~~
    //   CONSTRUCTORS
    //~~~~~~~~~~~~~~~~~~~~~~
    /**
    * Default constructor
    */
    dynContact();
    /**
    * Constructor with unknown moment and force direction
    * @param _bodyPart the body part associated to this contact
    * @param _linkNumber the number of the link on which this contact is applied
    * @param _CoP the 3-dim center of pressure of this contact in link reference frame
    */
    dynContact(const BodyPart &_bodyPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP);
    /**
    * Constructor with known moment (usually zero) and unknown force direction
    * @param _bodyPart the body part associated to this contact
    * @param _linkNumber the number of the link on which this contact is applied
    * @param _CoP the 3-dim center of pressure of this contact in link reference frame
    * @param _Mu the moment applied at this contact, expressed in link reference frame
    */
    dynContact(const BodyPart &_bodyPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, const yarp::sig::Vector &_Mu);
    /**
    * Constructor with known moment (usually zero) and known force direction
    * @param _bodyPart the body part associated to this contact
    * @param _linkNumber the number of the link on which this contact is applied
    * @param _CoP the 3-dim center of pressure of this contact in link reference frame
    * @param _Mu the moment applied at this contact, expressed in link reference frame
    * @param _Fdir the direction of the contact force, expressed in link reference frame
    */
    dynContact(const BodyPart &_bodyPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, 
        const yarp::sig::Vector &_Mu, const yarp::sig::Vector &_Fdir);

    //~~~~~~~~~~~~~~~~~~~~~~
    //   GET methods
    //~~~~~~~~~~~~~~~~~~~~~~
    /**
     * Get the contact force and moment in a single (6x1) vector
     * @return a 6x1 vector where 0:2=force 3:5=moment
     */
    virtual yarp::sig::Vector           getForceMoment()        const;
    /**
    * Get the contact force.
    * @return a 3-dim vector
    */
    virtual const yarp::sig::Vector&    getForce()              const;
    /**
    * Get the contact force direction.
    * @return a 3-dim vector with norm equal to one
    */
    virtual const yarp::sig::Vector&    getForceDirection()     const;
    /**
    * Get the contact force module.
    * @return the intensity of the contact force
    */
    virtual double                      getForceModule()        const;
    /**
    * Get the contact moment.
    * @return a 3-dim vector
    */
    virtual const yarp::sig::Vector&    getMoment()             const;
    /**
    * Get the contact center of pressure expressed in the reference frame.
    * @return a 3-dim vector
    */
    virtual const yarp::sig::Vector&    getCoP()                const;
    /**
    * Get the link number (where 0 is the first link of the chain).
    * @return the link number
    */
    virtual unsigned int                getLinkNumber()         const;
    /**
    * Get the body part of the contact.
    * @return the contact body part
    */
    virtual BodyPart                    getBodyPart()           const;
    /**
    * Get the name of the contact body part.
    * @return the name of the contact body part
    */
    virtual std::string                 getBodyPartName()       const;
    /**
    * Get the id of this contact.
    * @return the contact id
    */
    virtual unsigned long               getId()                 const;
    /**
    * Get the name of the link on which the contact is applied.
    * @return the link name
    */
    virtual std::string                 getLinkName()           const;
    /**
     * Get the name of the frame in which the quantities of this contact are expressed.
     * @return the link name
     */
    virtual std::string                 getFrameName()           const;
    //~~~~~~~~~~~~~~~~~~~~~~
    //   IS methods
    //~~~~~~~~~~~~~~~~~~~~~~
    /**
    * Get true if the moment applied at this contact is known a-priori.
    * @return true if the moment is known, false otherwise.
    */
    virtual bool isMomentKnown()                const;
    /**
    * Get true if the direction of the force applied at this contact is known a-priori.
    * @return true if the force direction is known, false otherwise.
    */
    virtual bool isForceDirectionKnown()        const;
   
    //~~~~~~~~~~~~~~~~~~~~~~
    //   SET methods
    //~~~~~~~~~~~~~~~~~~~~~~    
    /**
     * Set the contact force
     * @param _F a 3-dim vector containing the contact force
     * @return true if the operation succeeded, false otherwise
     */
    virtual bool setForce(const yarp::sig::Vector &_F);
    /**
     * Set the contact force module
     * @param _Fmodule the module of the contact force
     * @return true if the operation succeeded, false otherwise
     */
    virtual bool setForceModule(double _Fmodule);
    /**
     * Set the direction of the contact force
     * @param _Fdir a 3-dim vector containing the direction of the contact force
     * @return true if the operation succeeded, false otherwise
     * @note If norm(_Fdir) is not 1 then _Fdir is normalized so that its norm is 1
     */
    virtual bool setForceDirection(const yarp::sig::Vector &_Fdir);
    /**
     * Set the contact moment
     * @param _Mu a 3-dim vector containing the contact moment
     * @return true if the operation succeeded, false otherwise
     */
    virtual bool setMoment(const yarp::sig::Vector &_Mu);
    /**
     * Set the contact force and moment
     * @param _F a 3-dim vector containing the contact force
     * @param _Mu a 3-dim vector containing the contact moment
     * @return true if the operation succeeded, false otherwise
     */
    virtual bool setForceMoment(const yarp::sig::Vector &_F, const yarp::sig::Vector &_Mu);
    /**
     * Set the contact force and moment as a single (6x1) vector
     * @param _FMu a 6x1 vector where 0:2=force 3:5=moment
     * @return true if the operation succeeded, false otherwise
     */
    virtual bool setForceMoment(const yarp::sig::Vector &_FMu);
    /**
     * Set the contact center of pressure in reference frame
     * @param _CoP a 3x1 vector
     * @return true if the operation succeeded, false otherwise
     */
    virtual bool setCoP(const yarp::sig::Vector &_CoP);
    /**
     * Set the contact link number (0 is the first link)
     * @param _linkNum the link number
     */
    virtual void setLinkNumber(unsigned int _linkNum);
    /**
     * Set the body part of this contact
     * @param _bodyPart the contact body part
     */
    virtual void setBodyPart(BodyPart _bodyPart);
    /**
     * Set the name of the link on which the contact is applied.
     */
    virtual void setLinkName(const std::string & _linkName);
    /**
     * Set the name of the frame in which the quantities of this contact are expressed.
     */
    virtual void setFrameName(const std::string & _linkName);

    //~~~~~~~~~~~~~~~~~~~~~~
    //   FIX/UNFIX methods
    //~~~~~~~~~~~~~~~~~~~~~~ 
    /**
    * Fix the direction of the contact force. Differently from the method
    * setForceDirection, this method also sets the flag 'fDirKnown'
    * to true, so when estimating the contact wrenches the solver considers the
    * force direction of this contact as known a-priori.
    * @param _Fdir the contact force direction
    * @return true iff operation succeeded, false otherwise
    */
    virtual bool fixForceDirection(const yarp::sig::Vector &_Fdir);
    /**
    * Equivalent to calling fixMoment(zeros(3)).
    */
    virtual bool fixMoment();
    /**
    * Fix the contact moment. Differently from the method
    * setMoment, this method also sets the flag 'muKnown'
    * to true, so when estimating the contact wrenches the solver considers the
    * moment of this contact as known a-priori.
    * @param _Mu the contact moment
    * @return true iff operation succeeded, false otherwise
    */
    virtual bool fixMoment(const yarp::sig::Vector &_Mu);
    /**
     * Set the flag fDirKnown to false so that when estimating the contact wrenches 
     * the solver estimates also the force direction and does not consider it as known
     * a-priori.
     */
    virtual void unfixForceDirection();
    /**
     * Set the flag muKnown to false so that when estimating the contact wrenches 
     * the solver estimates also the contact moment and does not consider it as known
     * a-priori.
     */
    virtual void unfixMoment();

    //~~~~~~~~~~~~~~~~~~~~~~
    //   SERIALIZATION methods
    //~~~~~~~~~~~~~~~~~~~~~~ 
    /*
    * Read dynContact from a connection. It expects a list of 5 elements, that are:
    * - a list of 3 int, containing contactId, bodyPart and linkNumber
    * - a list of 3 double, containing the CoP
    * - a list of 3 double, containing the force
    * - a list of 3 double, containing the moment
    * - a list of 2 strings, containing linkName, frameName
    * @param connection the connection to read from
    * @return true iff a dynContact was read correctly
    */
    virtual bool read(yarp::os::ConnectionReader& connection);
    /**
    * Write dynContact to a connection as a list of 5 elements, that are:
    * - a list of 3 int, containing contactId, bodyPart, linkNumber
    * - a list of 3 double, containing the CoP
    * - a list of 3 double, containing the force
    * - a list of 3 double, containing the moment
    * - a list of 2 strings, containing linkName, frameName
    * @param connection the connection to write to
    * @return true iff the dynContact was written correctly
    */
    virtual bool write(yarp::os::ConnectionWriter& connection);

    
    /**
     * Convert this contact into a string. Useful to print some information.
     * @param precision number of decimal digits to use in the string representation
     * @return a string representation of this contact in this format:
     * "Contact id: "<< contactId<< ", link name: "<< linkName << ", frame name: " << frameName <<
     * ", Body part: "<< bodyPartName<< ", link index: "<< linkNumber<<
     * ", CoP: "<< CoP<< ", F: "<< F<< ", M: "<< Mu
     */
    virtual std::string toString(int precision=-1) const;
    /**
    * Set the verbosity level of comments during operations
    * @param verb level of verbosity
    */
    virtual void setVerbose(unsigned int verb = VERBOSE);
};

}

}
#endif


