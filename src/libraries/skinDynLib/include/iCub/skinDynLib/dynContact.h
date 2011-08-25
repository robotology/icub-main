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
* Class representing an external contact acting on a link.
*
* The link number and the contact position are supposed to be known.
* The force direction and the moment may be either known or unknown.
*/
class dynContact : public yarp::os::Portable
{
protected:
    /// id of this contact
    //int ID;

    /// part of the body of the robot where the contact is applied
    BodyPart bodyPart;
    /// number of the link where the contact is applied
    unsigned int linkNumber;
	/// center of pressure of the contact expressed w.r.t. the reference frame of the link
	yarp::sig::Vector CoP;
    ///contact force direction (unit vector)
	yarp::sig::Vector Fdir;
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


    void init(BodyPart _bodyPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, 
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
    */
    dynContact(BodyPart _bodyPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP);
    /**
    * Constructor with known moment (usually zero) and unknown force direction
    */
    dynContact(BodyPart _bodyPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, const yarp::sig::Vector &_Mu);
    /**
    * Constructor with known moment (usually zero) and known force direction
    */
    dynContact(BodyPart _bodyPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, const yarp::sig::Vector &_Mu, const yarp::sig::Vector &_Fdir);    

    //~~~~~~~~~~~~~~~~~~~~~~
	//   GET methods
	//~~~~~~~~~~~~~~~~~~~~~~
    /**
     * Get the contact force and moment in a single (6x1) vector
	 * @return a 6x1 vector where 0:2=force 3:5=moment
     */
	virtual yarp::sig::Vector           getForceMoment()        const;
    virtual yarp::sig::Vector	        getForce()		        const;
    virtual yarp::sig::Vector	        getForceDirection()	    const;
    virtual double	                    getForceModule()	    const;
	virtual yarp::sig::Vector	        getMoment()		        const;
	virtual yarp::sig::Vector	        getCoP()                const;
    virtual unsigned int                getLinkNumber()         const;
    virtual BodyPart                    getBodyPart()           const;
    virtual std::string                 getBodyPartName()       const;
 	
    //~~~~~~~~~~~~~~~~~~~~~~
	//   IS methods
	//~~~~~~~~~~~~~~~~~~~~~~
    virtual bool isMomentKnown()                const;
    virtual bool isForceDirectionKnown()        const;
   
    //~~~~~~~~~~~~~~~~~~~~~~
	//   SET methods
	//~~~~~~~~~~~~~~~~~~~~~~    
    virtual bool setForce(const yarp::sig::Vector &_F);
    virtual bool setForceModule(double _Fmodule);
    virtual bool setForceDirection(const yarp::sig::Vector &_Fdir);
	virtual bool setMoment(const yarp::sig::Vector &_Mu);
    virtual bool setCoP(const yarp::sig::Vector &_CoP);
    virtual void setLinkNumber(unsigned int _linkNum);
    virtual void setBodyPart(BodyPart _bodyPart);

    //~~~~~~~~~~~~~~~~~~~~~~
	//   FIX/UNFIX methods
	//~~~~~~~~~~~~~~~~~~~~~~    
    virtual bool fixForceDirection(const yarp::sig::Vector &_Fdir);
	virtual bool fixMoment();
    virtual bool fixMoment(const yarp::sig::Vector &_Mu);
    virtual void unfixForceDirection();
    virtual void unfixMoment();

    //~~~~~~~~~~~~~~~~~~~~~~
	//   SERIALIZATION methods
	//~~~~~~~~~~~~~~~~~~~~~~ 
    /*
    * Read dynContact from a connection.
    * return true iff a dynContact was read correctly
    */
    virtual bool read(yarp::os::ConnectionReader& connection);
    /**
    * Write dynContact to a connection.
    * return true iff a dynContact was written correctly
    */
    virtual bool write(yarp::os::ConnectionWriter& connection);

    
    
    /**
     * Useful to print some information.
     */
	virtual std::string toString();
    /**
	* Set the verbosity level of comments during operations
	*/
	virtual void setVerbose(unsigned int verb = VERBOSE);
};

}

}
#endif
