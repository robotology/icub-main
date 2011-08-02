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
 * Windows
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
#include <iostream>
#include <iomanip>

using namespace yarp::sig;
using namespace std;


namespace iCub
{

namespace iDyn
{

/**
* Class representing an external contact acting on a link.
*
* The link number and the contact position are supposed to be known.
* The force direction and the moment may be either known or unknown.
*/
class iDynContact
{
protected:
    /// id of this contact
    int ID;

    /// number of the link where the contact is applied
    unsigned int linkNumber;
	/// position of the contact wrt the reference frame of the link
	Vector pos;
    ///contact force direction (unit vector)
	Vector Fdir;
    /// contact force module
    double Fmodule;
	/// contact moment
	Vector Mu;

    /// True if the moment applied at the contact point is known
    bool muKnown;
    /// True if the direction of the force applied at the contact point is known
    bool fDirKnown;

	///verbosity flag
	unsigned int verbose;


    void init(unsigned int _linkNumber, const Vector &_pos);
    bool checkVectorDim(const Vector &v, unsigned int dim, const string &descr="");
    double norm(const Vector &v);

public:
    //~~~~~~~~~~~~~~~~~~~~~~
	//   CONSTRUCTORS
	//~~~~~~~~~~~~~~~~~~~~~~
    /**
    * Default constructor
    */
    iDynContact(unsigned int _linkNumber, const Vector &_pos);
    /**
    * Constructor with known moment (usually zero)
    */
    iDynContact(unsigned int _linkNumber, const Vector &_pos, const Vector &_Mu);
    /**
    * Constructor with known moment (usually zero) and known force direction
    */
    iDynContact(unsigned int _linkNumber, const Vector &_pos, const Vector &_Mu, const Vector &_Fdir);    

    //~~~~~~~~~~~~~~~~~~~~~~
	//   GET methods
	//~~~~~~~~~~~~~~~~~~~~~~
    /**
     * Get the contact force and moment in a single (6x1) vector
	 * @return a 6x1 vector where 0:2=force 3:5=moment
     */
	Vector          getForceMoment()        const;
    Vector	        getForce()		        const;
    Vector	        getForceDirection()	    const;
    double	        getForceModule()	    const;
	Vector	        getMoment()		        const;
	Vector	        getPosition()           const;
    unsigned int    getLinkNumber()         const;
 	
    //~~~~~~~~~~~~~~~~~~~~~~
	//   IS methods
	//~~~~~~~~~~~~~~~~~~~~~~
    bool isMomentKnown()                const;
    bool isForceDirectionKnown()        const;
   
    //~~~~~~~~~~~~~~~~~~~~~~
	//   SET methods
	//~~~~~~~~~~~~~~~~~~~~~~    
    bool setForce(const Vector &_F);
    bool setForceModule(double _Fmodule);
    bool setForceDirection(const Vector &_Fdir);
	bool setMoment(const Vector &_Mu);
    bool setPosition(const Vector &_pos);
    bool setLinkNumber(unsigned int _linkNum);

    //~~~~~~~~~~~~~~~~~~~~~~
	//   FIX/UNFIX methods
	//~~~~~~~~~~~~~~~~~~~~~~    
    bool fixForceDirection(const Vector &_Fdir);
	bool fixMoment();
    bool fixMoment(const Vector &_Mu);
    void unfixForceDirection();
    void unfixMoment();

    
    
    /**
     * Useful to print some information..
     */
	std::string toString() const;
    /**
	* Set the verbosity level of comments during operations
	* @param verb, a boolean flag
	*/
	 void setVerbose(unsigned int verb = VERBOSE);    
};





/**
* \ingroup iDynContact
*/
class iDynContactSolver : public iDynSensor
{
protected:

    /// list of contacts acting on the link chain
	deque<iDynContact> contactList;

    void findContactSubChain(unsigned int &firstLink, unsigned int &lastLink);
    Matrix crossProductMatrix(const Vector &v);
    
    Matrix buildA(unsigned int firstContactLink, unsigned int lastContactLink);
    Vector buildB(unsigned int firstContactLink, unsigned int lastContactLink);
    // print a matrix nicely
    void printMatrix(string s, Matrix &m)
    {
	    cout<<s<<endl;
	    for(int i=0;i<m.rows();i++)
	    {
		    for(int j=0;j<m.cols();j++)
			    cout<< setiosflags(ios::fixed)<< setprecision(3)<< setw(6)<<m(i,j)<<"\t";
		    cout<<endl;
	    }
    }

    // print a vector nicely
    void printVector(string s, Vector &v)
    {
	    cout<<s<<endl;
	    for(int j=0;j<v.length();j++)
		    cout<< setiosflags(ios::fixed)<< setprecision(3)<< setw(6)<<v(j)<<"\t";
	    cout<<endl;
    }

public:
	

    /**
     * Default constructor 
     */
    iDynContactSolver(iDynChain *_c, const string &_info="", const NewEulMode _mode=DYNAMIC, unsigned int verb=NO_VERBOSE);
    
    iDynContactSolver(iDynChain *_c, unsigned int sensLink, SensorLinkNewtonEuler *sensor, 
        const string &_info="", const NewEulMode _mode=DYNAMIC, unsigned int verb=NO_VERBOSE);
    /**
     * Constructor with F/T sensor information
     */
    iDynContactSolver(iDynChain *_c, unsigned int sensLink, const Matrix &_H, const Matrix &_HC, double _m, const Matrix &_I, 
        const string &_info="", const NewEulMode _mode=DYNAMIC, unsigned int verb=NO_VERBOSE);
    /**
     * Default destructor
     */
    ~iDynContactSolver();

    /**
     * Add a new element to the contact list. The content of this new element is initialized to a copy of "contact".
	 * @param contact
	 * @return true if the operation is successful, false otherwise (eg index out of range)
     */
	bool addContact(const iDynContact &contact);
    bool addContacts(const deque<iDynContact> &contacts);
    bool removeContact(const int contactId);
    void clearContactList();

    /**
     * Compute an estimate of the external contact wrenches.
     * @return A copy of the external contact list
     */
    deque<iDynContact> computeExternalContacts(const Vector &FMsens);

	deque<iDynContact> computeExternalContacts();

	void computeWrenchFromSensorNewtonEuler();

    //***************************************************************************************
    // GET METHODS
    //***************************************************************************************

    /**
     * @return A copy of the external contact list
     */
    deque<iDynContact> getContactList() const;
    unsigned int getUnknownNumber() const;

	//***************************************************************************************
    // UTILITY METHODS
    //***************************************************************************************

	/**
	 * Compute the rototraslation matrix from frame a to frame b.
	 */
	Matrix getHFromAtoB(unsigned int a, unsigned int b);

	/**
	 * Compute the wrench of the specified contact expressed w.r.t. the root reference
	 * frame of the chain (not the 0th frame, but the root).
	 */
	Vector projectContact2Root(const iDynContact &c);

};


}

}//end namespace

#endif


