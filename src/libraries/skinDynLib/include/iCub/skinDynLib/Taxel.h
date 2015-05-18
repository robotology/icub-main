/**
 * Copyright (C) 2015 iCub Facility, Istituto Italiano di Tecnologia
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 *
 *
 * This file contains the definition of a Taxel, i.e. the atomic element the iCub skin is composed of.
 * It is empowered by a Position and a Normal (both relative to the body part it belong to), and some more
 * useful members.
 *
 * \section intro_sec Description
 * 
 * \section tested_os_sec Tested OS
 * 
 * Windows, Linux
 *
 * \author Alessandro Roncone
 * 
 **/

#ifndef __TAXEL_H__
#define __TAXEL_H__

#include "iCub/skinDynLib/skinContact.h"
#include "iCub/skinDynLib/utils.h"
#include "iCub/skinDynLib/parzenWindowEstimator.h"

#include <sstream>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace iCub::skinDynLib;

using namespace std;

namespace iCub
{
namespace skinDynLib
{

/** 
* @ingroup skinDynLib 
*
* Class that encloses everything relate to a Taxel, i.e. the atomic element the iCub skin is composed of.
* It is empowered by a Position and a Normal (both relative to the body part it belong to), and some more
* useful members (such as its 2D coordinates into the image frame of one of the eyes, its Frame of Reference
* base on the Position and Normal members, its position into the World Reference Frame)
*/
class Taxel
{
  public:
    int ID;                    // taxels' ID
    yarp::sig::Vector px;      // (u,v) projection in the image plane
    yarp::sig::Vector Pos;     // taxel's position w.r.t. the limb
    yarp::sig::Vector WRFPos;  // taxel's position w.r.t. the root FoR
    yarp::sig::Vector Norm;    // taxel's normal   w.r.t. the limb
    yarp::sig::Matrix FoR;     // taxel's reference Frame (computed from Pos and Norm)

    /**
    * Constructors
    **/    
    Taxel();
    Taxel(const Vector &p, const Vector &n);
    Taxel(const Vector &p, const Vector &n, const int &i);

    /**
    * Copy Operator
    **/
    Taxel &operator=(const Taxel &t);

    /**
    * init function
    **/
    void init();

    /**
    * Compute and set the taxel's reference frame (from its position and its normal vector)
    **/
    void setFoR();

    /**
    * Print Method
    **/
    virtual void print(int verbosity=0) {};

    /**
    * toString Method
    **/
    virtual string toString(int precision=0) {};

    /**
    * Resets the parzen window estimator
    **/
    virtual bool resetParzenWindowEstimator() {};

    /**
    * Computes the response of the taxel.
    **/
    virtual bool computeResponse() {};
};

/**
 * @ingroup skinDynLib 
 * 
 * Base wrapper for the Parzen Window Estimator(PWE). This class inherits from Taxel
 * and is provided with a PWE in order to compute a visuo-tactile estimation of its
 * local peripersonal space.
 */
class TaxelPWE : public Taxel
{
  public:
    int       Resp;                 // taxels' activation level (0-255)
    double RFangle;                 // angle of the receptive field [rad]

    IncomingEvent4TaxelPWE Evnt;    // 
    parzenWindowEstimator *pwe;     // 

    /**
    * Constructors
    **/    
    TaxelPWE();
    TaxelPWE(const Vector &p, const Vector &n);
    TaxelPWE(const Vector &p, const Vector &n, const int &i);

    /*
    * Destructor
    **/
    ~TaxelPWE();

    /**
    * init function
    **/
    void init() { Taxel::init(); };

    /**
    * Add or remove a sample from the pwe's histogram
    **/
    bool    addSample(IncomingEvent4TaxelPWE ie);
    bool removeSample(IncomingEvent4TaxelPWE ie);

    /**
    * Check if the input sample is inside the Receptive field (i.e. the cone)
    **/
    bool insideFoRCheck(const IncomingEvent4TaxelPWE ie);

    /**
    * Print Method
    **/
    void print(int verbosity=0);

    /**
    * toString Method
    **/
    string toString(int precision=0);

    /**
    * Resets the parzen window estimator
    **/
    bool resetParzenWindowEstimator();

    /**
    * Computes the response of the taxel.
    **/
    bool computeResponse();

    /**
    * Convert the taxel into a bottle in order to be saved on file
    **/
    Bottle TaxelPWEIntoBottle();
};

/**
 * @ingroup skinDynLib 
 * 
 * Class used in the case of one-dimensional Parzen Window Estimation
 * 
 */
class TaxelPWE1D : public TaxelPWE
{
  public:

    /**
    * Constructors
    **/    
    TaxelPWE1D() : TaxelPWE()                                                    { pwe = new parzenWindowEstimator1D();};
    TaxelPWE1D(const Vector &p, const Vector &n) : TaxelPWE(p,n)                 { pwe = new parzenWindowEstimator1D();};
    TaxelPWE1D(const Vector &p, const Vector &n, const int &i) : TaxelPWE(p,n,i) { pwe = new parzenWindowEstimator1D();};

    /**
    * init function
    **/
    void init() { TaxelPWE::init(); };
};

/**
 * @ingroup skinDynLib 
 * 
 * Class used in the case of two-dimensional Parzen Window Estimation
 * 
 */
class TaxelPWE2D : public TaxelPWE
{
  public:

    /**
    * Constructors
    **/    
    TaxelPWE2D() : TaxelPWE()                                                    { pwe = new parzenWindowEstimator2D();};
    TaxelPWE2D(const Vector &p, const Vector &n) : TaxelPWE(p,n)                 { pwe = new parzenWindowEstimator2D();};
    TaxelPWE2D(const Vector &p, const Vector &n, const int &i) : TaxelPWE(p,n,i) { pwe = new parzenWindowEstimator2D();};

    /**
    * init function
    **/
    void init() { TaxelPWE::init(); };
};

}

}//end namespace

#endif

// empty line to make gcc happy
