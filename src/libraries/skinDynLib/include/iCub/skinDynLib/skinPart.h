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
 * This file contains the definition of a skinPart, i.e. a class that collects multiple taxels..
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

#ifndef __SKINPART_H__
#define __SKINPART_H__

#include "iCub/skinDynLib/skinContact.h"
#include "iCub/skinDynLib/Taxel.h"
#include "iCub/skinDynLib/utils.h"
#include "iCub/skinDynLib/parzenWindowEstimator.h"

#include <vector>
#include <map>
#include <list>

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
* Class that encloses all the information related to a skinpart.
**/
class skinPart
{
  public:
    SkinPart name;
    int size;   // theoretical maximum size of the skinPart if the patches were full ~ corresponds to number of values on the respective port 
    //and number of columns in the .txt files in icub-main/app/skinGui/conf/positions
    //IMPORTANT: it differs from taxel.size(), that is the real size of the skinPart with "active" or valid taxels
             
    /**
    * Indexing variable used in the case of reducing the resolution - e.g. taking only triangle centers
    * The index into the vector is the taxel ID, the value stored is its representative
    **/
    vector<int> Taxel2Repr; 

    /**
    * Mapping in the opposite direction
    * Indexed by representative taxel IDs, it stores lists of the taxels being represented - e.g. all taxels of a triangle
    **/
    map<unsigned int, list<unsigned int> > Repr2TaxelList;

    /**
    * Constructor
    **/    
    skinPart();
    // skinPart(const string _name);

    /**
    * Copy Operator
    **/
    virtual skinPart &operator=(const skinPart &spw);

    /**
    * Print Method
    **/
    virtual void print(int verbosity=0);

    /**
    * toString Method
    **/
    virtual string toString(int precision=0);
};

class skinPartTaxel : public skinPart
{
  public:
    /**
    * List of taxels that belong to the skinPart.
    **/
    vector<Taxel*> txls;

    /**
    * Destructor
    **/
    ~skinPartTaxel();

    /**
    * Copy Operator
    **/
    skinPartTaxel &operator=(const skinPartTaxel &spw);

    /**
    * Print Method
    **/
    void print(int verbosity=0);

    /**
    * toString Method
    **/
    string toString(int precision=0);
};

class skinPartPWE : public skinPart
{
  public:
    /**
    * Modality (either 1D or 2D)
    */
    string modality;

    /**
    * List of taxelsPWE that belong to the skinPart (either 1D or 2D)
    **/
    vector<TaxelPWE*> txls;

    /*
    * Constructor that assigns modality member
    **/
    skinPartPWE(const string &_modality) : skinPart(), modality(_modality) { txls.clear(); };

    /**
    * Destructor
    **/
    ~skinPartPWE();

    /**
    * Copy Operator
    **/
    skinPartPWE &operator=(const skinPartPWE &spw);

    /**
    * Print Method
    **/
    void print(int verbosity=0);

    /**
    * toString Method
    **/
    string toString(int precision=0);

    int get_taxelSize() { return txls.size(); };
};

}

}//end namespace

#endif

// empty line to make gcc happy
