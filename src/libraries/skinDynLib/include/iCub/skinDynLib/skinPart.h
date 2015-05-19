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

class skinPart
{
  public:
    SkinPart name;
    int size;       // theoretical maximum size of the skinPart
                    // it corresponds to the number of values on the respective port 
                    // and number of rows in the .txt files in icub-main/app/skinGui/conf/positions
                    // IMPORTANT: it may differ from txls.size()
             
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

/** 
* @ingroup skinDynLib 
*
* Class that encloses everything relate to a skinPart.
* It consists of a std::vector of Taxel(s), and a number of methods for loading and populating these taxels from files.
* 
*/
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

    int get_taxelSize() { return txls.size(); };
};

}

}//end namespace

#endif

// empty line to make gcc happy
