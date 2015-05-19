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

#include <fstream>
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

class skinPartBase
{
  protected:
    SkinPart name;
    int size;       // theoretical maximum size of the skinPart
                    // it corresponds to the number of values on the respective port 
                    // and number of rows in the .txt files in icub-main/app/skinGui/conf/positions
                    // IMPORTANT: it may differ from txls.size()
  public:
    /**
    * Constructor
    **/    
    skinPartBase();

    /**
    * Constructor with the name
    **/    
    skinPartBase(const SkinPart &_name);

    /**
    * Copy Operator
    **/
    virtual skinPartBase &operator=(const skinPartBase &spw);

    void setName(const SkinPart &_name);
    SkinPart getName();
    int getSize();

    /**
     * Populates the skinPartBase by reading from a file.
     * @param  filePath is the full absolute path of the file
     * @return          true/false in case of success/failure
     */
    virtual bool setTaxelPosesFromFile(const string &filePath, const string &modality="full");

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
class skinPartTaxel : public skinPartBase
{
  public:
    /**
    * List of taxels that belong to the skinPart.
    **/
    vector<Taxel*> taxels;

    /**
     * Modality used in building up the skinPartTaxel class. 
     * It can be either "full" or "mapping", the latter of which remaps the taxels onto the center
     * of their respective triangular patch
     */
    string modality;

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
    * Copy Operator
    **/
    skinPartTaxel &operator=(const skinPartTaxel &spw);

    /**
     * Constructor
     */
    skinPartTaxel();

    /**
     * Constructor that specifies the type of modality
     * @param _modality the modality to be used
     */
    skinPartTaxel(const string &_modality);

    /**
     * Populates the skinPart by reading from a file.
     * @param  filePath is the full absolute path of the file
     * @param  modality is the type of reading to perform.
     *                  if "default", it keeps the modality already stored in the class
     *                  if "full", it reads every single taxel in the file
     *                  if "mapping", it maps the taxels into the center of their triangular patch
     * @return          true/false in case of success/failure
     */
    bool setTaxelPosesFromFile(const string &filePath, const string &_modality="default");

    /**
     * Initializes the mapping between the taxels and their representatives
     * (i.e. the centers of the triangles)
     * @return true/false in case of success/failure
     */
    bool initRepresentativeTaxels();

    /**
     * gets the size of the taxel vector (it differs from skinPartBase::getSize())
     * @return the size of the taxel vector
     */
    int getTaxelSize();

    /**
    * Print Method
    **/
    void print(int verbosity=0);

    /**
    * toString Method
    **/
    string toString(int precision=0);

    /**
    * Destructor
    **/
    ~skinPartTaxel();
};

}

}//end namespace

#endif

// empty line to make gcc happy
