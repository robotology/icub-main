/**
 * Copyright (C) 2015 iCub Facility, Istituto Italiano di Tecnologia
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@yale.edu
 * Contribution: Matej Hoffmann, matej.hoffmann@iit.it
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

#include "iCub/skinDynLib/Taxel.h"
#include "iCub/skinDynLib/common.h"

#include <yarp/os/RFModule.h>

#include <fstream>
#include <vector>
#include <map>
#include <list>
#include <mutex>

namespace iCub
{
namespace skinDynLib
{

class skinPartBase
{
  public:
    std::string name;
    int         size;   // theoretical maximum size of the skinPart
                        // it corresponds to the number of values on the respective port 
                        // and number of rows in the taxel positions .txt files in
                        // icub-main/app/skinGui/conf/positions in the [calibration] group 
                        // IMPORTANT: it may differ from taxels.size()
    std::string version; // Version of the skin part - "V1", "V2", "V2.1" (or "unknown_version"). 
                         // Depending on the physical version of skin on the robot. For example,
                         // forearm V2 has one triangle more compared to V1 and also partly different taxel IDs. 
    std::recursive_mutex recursive_mtx;
  public:
    /**
    * Constructor
    **/    
    skinPartBase();
    
    /**
     * Copy constructor
     * @param _spb is the skinPart to copy from
     */
    skinPartBase(const skinPartBase &_spb);

    /**
    * Copy Operator
    * @param _spb is the skinPart to copy from
    **/
    virtual skinPartBase &operator=(const skinPartBase &_sp);

    /**
     * Sets the name of the class
     * @param _name new name of the class
     */
    void setName(const std::string &_name);

    /**
     * Gets the name of the class
     * @return string containing the name of the class
     */
    std::string getName();

    /**
     * Sets the size of the class
     * @param _size new size of the class
     */
    void setSize(int _size);

    /**
     * Gets the size of the class
     * @return int containing the size of the class
     */
    int getSize();

    /**
     * Sets the version ("V1" / "V2" / "V2.1")
     * @param _version 
     */
    void setVersion(const std::string &_version);

    /**
     * Gets the version
     * @return string containing the version
     */
    std::string getVersion();
    
    /**
     * Populates the skinPartBase by reading from a file.
     * @param  _filePath   is the full absolute path of the file
     * @return true/false in case of success/failure
     */
    virtual bool setTaxelPosesFromFile(const std::string &_filePath, const std::string &_spatial_sampling="default") { return false; }

    /**
    * Print Method
    * @param verbosity is the verbosity level
    **/
    virtual void print(int verbosity=0);

    /**
    * toString Method
    * @param verbosity is the verbosity level
    **/
    virtual std::string toString(int precision=0);
};

/** 
* @ingroup skinDynLib 
*
* Class that encloses everything relate to a skinPart.
* It consists of a std::vector of Taxel(s), and a number of methods for loading and populating these taxels from files.
* 
*/
class skinPart : public skinPartBase
{
  public:
    /**
    * List of taxels that belong to the skinPart.
    **/
    std::vector<Taxel*> taxels;
      
    /**
     * Spatial_sampling used in building up the skinPart class. 
     * It can be either full, i.e. "taxel", or "triangle", where the latter remaps the individual taxels onto the center taxels
     * of their respective triangular module (composed of 10 taxels, but 12 values on the port - 2 are thermal pads)
     */
    std::string spatial_sampling;

    /**
    * Indexing variable used in the case of reducing the resolution - e.g. taking only triangle centers
    * The index into the vector is the taxel ID, the value stored is its representative
    **/
    std::vector<int> taxel2Repr; 

    /**
    * Mapping in the opposite direction
    * Indexed by representative taxel IDs, it stores lists of the taxels being represented - e.g. all taxels of a triangle
    **/
    std::map<int, std::list<unsigned int> > repr2TaxelList;

  protected:
    /**
     * Populates the skinPart by reading from a file - old convention.
     * Spatial Sampling will be forced to "taxel"
     * @param  _filePath   is the full absolute path of the file
     * @return true/false in case of success/failure
     */
    bool setTaxelPosesFromFileOld(const std::string &_filePath);

    /**
     * Maps the taxels onto themselves, performing a 1:1 mapping. This has been
     * made in order for the "taxel" spatial_sampling modality to be consistent
     * with the "triangle" sampling, thus maintaining integrity into taxel2Repr and repr2TaxelList
     * @return true/false in case of success/failure
     */
    bool mapTaxelsOntoThemselves();

  public:
    /**
     * Constructor
     */
    skinPart();

    /**
     * Constructor with the configuration file to load from
     * @param _filePath is the absolute path of the file to read
     */
    skinPart(const std::string &_filePath);

    /**
     * Copy constructor
     * @param _sp is the skinPart to copy from
     */
    skinPart(const skinPart &_sp);

    /**
    * Copy Operator
    * @param _sp is the skinPart to copy from
    **/
    skinPart &operator=(const skinPart &_sp);

    /**
     * Populates the skinPart by reading from a file.
     * @param  _filePath   is the full absolute path of the file
     * @param  _spatial_sampling is the type of spatial sampling to perform (default "default")
     *                           if "default", the spatial_sampling will be read from file
     *                           if "taxel" or "triangle", this will be the spatial_sampling performed 
     * @return true/false in case of success/failure
     */
    bool setTaxelPosesFromFile(const std::string &_filePath,
                               const std::string &_spatial_sampling="default");

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
    int getTaxelsSize();

    /**
     * Clears the vector of taxels properly and gracefully.
     * WARNING: it deletes the pointed objects as well!
     */
    void clearTaxels();

    /**
    * Print Method
    * @param verbosity is the verbosity level
    **/
    void print(int verbosity=0);

    /**
    * toString Method
    * @param verbosity is the verbosity level
    **/
    std::string toString(int precision=0);

    /**
    * Destructor
    **/
    ~skinPart();
};

}

}//end namespace

#endif

// empty line to make gcc happy
