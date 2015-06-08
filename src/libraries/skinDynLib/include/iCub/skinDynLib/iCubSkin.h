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
 * This file contains the definition of the iCubSkin, i.e. a class that collects multiple skinPart(s).
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

#ifndef __ICUBSKIN_H__
#define __ICUBSKIN_H__

#include "iCub/skinDynLib/skinPart.h"

#include <vector>

namespace iCub
{
namespace skinDynLib
{

/** 
* @ingroup skinDynLib 
*
* Class that collects a set of skinParts.
* It consists of a std::vector of skinPart(s), and a number of methods for loading and populating these taxels from files.
* 
*/    
class iCubSkin
{
  protected:
    std::vector<skinPart> skin;

  public:
    /**
     * Default constructor
     */
    iCubSkin();

    /**
     * Constructor that configures the class with an .ini file.
     * The context used will be the default one, i.e. 'skinGui'
     * @param _from     the name of the file (default skinManAll.ini)
     */
    iCubSkin(const std::string &_from);

    /**
     * Constructor that configures the class with an .ini file
     * @param _from     the name of the file (default skinManAll.ini)
     * @param _context  the context to load the file from (default skinGui)
     */
    iCubSkin(const std::string &_from, const std::string &_context);

    /**
     * Configures the class with an .ini file
     * @param _from     the name of the file (default skinManAll.ini)
     * @param _context  the context to load the file from (default skinGui)
     * @return true/false in case of success/failure
     */
    bool configureSkinFromFile(const std::string &_from="skinManAll.ini",
                               const std::string &_context="skinGui");

    /**
    * Print Method
    **/
    void print(int verbosity=0);
};

}

}//end namespace

#endif

// empty line to make gcc happy

