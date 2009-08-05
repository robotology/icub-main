/*
Copyright (C) 2003-2004  Etienne Lachance

This library is free software; you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation; either version 2.1 of the
License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


Report problems and direct all questions to:

email: etienne.lachance@polytml.ca or richard.gourdeau@polymtl.ca
-------------------------------------------------------------------------------
Revision_history:

2004/07/01: Etienne Lachance
   -Added doxygen documentation.

2004/07/01: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace
    -Added dependance on utils.h because we need to get the use_namespace setting

2004/07/13: Ethan Tira-Thompson
    -Added a select_real and add_real function for type indepence of Real
    -Added functions to test for sections and parameters existance

2004/07/29: Etienne Lachance
   -Added clear function. Suggested by Sylvain Marleau.

2004/08/10: Etienne Lachance
   -Removed select_real and add_real functions in order to make config.h/cpp 
    independent of ROBOOP.
   -Removed using ROBOOP namespace

2004/08/14: Etienne Lachance
   -Merge all select_* and add_* functions into select() and add() functions.

2006/02/04: Etienne Lachance
   -Member functions add and select are now in template form.
-------------------------------------------------------------------------------
*/

#ifndef CONFIG_H
#define CONFIG_H

/*!
  @file config.h
  @brief Header file for Config class definitions.
*/

//! @brief RCS/CVS version.
static const char header_config_rcsid[] = "$Id: config.h,v 1.1 2007/07/24 16:03:09 amaldo Exp $";


#ifdef _MSC_VER                  // Microsoft
#pragma warning (disable:4786)  // Disable decorated name truncation warnings 
#pragma warning (disable:4503)  // Disable decorated name truncation warnings 
#endif
#include <iostream>
#include <string>
#include <iomanip>
#include <fstream>

#include <boost/lexical_cast.hpp>

#include <sstream>
#include <vector>


//! @brief Return when can not open file.
#define CAN_NOT_OPEN_FILE                     -1

//! @brief Return when can not create a file.
#define CAN_NOT_CREATE_FILE                   -2


//! @brief Basic data element used in Config class.
typedef struct Data{
   std::string section;
   std::string parameter;
   std::string value;
} Data;

//! @brief Configuration data type.
typedef std::vector< Data > Conf_data;

//! @brief Handle configuration files.
class Config {

public:
    Config(const bool bPrintErrorMessages = true);
    short read_conf(std::ifstream & inconffile);
    void clear();
    void print();

    bool section_exists(const std::string & section) const;
    bool parameter_exists(const std::string & section, const std::string & parameter) const;


    template<typename T> bool select(const std::string & section, const std::string & parameter,
                                     T & value) const
    /*!
        @brief Get a parameter data, of a certain section, into the string value.
        @return false if the data can not be found and true otherwise.
    */
    {
        for(Conf_data::const_iterator iter = conf.begin(); iter != conf.end(); ++iter)
        {
            if( (iter->section == section) && (iter->parameter == parameter) )
            {
                try
                {
                    value = boost::lexical_cast<T>(iter->value);
                }
                catch (boost::bad_lexical_cast & e)
                {
                    return false;
                }
                return true;
            }
        }
        return false;
    }

    short write_conf(std::ofstream & outconffile, const std::string & file_title,
                     const int space_between_column);

    template <typename T> bool add(const std::string & section, const std::string & parameter, 
                                   const T & value)
    /*!
        @brief Added the value(string) of the parameter in the section in the 
               configuration data.
        The functions will added the parameter and the section if it does not already exist.
     */
    {
        Data dataSet;
        dataSet.section = section;
        dataSet.parameter = parameter;
        try
        {
            dataSet.value = boost::lexical_cast<std::string>(value);
        }
        catch (boost::bad_lexical_cast & e)
        {
            return false;
        }

        for(Conf_data::iterator iterConf = conf.begin(); iterConf != conf.end(); ++iterConf)
        {
            if(section == iterConf->section) // section already exist
            {
                if(parameter == iterConf->parameter) // parameter already exist
                {
                    try
                    {
                        iterConf->value = boost::lexical_cast<std::string>(value);
                    }
                    catch (boost::bad_lexical_cast & e)
                    {
                        return false;
                    }
                }
                // parameter does not exist
                for(Conf_data::iterator iterConf2 = iterConf; iterConf2 != conf.end(); ++iterConf2)
                {
                    if(section != iterConf2->section)
                    {
                        conf.insert(iterConf2, dataSet);
                        return true;
                    }
                }
            }
        }
        // section and parameter does not exist.
        conf.push_back(dataSet);
        return true;
    }

private:
   Conf_data conf;            //!< Data store from/to configuration file.
   bool bPrintErrorMessages;  //!< Print error messages on stderr. 
};

#endif 

