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

2004/06/10: Etienne Lachance
    -Added doxygen documentation.

2004/07/01: Ethan Tira-Thompson
    -Added support for newmat's use_namespace #define, using ROBOOP namespace

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

2004/08/20: Etienne Lachance
   -Parameter value can now contain space.
   -Fix print member function.
-------------------------------------------------------------------------------
*/

/*! 
  @file config.cpp
  @brief Configuration class functions.
*/

//! @brief RCS/CVS version.
static const char rcsid[] = "$Id: config.cpp,v 1.1 2007/07/24 16:03:09 amaldo Exp $";


#include "config.h"

using namespace std;

Config::Config(const bool bPrintErrorMessages_) 
//! @brief Constructor.
{
   bPrintErrorMessages = bPrintErrorMessages_;
}

short Config::read_conf(ifstream & inconffile)
/*!
  @brief Read a configuration file.

  This function reads the configuration file specified in 
  the constructor parameter. The information is stored in the
  variable conf.

  A configuration file contains section(s) (between [ ]), and the 
  section(s) contains parameter(s) with there respective value(s). 
  The section and the parameter are always access via a string. Below 
  is an exemple: one section named PUMA560_mDH, and two parameters.

  [PUMA560_mDH]
  DH:         0
  dof:        6
*/
{
   //std::ifstream inconffile(filename.c_str(), std::ios::in);

   if(inconffile)
   {
      string temp;
      unsigned int tmpPos;
      Data data;
      getline(inconffile, temp);

      while( !inconffile.eof() )
      {
	 // Is-it comment line?
         if(temp.substr(0,1) != "#")
         {
	    // Is-it a section name?
            if(temp.substr(0,1) == "[") // [section]
            {
	       // Search for the end of the section name and ignore the rest of the line.
 	       tmpPos = temp.find("]");
	       if (tmpPos != string::npos)
		 {
		   data.section = temp.substr(1, tmpPos-1); // remove []
		   // Read the next line, it's should be a parameter.
		   getline(inconffile, temp);
		   // Be sure that is not another section name.
		   while( (temp.substr(0,1) != "[") &&
			  (!inconffile.eof()) )
		     {
		       if(temp.substr(0,1) != "#") // ignore comments
			 {
			   if(temp.find(":") != string::npos)
			     {
			       istringstream inputString(temp);
			       inputString >> data.parameter >> data.value;
			       string tmp_value;
			       while(inputString >> tmp_value)
			       {
				   data.value.append(" ");
				   data.value.append(tmp_value);
			       }
			       // Find ":" in parameter.
			       tmpPos = data.parameter.find(":");
			       if (tmpPos != string::npos)
				 // remove ":" a the end of parameter
				 data.parameter = data.parameter.substr(0, tmpPos);
			       else
				 {
				   inputString >> data.value;
				   string tmp_value;
				   while(inputString >> tmp_value)
				   {
				       data.value.append(" ");
				       data.value.append(tmp_value);
				   }
				 }

			       // Add data to the config vector
			       conf.push_back(data);
			     }

			   // Read the next line.
			   getline(inconffile, temp);
			 }
		       else
			 {
			   // Ignore comments and read the next line.
			   getline(inconffile, temp);
			 }
		     }
		 }
            }
         }
         if(temp.substr(0,1) != "[") {
            getline(inconffile, temp);
         }
      }
   }
   else
   {
      if (bPrintErrorMessages)
      {
          cerr << "Config::read_conf: invalid input ifstream " << endl;
      }
      return CAN_NOT_OPEN_FILE;
   }
   return 0;
}

void Config::clear()
//! @brief Clear the data buffer.
{
    conf.clear();
}

void Config::print()
//! @brief Print the configuration data.
{
  string tmpSection;
  for(Conf_data::iterator iter = conf.begin(); iter != conf.end(); ++iter)
    {
      if (tmpSection != iter->section)
	{
	  //Beginning of a section
	  tmpSection = iter->section;
	  cout << "\n[" << tmpSection << "]" << endl;
	  cout << iter->parameter+":" << " " << iter->value << endl;
	}
      else
	  cout << iter->parameter+":" << " " << iter->value << endl;
    }
}

bool Config::section_exists(const string& section) const
/*!
  @brief Test to see if a section exists
  @return true if @a section is found
*/
{
	for(Conf_data::const_iterator iter = conf.begin(); iter != conf.end(); ++iter)
		if(iter->section == section)
			return true;
	return false;
}

bool Config::parameter_exists(const string& section, const string& parameter) const
/*!
  @brief Test to see if a parameter exists within a section
  @return true if @a parameter is found within @a section
*/
{
	for(Conf_data::const_iterator iter = conf.begin(); iter != conf.end(); ++iter)
		if( (iter->section == section) && (iter->parameter == parameter) )
			return true;
	return false;
}

short Config::write_conf(ofstream & outconffile, const string & file_title,
                         const int space_between_column)
/*!
  @brief Write the configuration information, contained in conf, on disk.
  @param filename: Configuration file name.
  @param file_title: Title in the configuration file header.
  @param space_between_column: Number of blanks between : (of a parameter) and it's value.
*/
{
   if(outconffile)
   {                     // file header
      outconffile << "# ---------------------------------------------------" << endl;
      outconffile << "# " << file_title << endl;
      outconffile << "# ---------------------------------------------------" << endl;
      outconffile << endl;

      string section = "";

      for(Conf_data::iterator iterConf = conf.begin(); iterConf != conf.end(); ++iterConf)
      {
         if(section != iterConf->section)
         {
            section = iterConf->section;
            outconffile << "\n[" << section << "]\n" << endl;
         }
	 outconffile << setw(space_between_column-iterConf->parameter.size()) 
		     << iterConf->parameter + ":" << " " << iterConf->value << endl;
      }
      return 0;
   }
   else
   {
      if (bPrintErrorMessages)
      {
          cerr << "Config::write_conf: invalid input ofstream " << endl;
      }
      return CAN_NOT_CREATE_FILE;
   }
}

