/////////////////////////////////////////////////////////////////////////
///                                                                   ///
///       YARP - Yet Another Robotic Platform (c) 2001-2004           ///
///                                                                   ///
///                    #Add our name(s) here#                         ///
///                                                                   ///
///     "Licensed under the Academic Free License Version 1.0"        ///
///                                                                   ///
/// The complete license description is contained in the              ///
/// licence.template file included in this distribution in            ///
/// $YARP_ROOT/conf. Please refer to this file for complete           ///
/// information about the licensing of YARP                           ///
///                                                                   ///
/// DISCLAIMERS: LICENSOR WARRANTS THAT THE COPYRIGHT IN AND TO THE   ///
/// SOFTWARE IS OWNED BY THE LICENSOR OR THAT THE SOFTWARE IS         ///
/// DISTRIBUTED BY LICENSOR UNDER A VALID CURRENT LICENSE. EXCEPT AS  ///
/// EXPRESSLY STATED IN THE IMMEDIATELY PRECEDING SENTENCE, THE       ///
/// SOFTWARE IS PROVIDED BY THE LICENSOR, CONTRIBUTORS AND COPYRIGHT  ///
/// OWNERS "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, ///
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   ///
/// FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO      ///
/// EVENT SHALL THE LICENSOR, CONTRIBUTORS OR COPYRIGHT OWNERS BE     ///
/// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN   ///
/// ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN ///
/// CONNECTION WITH THE SOFTWARE.                                     ///
///                                                                   ///
/////////////////////////////////////////////////////////////////////////

///
///
///       YARP - Yet Another Robotic Platform (c) 2001-2003 
///
///							#nat#
///
///	     "Licensed under the Academic Free License Version 1.0"
///
/// $Id: YARPConfigFile.h,v 1.1 2006/12/19 14:30:41 nat Exp $
///  
/// very simple class to handle config files... by nat May 2003
//

#ifndef __YARPCONFIGFILE__
#define __YARPCONFIGFILE__

/**
 * \file YARPConfigFile.h A simple class for reading initialization files.
 */

#include <stdio.h>

#include <string>

/**
 * A simple class for reading configuration files (similar in spirit to
 * Windows .ini files). The configuration file is built of sections 
 * identified by a name in square brackets and parameters followed by
 * '=' and a value or an array of values. The file is terminated by
 * [ENDINI] which is mandatory.
 *
 */
class YARPConfigFile
{
public:
	/**
	 * Constructor.
	 */
	YARPConfigFile() { _openFlag = false; }

	/**
	 * Constructor.
	 * Builds the object and provides a path and filename.
	 * @param path is the path.
	 * @param filename is the filename.
	 */
	YARPConfigFile(const std::string &filename)
	{
		_filename = filename;
		_openFlag = false;
	}
	
	/**
	 * Destructor.
	 * Closes also any open file.
	 */
	virtual ~YARPConfigFile(void) { _close(); };

	/**
	 * Gets a double precision number.
	 * @param section represents the section of the file.
	 * @param name is the parameter name.
	 * @param out is a pointer to the return values.
	 * @param n is the number of values to read.
	 * @return YARP_OK on success.
	 */
	int get(const char *section, const char *name, double *out, int n = 1);

	/**
	 * Gets integer(s).
	 * @param section represents the section of the file.
	 * @param name is the parameter name.
	 * @param out is a pointer to the return values.
	 * @param n is the number of values to read.
	 * @return YARP_OK on success.
	 */
	int get(const char *section, const char *name, int *out, int n = 1);

	/**
	 * Gets unsigned integer(s).
	 * @param section represents the section of the file.
	 * @param name is the parameter name.
	 * @param out is a pointer to the return values.
	 * @param n is the number of values to read.
	 * @return YARP_OK on success.
	 */
	int get(const char *section, const char *name, unsigned int *out, int n = 1);

	/**
	 * Gets short integer(s).
	 * @param section represents the section of the file.
	 * @param name is the parameter name.
	 * @param out is a pointer to the return values.
	 * @param n is the number of values to read.
	 * @return YARP_OK on success.
	 */
	int get(const char *section, const char *name, short *out, int n = 1);

	/**
	 * Gets chars as integer(s).
	 * @param section represents the section of the file.
	 * @param name is the parameter name.
	 * @param out is a pointer to the return values.
	 * @param n is the number of values to read.
	 * @return YARP_OK on success.
	 */
	int get(const char *section, const char *name, char *out, int n = 1);

	/**
	 * Gets a matrix of double precision numbers.
	 * @param section represents the section of the file.
	 * @param name is the parameter name.
	 * @param out is a pointer to the return values.
	 * @param n is the number of rows.
	 * @param m is the number of columns.
	 * @return YARP_OK on success.
	 */
	int get(const char *section, const char *name, double **out, int n, int m);

	/**
	 * Gets a char or an array as hex value(s).
	 * @param section represents the section of the file.
	 * @param name is the parameter name.
	 * @param out is a pointer to the return values.
	 * @param n is the number of rows.
	 * @return YARP_OK on success.
	 */
	int getHex(const char *section, const char *name, char *out, int n = 1);

	/**
	 * Gets a short integer or an array as hex value(s).
	 * @param section represents the section of the file.
	 * @param name is the parameter name.
	 * @param out is a pointer to the return values.
	 * @param n is the number of rows.
	 * @return YARP_OK on success.
	 */
	int getHex(const char *section, const char *name, short *out, int n = 1);

	/**
	 * Gets a string.
	 * @param section represents the section of the file.
	 * @param name is the parameter name.
	 * @param out is a pointer to the return values.
	 * @param n is the number of rows.
	 * @return YARP_OK on success.
	 */
	int getString(const char *section, const char *name, char *out);

	/**
	 * Gets a short integer or an array as hex value(s).
	 * @param section represents the section of the file.
	 * @param name is the parameter name.
	 * @param str is a reference to a YARPString containing the string if any.
	 * @param n is the number of rows.
	 * @return YARP_OK on success.
	 */
	int getString(const char *section, const char *name, std::string &str);

	/**
	 * Sets a new path and filename.
	 * This only sets the internal variables but doesn't close nor
	 * open any file in use.
	 * @param path is the path.
	 * @param name is the new filename.
	 */
	void set(const std::string &name)
	{
		setName(name);
	}

	/**
	 * Sets a new filename.
	 * This only sets the internal variables but doesn't close nor
	 * open any file in use.
	 * @param name is the new filename.
	 */
	void setName(const std::string &name) { _filename = name; }

private:
	std::string _filename;

	bool _open(const std::string &filename);
	void _close()
	{
		if (_openFlag) {
			fclose(_pFile);
			_openFlag = false;
		}
	}

	int _get(const char *section, const char *name);
	bool _findString(const char *str);
	bool _findSection(const char *sec);

	FILE *_pFile;
	bool _openFlag;
};

#endif
