
/****************************************************************************** 
 * 
 *  file:  DocBookOutput.h
 * 
 *  Copyright (c) 2004, Michael E. Smoot
 *  All rights reverved.
 * 
 *  See the file COPYING in the top directory of this distribution for
 *  more information.
 *  
 *  THE SOFTWARE IS PROVIDED _AS IS_, WITHOUT WARRANTY OF ANY KIND, EXPRESS 
 *  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 *  DEALINGS IN THE SOFTWARE.  
 *  
 *****************************************************************************/ 

#ifndef TCLAP_DOCBOOKOUTPUT_H
#define TCLAP_DOCBOOKOUTPUT_H

#include <string>
#include <vector>
#include <list>
#include <iostream>
#include <algorithm>

#include <tclap/CmdLineInterface.h>
#include <tclap/CmdLineOutput.h>
#include <tclap/XorHandler.h>
#include <tclap/Arg.h>

namespace TCLAP {

/**
 * A class that generates DocBook output for usage() method for the 
 * given CmdLine and its Args.
 */
class DocBookOutput : public CmdLineOutput
{

	public:

		/**
		 * Prints the usage to stdout.  Can be overridden to 
		 * produce alternative behavior.
		 * \param c - The CmdLine object the output is generated for. 
		 */
		virtual void usage(CmdLineInterface& c);

		/**
		 * Prints the version to stdout. Can be overridden 
		 * to produce alternative behavior.
		 * \param c - The CmdLine object the output is generated for. 
		 */
		virtual void version(CmdLineInterface& c);

		/**
		 * Prints (to stderr) an error message, short usage 
		 * Can be overridden to produce alternative behavior.
		 * \param c - The CmdLine object the output is generated for. 
		 * \param e - The ArgException that caused the failure. 
		 */
		virtual void failure(CmdLineInterface& c, 
						     ArgException& e );

	protected:

		/**
		 * Substitutes the char r for string x in string s.
		 * \param s - The string to operate on. 
		 * \param r - The char to replace. 
		 * \param x - What to replace r with. 
		 */
		void substituteSpecialChars( std::string& s, char r, std::string& x );
};


inline void DocBookOutput::version(CmdLineInterface& _cmd) 
{ 
	std::cout << _cmd.getVersion() << std::endl;
}

inline void DocBookOutput::usage(CmdLineInterface& _cmd ) 
{
	std::list<Arg*> argList = _cmd.getArgList();
	std::string progName = _cmd.getProgramName();
	XorHandler xorHandler = _cmd.getXorHandler();
	std::vector< std::vector<Arg*> > xorList = xorHandler.getXorList();


	std::cout << "<?xml version='1.0'?>" << std::endl;
	std::cout << "<!DOCTYPE book PUBLIC \"-//Norman Walsh//DTD DocBk XML V4.2//EN\"" << std::endl;
	std::cout << "\t\"http://www.oasis-open.org/docbook/xml/4.2/docbookx.dtd\">" << std::endl << std::endl;

	std::cout << "<book>" << std::endl;
	std::cout << "<cmdsynopsis>" << std::endl;

	std::cout << "<command>" << progName << "</command>" << std::endl;

	std::string lt = "&lt;"; 
	std::string gt = "&gt;"; 

	// xor
	for ( int i = 0; (unsigned int)i < xorList.size(); i++ )
	{
		std::cout << "<group>" << std::endl;
		for ( ArgVectorIterator it = xorList[i].begin(); 
						it != xorList[i].end(); it++ )
		{
			std::string id = (*it)->shortID();
			substituteSpecialChars(id,'<',lt);
			substituteSpecialChars(id,'>',gt);
			std::cout << "<arg>" << id << "</arg>" << std::endl; 
		}

		std::cout << "</group>" << std::endl;
	}

	for (ArgListIterator it = argList.begin(); it != argList.end(); it++)
		if ( !xorHandler.contains( (*it) ) )
		{
			std::string id = (*it)->shortID();
			substituteSpecialChars(id,'<',lt);
			substituteSpecialChars(id,'>',gt);
			std::cout << "<arg>" << id << "</arg>" << std::endl; 
		}

 	std::cout << "</cmdsynopsis>" << std::endl;
 	std::cout << "</book>" << std::endl;

}

inline void DocBookOutput::failure( CmdLineInterface& _cmd,
				                ArgException& e ) 
{ 
		std::cout << e.what() << std::endl;
}

inline void DocBookOutput::substituteSpecialChars( std::string& s,
				                                   char r,
												   std::string& x )
{
	size_t p;
	while ( (p = s.find_first_of(r)) != std::string::npos )
	{
		s.erase(p,1);
		s.insert(p,x);
	}
}

} //namespace TCLAP
#endif 
