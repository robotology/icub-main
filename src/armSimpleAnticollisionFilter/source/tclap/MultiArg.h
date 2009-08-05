/****************************************************************************** 
 * 
 *  file:  MultiArg.h
 * 
 *  Copyright (c) 2003, Michael E. Smoot .
 *  Copyright (c) 2004, Michael E. Smoot, Daniel Aarno.
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


#ifndef TCLAP_MULTIPLE_ARGUMENT_H
#define TCLAP_MULTIPLE_ARGUMENT_H

#include <string>
#include <vector>

#include <tclap/Arg.h>

#ifdef HAVE_CONFIG_H
#include <config.h>
#else
#define HAVE_SSTREAM
#endif

#if defined(HAVE_SSTREAM)
#include <sstream>
#elif defined(HAVE_STRSTREAM)
#include <strstream>
#else
#error "Need a stringstream (sstream or strstream) to compile!"
#endif

namespace TCLAP {

template<class T> class MultiArg;

namespace MULTI_ARG_HELPER {

enum Error_e { EXTRACT_FAILURE = 1000, EXTRACT_TOO_MANY };

/**
 * This class is used to extract a value from an argument. 
 * It is used because we need a special implementation to
 * deal with std::string and making a specialiced function
 * puts it in the T segment, thus generating link errors.
 * Having a specialiced class makes the symbols weak.
 * This is not pretty but I don't know how to make it
 * work any other way.
 */
template<class T>
class ValueExtractor 
{
	friend class MultiArg<T>;
  
	private:

		/**
		 * Reference to the vector of values where the result of the 
		 * extraction will be put.
		 */
   		std::vector<T> &_values;
  
		/**
		 * Constructor.
		 * \param values - Where the values extracted will be put.
		 */
		ValueExtractor(std::vector<T> &values) : _values(values) {}
  
		/**
		 * Method that will attempt to parse the input stream for values
		 * of type T.
		 * \param val - Where the values parsed will be put.
		 */
		int extractValue( const std::string& val ) 
		{
			T temp;

#if defined(HAVE_SSTREAM)
			std::istringstream is(val);
#elif defined(HAVE_STRSTREAM)
			std::istrstream is(val.c_str());
#else
#error "Need a stringstream (sstream or strstream) to compile!"
#endif

			int valuesRead = 0;
    
			while ( is.good() ) 
			{
				if ( is.peek() != EOF )
					is >> temp; 
				else
					break;
      
				valuesRead++;
			}		
    
			if ( is.fail() )
				return EXTRACT_FAILURE;
    
			if ( valuesRead > 1 )
				return EXTRACT_TOO_MANY;
    
			_values.push_back(temp);
    
			return 0;
		} 
};

/**
 * Specialization for string.  This is necessary because istringstream
 * operator>> is not able to ignore spaces...  meaning -x "X Y" will only 
 * read 'X'... and thus the specialization.
 */
template<>
class ValueExtractor<std::string> 
{
	friend class MultiArg<std::string>;

   	private:

		/**
		 * Reference to the vector of strings where the result of the 
		 * extraction will be put.
		 */
        std::vector<std::string> &_values;
  
		/**
		 * Constructor.
		 * \param values - Where the strings extracted will be put.
		 */
        ValueExtractor(std::vector<std::string> &values) : _values(values) {}

		/**
		 * Method that will attempt to parse the input stream for values
		 * of type std::string.
		 * \param val - Where the values parsed will be put.
		 */
        int extractValue( const std::string& val ) 
		{
            _values.push_back( val );
            return 0;
        }
};

} //namespace MULTI_ARG_HELPER

/**
 * An argument that allows multiple values of type T to be specified.  Very
 * similar to a ValueArg, except a vector of values will be returned
 * instead of just one.
 */
template<class T>
class MultiArg : public Arg
{
	protected:

		/**
		 * The list of values parsed from the CmdLine.
		 */
		std::vector<T> _values;

		/**
		 * A list of allowed values.
		 * A list of values allowed for this argument. If the value parsed
		 * for this arg is not found in this list, then an exception is
		 * thrown.  If the list is empty, then any value is allowed.
		 */
		std::vector<T> _allowed;

		/**
		 * The description of type T to be used in the usage.
		 */
		std::string _typeDesc;

		/**
		 * Extracts the value from the string.
		 * Attempts to parse string as type T, if this fails an exception
		 * is thrown.
		 * \param val - The string to be read.
		 */
		void _extractValue( const std::string& val );

		/**
		 * Checks to see if parsed value is in allowed list.
		 * \param val - value parsed (only used in output).
		 */
		void _checkAllowed( const std::string& val );

	public:

   		/**
		 * Constructor.
		 * \param flag - The one character flag that identifies this
		 * argument on the command line.
		 * \param name - A one word name for the argument.  Can be
		 * used as a long flag on the command line.
		 * \param desc - A description of what the argument is for or
		 * does.
		 * \param req - Whether the argument is required on the command
		 * line.
		 * \param typeDesc - A short, human readable description of the
		 * type that this object expects.  This is used in the generation
		 * of the USAGE statement.  The goal is to be helpful to the end user
		 * of the program.
		 * \param v - An optional visitor.  You probably should not
		 * use this unless you have a very good reason.
		 */
		MultiArg( const std::string& flag,
                  const std::string& name,
                  const std::string& desc,
                  bool req,
                  const std::string& typeDesc,
                  Visitor* v = NULL);

		/**
		 * Constructor.
		 * \param flag - The one character flag that identifies this
		 * argument on the command line.
		 * \param name - A one word name for the argument.  Can be
		 * used as a long flag on the command line.
		 * \param desc - A description of what the argument is for or
		 * does.
		 * \param req - Whether the argument is required on the command
		 * line.
		 * \param typeDesc - A short, human readable description of the
		 * type that this object expects.  This is used in the generation
		 * of the USAGE statement.  The goal is to be helpful to the end user
		 * of the program.
		 * \param parser - A CmdLine parser object to add this Arg to
		 * \param v - An optional visitor.  You probably should not
		 * use this unless you have a very good reason.
		 */
		MultiArg( const std::string& flag, 
                  const std::string& name,
                  const std::string& desc,
                  bool req,
                  const std::string& typeDesc,
                  CmdLineInterface& parser,
                  Visitor* v = NULL );

		/**
		 * Constructor.
		 * \param flag - The one character flag that identifies this
		 * argument on the command line.
		 * \param name - A one word name for the argument.  Can be
		 * used as a long flag on the command line.
		 * \param desc - A description of what the argument is for or
		 * does.
		 * \param req - Whether the argument is required on the command
		 * line.
		 * \param allowed - A vector of type T that where the values in the
		 * vector are the only values allowed for the arg.
		 * \param v - An optional visitor.  You probably should not
		 * use this unless you have a very good reason.
		 */
		MultiArg( const std::string& flag,
                  const std::string& name,
                  const std::string& desc,
                  bool req,
                  const std::vector<T>& allowed,
                  Visitor* v = NULL );
		  
		/**
		 * Constructor.
		 * \param flag - The one character flag that identifies this
		 * argument on the command line.
		 * \param name - A one word name for the argument.  Can be
		 * used as a long flag on the command line.
		 * \param desc - A description of what the argument is for or
		 * does.
		 * \param req - Whether the argument is required on the command
		 * line.
		 * \param allowed - A vector of type T that where the values in the
		 * vector are the only values allowed for the arg.
		 * \param parser - A CmdLine parser object to add this Arg to
		 * \param v - An optional visitor.  You probably should not
		 * use this unless you have a very good reason.
		 */
		MultiArg( const std::string& flag, 
                  const std::string& name,
                  const std::string& desc,
                  bool req,
                  const std::vector<T>& allowed,
                  CmdLineInterface& parser,
                  Visitor* v = NULL );
		  
		/**
		 * Handles the processing of the argument.
		 * This re-implements the Arg version of this method to set the
		 * _value of the argument appropriately.  It knows the difference
		 * between labeled and unlabeled.
		 * \param i - Pointer the the current argument in the list.
		 * \param args - Mutable list of strings. Passed from main().
		 */
		virtual bool processArg(int* i, std::vector<std::string>& args); 

		/**
		 * Returns a vector of type T containing the values parsed from
		 * the command line.
		 */
		const std::vector<T>& getValue();

		/**
		 * Returns the a short id string.  Used in the usage. 
		 * \param val - value to be used.
		 */
		virtual std::string shortID(const std::string& val="val") const;

		/**
		 * Returns the a long id string.  Used in the usage. 
		 * \param val - value to be used.
		 */
		virtual std::string longID(const std::string& val="val") const;

		/**
		 * Once we've matched the first value, then the arg is no longer
		 * required.
		 */
		virtual bool isRequired() const;

	private:

		/**
		 * Common initialization code for constructors with allowed vectors.
		 */
		void allowedInit();
};

/**
 *
 */
template<class T>
void MultiArg<T>::allowedInit()
{
	for ( unsigned int i = 0; i < _allowed.size(); i++ )
   	{

#if defined(HAVE_SSTREAM)
   		std::ostringstream os;
#elif defined(HAVE_STRSTREAM)
   		std::ostrstream os;
#else
#error "Need a stringstream (sstream or strstream) to compile!"
#endif

   		os << _allowed[i];

   		std::string temp( os.str() );

   		if ( i > 0 )
			_typeDesc += "|";
	
		_typeDesc += temp;
	}
}

/**
 *
 */
template<class T>
MultiArg<T>::MultiArg(const std::string& flag, 
                      const std::string& name,
                      const std::string& desc,
                      bool req,
                      const std::string& typeDesc,
                      Visitor* v)
: Arg( flag, name, desc, req, true, v ),
  _typeDesc( typeDesc )
{ }

template<class T>
MultiArg<T>::MultiArg(const std::string& flag, 
                      const std::string& name,
                      const std::string& desc,
                      bool req,
                      const std::string& typeDesc,
                      CmdLineInterface& parser,
                      Visitor* v)
: Arg( flag, name, desc, req, true, v ),
  _typeDesc( typeDesc )
{ 
	parser.add( this );
}

/**
 *
 */
template<class T>
MultiArg<T>::MultiArg(const std::string& flag, 
                      const std::string& name,
                      const std::string& desc,
                      bool req,
                      const std::vector<T>& allowed,
                      Visitor* v)
: Arg( flag, name, desc, req, true, v ),
  _allowed( allowed )
{ 
	allowedInit();
}

template<class T>
MultiArg<T>::MultiArg(const std::string& flag, 
                      const std::string& name,
                      const std::string& desc,
                      bool req,
                      const std::vector<T>& allowed,
                      CmdLineInterface& parser,
                      Visitor* v)
: Arg( flag, name, desc, req, true, v ),
  _allowed( allowed )
{ 
	allowedInit();
	parser.add( this );
}

/**
 *
 */
template<class T>
const std::vector<T>& MultiArg<T>::getValue() { return _values; }

/**
 *
 */
template<class T>
bool MultiArg<T>::processArg(int *i, std::vector<std::string>& args) 
{
 	if ( _ignoreable && Arg::ignoreRest() )
		return false;

	if ( _hasBlanks( args[*i] ) )
		return false;

	std::string flag = args[*i];
	std::string value = "";

   	trimFlag( flag, value );

   	if ( argMatches( flag ) )
   	{
   		if ( Arg::delimiter() != ' ' && value == "" )
			throw( ArgParseException( 
			           "Couldn't find delimiter for this argument!",
					   toString() ) );

		if ( value == "" )
		{
			(*i)++;
			if ( (unsigned int)*i < args.size() )
				_extractValue( args[*i] );
			else
				throw( ArgParseException("Missing a value for this argument!",
                                         toString() ) );
		}
		else
			_extractValue( value );

		_checkWithVisitor();

		return true;
	}
	else
		return false;
}

/**
 * Checks to see if the value parsed is in the allowed list.
 */
template<class T>
void MultiArg<T>::_checkAllowed( const std::string& val )
{
	if ( _allowed.size() > 0 )
		if ( find(_allowed.begin(),_allowed.end(),_values.back()) 
                 == _allowed.end() )
			throw( CmdLineParseException( "Couldn't find '" + val +
                                          "' in allowed list.", toString() ) );
}

/**
 *
 */
template<class T>
std::string MultiArg<T>::shortID(const std::string& val) const
{
	std::string id = Arg::shortID(_typeDesc) + " ... ";

	return id;
}

/**
 *
 */
template<class T>
std::string MultiArg<T>::longID(const std::string& val) const
{
	std::string id = Arg::longID(_typeDesc) + "  (accepted multiple times)";

	return id;
}

/**
 * Once we've matched the first value, then the arg is no longer
 * required.
 */
template<class T>
bool MultiArg<T>::isRequired() const
{
	if ( _required )
	{
		if ( _values.size() > 1 )
			return false;
		else
			return true;
   	}
   	else
		return false;

}

template<class T>
void MultiArg<T>::_extractValue( const std::string& val ) 
{
	MULTI_ARG_HELPER::ValueExtractor<T> ve(_values);
		  
	int err = ve.extractValue(val);

	if ( err == MULTI_ARG_HELPER::EXTRACT_FAILURE )
		throw( ArgParseException("Couldn't read argument value "
                                 "from string '" + val + "'", toString() ) );

	if(err == MULTI_ARG_HELPER::EXTRACT_TOO_MANY)
	    throw( ArgParseException("More than one valid value "
                                 "parsed from string '" + val + "'", 
								 toString() ) );		    
	_checkAllowed( val );
}
		

} // namespace TCLAP

#endif
