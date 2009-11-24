#ifndef IHA_UTILS_H
#define IHA_UTILS_H

#include <yarp/String.h> 
#include <ace/OS.h>
#include <ace/Date_Time.h>
#include <yarp/os/all.h>
#include <cmath>

using yarp::String;
using namespace std;
using namespace yarp::os;

namespace iCub {
	namespace iha {
		unsigned char getBin(double val, int nb);
		bool boolStringTest(ConstString str);
		char * printHeader(char ** argv);
		bool connectToParam(Searchable& config, const char* paramName, const char* toPortName, double delay, yarp::os::Module *module, bool reverse=false);
		bool connectToParamReverse(Searchable& config, const char* paramName, const char* toPortName, double delay, yarp::os::Module *module);
		bool connectToParamWithSuffix(Searchable& config, const char* paramName, const char* suffix, const char* toPortName, double delay, yarp::os::Module *module, bool reverse=false);
		bool connectToParamWithSuffixReverse(Searchable& config, const char* paramName, const char* suffix, const char* toPortName, double delay, yarp::os::Module *module);
		bool connectToParamNonModule(Searchable& config, const char* paramName, const char* toPortName, double delay, bool& running, bool reverse=false);
		void writeStatus(yarp::os::Port &outport, const char* subsystem, const char* item, int value) ;
	}
}
using namespace iCub::iha;

/**
 * Function to calculate bin number
 * for given (normalized) value
 */
unsigned char iCub::iha::getBin(double val, int nb) {
	//int bin = (unsigned char) (val * (double) nb);
	//if (bin>=nb) bin=(unsigned char) (nb-1);
	//if (bin<=0) bin=0;

	// Don't use type casting to return floor/ceiling as the internal
	// binary representation screws this up
	
	// the correct way to do this is by using 1 les than the ceiling function
	// as then the range will always be 0..(nb-1)
	//
	//  except for special case val=0
	if (val<=0) return 0;
	//
	// Catch situations where the normalised value was greater than 1
	if (val>1) return (nb-1);

	return (unsigned char) (ceil( val * (double) nb)) -1;
}

/**
 * Test a string for truth value
 */
bool iCub::iha::boolStringTest(ConstString str) {
	if (str=="TRUE" || 
	    str=="True" || 
	    str=="true" || 
	    str=="ON" || 
	    str=="On" || 
	    str=="on" || 
	    str=="1"
	   ) {
		return true; 
	}
	return false;
}

/**
 * Print a header string containing program name and date/time
 */
char * iCub::iha::printHeader(char ** argv) {
	ACE_Date_Time now(ACE_OS::gettimeofday());
	char *prog = NULL; char *next = NULL;
	prog = strtok(argv[0],	ACE_DIRECTORY_SEPARATOR_STR_A);
	next = strtok(NULL,	ACE_DIRECTORY_SEPARATOR_STR_A);
	while ( next!=NULL ) {
		prog = next;
		next = strtok(NULL,	ACE_DIRECTORY_SEPARATOR_STR_A);
	}
	ACE_OS::printf("PROGRAM %s DATE %04d%02d%02d TIME %02d%02d%02d\n",prog,
			now.year(),now.month(),now.day(),now.hour(),now.minute(),now.second());
	return prog;
}

/**
 * Connect a port to the port given in a parameter while also
 * waiting for it to be available
 * Module version
 */
bool iCub::iha::connectToParam(Searchable& config, const char* paramName, const char* toPortName, double delay, yarp::os::Module *module, bool reverse) {
	if (config.check(paramName)) {
		ConstString paramPortName = config.find(paramName).asString();
		IhaDebug::pmesg(DBGL_INFO,"Waiting for port %s ",paramPortName.c_str());

		while(!Network::sync(paramPortName.c_str()) && !module->isStopping()) {
				Time::delay(delay);
				IhaDebug::pmesg(DBGL_INFO,".");
		}
		if (reverse) {
			IhaDebug::pmesg(DBGL_INFO," ready. Now connecting %s to %s ",toPortName,paramPortName.c_str());
			if (!Network::connect(toPortName,config.find(paramName).asString().c_str(),0,false)) {
				IhaDebug::pmesg(DBGL_INFO,"Error.\n");
				return false;
			}
		} else {
			IhaDebug::pmesg(DBGL_INFO," ready. Now connecting %s to %s ",paramPortName.c_str(),toPortName);
			if (!Network::connect(config.find(paramName).asString().c_str(),toPortName,0,false)) {
				IhaDebug::pmesg(DBGL_INFO,"Error.\n");
				return false;
			}
		}
		IhaDebug::pmesg(DBGL_INFO," connected.\n");
		return true;
	} else {
		IhaDebug::pmesg(DBGL_INFO,"ConnectToParam: Error: No parameter %s\n",paramName);
		return false;
	}
}

/**
 * Connect a port to the port given in a parameter (reverse connection)
 * while also waiting for it to be available
 * Module version
 */
bool iCub::iha::connectToParamReverse(Searchable& config, const char* paramName, const char* toPortName, double delay, yarp::os::Module *module) {
	connectToParam(config, paramName, toPortName, delay, module, true);
}

/**
 * Connect a port to the port given in a parameter + specified suffix
 * while also waiting for it to be available
 * Module version
 */
bool iCub::iha::connectToParamWithSuffix(Searchable& config, const char* paramName, const char* suffix, const char* toPortName, double delay, yarp::os::Module *module, bool reverse) {
	const char* carrier = "tcp";
	if (config.check(paramName)) {
		ConstString paramPortName = config.find(paramName).asString() + suffix;
		IhaDebug::pmesg(DBGL_INFO,"Waiting for port %s ",paramPortName.c_str());

		while(!Network::sync(paramPortName.c_str()) && !module->isStopping()) {
				Time::delay(delay);
				IhaDebug::pmesg(DBGL_INFO,".");
		}
		if (reverse) {
			IhaDebug::pmesg(DBGL_INFO," ready. Now connecting %s to %s ",toPortName,paramPortName.c_str());
			if (!Network::connect(toPortName,paramPortName.c_str(),carrier,false)) {
				IhaDebug::pmesg(DBGL_INFO,"Error.\n");
				return false;
			}
		} else {
			IhaDebug::pmesg(DBGL_INFO," ready. Now connecting %s to %s ",paramPortName.c_str(),toPortName);
			if (!Network::connect(paramPortName.c_str(),toPortName,carrier,false)) {
				IhaDebug::pmesg(DBGL_INFO,"Error.\n");
				return false;
			}
		}
		IhaDebug::pmesg(DBGL_INFO," connected.\n");
		return true;
	} else {
		IhaDebug::pmesg(DBGL_INFO,"ConnectToParam: Error: No parameter %s\n",paramName);
		return false;
	}

}

/**
 * Connect a port to the port given in a parameter + specified suffix
 * (reverse direction)
 * while also waiting for it to be available
 * Module version
 */
bool iCub::iha::connectToParamWithSuffixReverse(Searchable& config, const char* paramName, const char* suffix, const char* toPortName, double delay, yarp::os::Module *module) {
	connectToParamWithSuffix(config, paramName, suffix, toPortName, delay, module, true);
}

/**
 * Connect a port to the port given in a parameter while also
 * waiting for it to be available
 * Non-Module version
 */
bool iCub::iha::connectToParamNonModule(Searchable& config, const char* paramName, const char* toPortName, double delay, bool& running, bool reverse) {
	if (config.check(paramName)) {
		ConstString paramPortName = config.find(paramName).asString();
		IhaDebug::pmesg(DBGL_INFO,"Waiting for port %s ",paramPortName.c_str());

		while(!Network::sync(paramPortName.c_str()) && running) {
				Time::delay(delay);
				IhaDebug::pmesg(DBGL_INFO,".");
		}
		if (reverse) {
			IhaDebug::pmesg(DBGL_INFO," ready. Now connecting %s to %s ",toPortName,paramPortName.c_str());
			if (!Network::connect(toPortName,config.find(paramName).asString().c_str(),0,false)) {
				IhaDebug::pmesg(DBGL_INFO,"Error.\n");
				return false;
			}
		} else {
			IhaDebug::pmesg(DBGL_INFO," ready. Now connecting %s to %s ",paramPortName.c_str(),toPortName);
			if (!Network::connect(config.find(paramName).asString().c_str(),toPortName,0,false)) {
				IhaDebug::pmesg(DBGL_INFO,"Error.\n");
				return false;
			}
		}
		IhaDebug::pmesg(DBGL_INFO," connected.\n");
		return true;
	} else {
		IhaDebug::pmesg(DBGL_INFO,"ConnectToParam: Error: No parameter %s\n",paramName);
		return false;
	}
}

/**
 * Utility to write a status message to a port for the Status Monitor
 */
void iCub::iha::writeStatus(Port &outport, const char* subsystem, const char* item, int value) {
	Bottle b;
	b.addString(subsystem);
	b.addString(item);
	b.addInt(value);
	outport.write(b);
}
#endif
