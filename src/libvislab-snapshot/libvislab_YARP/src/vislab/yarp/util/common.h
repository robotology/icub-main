/*******************************************************************************
 * Copyright (C) 2009 Christian Wressnegger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *******************************************************************************/

#ifndef __VISLAB_YARP_UTIL_COMMON_H__
#define __VISLAB_YARP_UTIL_COMMON_H__

#include "MotionSequence.h"

#include <vislab/util/common.h>

#include <iostream>
#include <sstream>
#include <cstring>
#include <string>
#include <vector>
#include <map>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/math/Math.h>

namespace vislab {
namespace yarp {
namespace util {

/**
 * Adds a string with multiple line to the specified bottle.
 * @param b The {@link Bottle} to be populated.
 * @param str The multi-line {@link ConstString}.
 */
void addMultilineString(::yarp::os::Bottle& b, const ::yarp::os::ConstString& str);

/**
 * Strips the following value from the given {@link yarp::os::Property}:
 * "from", "style", "/", "default_capability", "capability_directory"
 * @param p The {@link yarp::os::Property} to be modified.
 */
void stripRFProperties(::yarp::os::Property& p);

/**
 * Replaces double occurrences of slashes "/" by a single one in the given {@link ::yarp::os:ConstString}
 * @param str The {@link ::yarp::os::ConstString} to be modified.
 * @return
 */
bool replaceDoubleSlash(::yarp::os::ConstString &str);
/**
 * Prints the contents of the data vector to the provided {@link std::ostream}.
 * @param v The data vector of size <param>len</param>
 * @param len The lenght of the data vector
 * @param s The output stream.
 */
void printVector(const double* const v, const size_t len, std::ostream& s = std::cout);
/**
 * Prints the contents of the {@link ::yarp::sig::Vector} to the provided {@link std::ostream}.
 * @param m The {@::yarp::sig::Vector} to be printed.
 * @param s The output stream.
 */
void printVector(const ::yarp::sig::Vector &v, std::ostream& s = std::cout);
/**
 * Prints the contents of the {@link ::yarp::sig::Matrix} to the provided {@link std::ostream}.
 * @param m The {@::yarp::sig::Matrix} to be printed.
 * @param s The output stream.
 */
void printMatrix(const ::yarp::sig::Matrix &m, std::ostream& s = std::cout);

/**
 * Writes the contents of the provided {@link ::yarp::os::Bottle} object as double values
 * to the given memory.
 * @param in The {@link ::yarp::os::Bottle} object to be parsed.
 * @param out The destination memory location.
 * @param valueSize The expected size of the data, i.e. the size of data the memory location
 * is able to hold.
 */
void readVector(const ::yarp::os::Bottle& in, double* out, const int valueSize);
/**
 * Converts the content of the provided {@link ::yarp::os::Bottle} object to a {@link ::yarp::sig::Vector}.
 * @param in The {@link ::yarp::os::Bottle} object to be parsed.
 * @param out The destination {@link ::yarp::sig::Vector}.
 * @param valueSize The expected size of the data. If <= 0 the size of the {@link ::yarp::sig::Vector}
 * will be adjusted automatically.
 */
void readVector(const ::yarp::os::Bottle& in, ::yarp::sig::Vector& out, const int valueSize = 0);
/**
 * Converts the content of the provided {@link ::yarp::os::Bottle} object to {@link ::yarp::sig::Vector}s
 * with the according identifiers.
 * @param in The {@link ::yarp::os::Bottle} object to be parsed.
 * @param out The {@link std::map} holding the resulting {@link ::yarp::sig::Vector}s.
 * @param valueSize The expected size of each {@link ::yarp::sig::Vector}. If <= 0 the size of the {@link ::yarp::sig::Vector}
 * will be adjusted automatically.
 */
void readVectors(::yarp::os::Bottle& in, std::map<const std::string, ::yarp::sig::Vector>& output,
		const std::string values[], const int numValues, const int valueSize = 0);
/**
 * Converts the content of the provided {@link ::yarp::os::Bottle} object to matrices
 * with the according identifiers.
 * @param in The {@link ::yarp::os::Bottle} object to be parsed.
 * @param out The {@link std::map} holding the resulting matrices.
 * @param isSubProperty Indicates if the content to be converted is a sub property of the {@link ::yarp::os::Bottle}.
 * If so the function ignores the first layer of the {@link ::yarp::os::Bottle} structure.
 */
void readMatrices(const ::yarp::os::Bottle& in,
		std::map<const std::string, ::yarp::sig::Matrix>& output, bool isSubProperty = false);

class MotionSequence;

/**
 * Converts the content of the provided {@link ::yarp::os::Bottle} object to a {@link MotionSequence}.
 * @param in The {@link ::yarp::os::Bottle} object to be parsed.
 * @param output The {@link MotionSequence} to be populated.
 */
void readMotionSequence(const ::yarp::os::Bottle& in, vislab::yarp::util::MotionSequence& output);
/**
 * Converts the content of the provided {@link ::yarp::os::Bottle} object to {@link MotionSequence}s
 * with the according identifiers.
 * @param in The {@link ::yarp::os::Bottle} object to be parsed.
 * @param output The {@link std::map} holding the resulting {@link MotionSequence}s.
 */
void readMotionSequences(const ::yarp::os::Bottle& in, std::map<const std::string,
		vislab::yarp::util::MotionSequence>& output);

/**
 * Parses a comma separated list of {@link T} objects.
 * @param in The input {@link ConstString}.
 * @param out The output {@link std::vector} of {@link T} objects.
 */
template<class T>
void parseListOf(const ::yarp::os::ConstString& in, std::vector<T>& out) {

	std::vector<std::string> strList;
	vislab::util::explode(in.c_str(), ",", strList);
	if (strList.size() <= 0) {
		out.clear();
	} else {
		out.resize(strList.size());
		for (unsigned int i = 0; i < strList.size(); i++) {
			std::istringstream ss(strList[i]);
			ss >> out[i];
		}
	}
}

/**
 * Parses a comma separated list of {@link T} objects.
 * @param in The input {@link ConstString}.
 * @param out The output {@link VectorOf} {@link T} objects.
 */
template<class T>
void parseListOf(const ::yarp::os::ConstString& in, ::yarp::sig::VectorOf<T>& out) {
	::yarp::sig::VectorOf<T> v;
	parseListOf<T> (in, v);
	if (in.length() <= 0) {
		out.clear();
	} else {
		out.resize(v.size());
		for (int i = 0; i < out.size(); i++) {
			out[i] = v[i];
		}
	}
}

/**
 * Parses a comma separated list of double values.
 * @param in The input {@link ConstString}.
 * @param out The output {@link Vector}.
 */
void parseListOfDoubles(const ::yarp::os::ConstString& in, ::yarp::sig::Vector& out);

}
}
}

#endif // __VISLAB_YARP_UTIL_COMMON_H__
