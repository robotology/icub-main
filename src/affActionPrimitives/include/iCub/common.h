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

#ifndef __AFFACTIONPRIMITIVES_COMMON_H__
#define __AFFACTIONPRIMITIVES_COMMON_H__

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/os/Bottle.h>


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


#endif // __VISLAB_YARP_UTIL_COMMON_H__
