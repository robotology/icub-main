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

#ifndef __VISLAB_UTIL_YARP_UTIL_CONTACTABLES_H_
#define __VISLAB_UTIL_YARP_UTIL_CONTACTABLES_H_

#include "common.h"

#include <string>
#include <vector>

#include <yarp/os/all.h>

namespace vislab {
namespace yarp {
namespace util {

/**
 * Manages a set of {@link yarp::os::Contactable}s.
 *
 * @author Christian Wressnegger
 * @date 2009
 *
 * @author Christian Wressnegger
 * @date 2009
 */
class Contactables {
std::vector< ::yarp::os::Contactable*> contactables;
std::vector< ::yarp::os::ConstString> names;

public:

	/**
	 * The destructor.
	 */
	virtual ~Contactables();

	/**
	 * Adds a new {@link yarp::os::Contactable} with the given name.
	 * @param key The identifier of the {@link yarp::as::Contactable}.
	 * @param name The name of the {@link yarp::os::Contactable}.
	 * @param c The {@link yarp::os::Contactable} to be added. The reference  might be NULL!
	 * In that case it is excluded from open, close, interrupt calls. If a existing object is passed,
	 * it will be automatically deleted/ freed by this object.
	 */
	void add(unsigned int& key, ::yarp::os::ConstString& name, ::yarp::os::Contactable* c);
	/**
	 * Returns the name of the {@link yarp::os::Contactable} with the given identifier.
	 * @param key The identifier of the {@link yarp::os::Contactable}.
	 * @return The name of the {@link yarp::os::Contactable} with the given identifier.
	 */
	const ::yarp::os::ConstString getName(const int& key);
	/**
	 * Returns the {@link yarp::os:Contactable} with the given identifier.
	 * @param key The identifier of the {@link yarp::os::Contactable}.
	 * @return The {@link yarp::os:Contactable} with the given identifier.
	 */
	const ::yarp::os::Contactable* const getContactable(const int& key);
	/**
	 * @see #getContactable(key)
	 */
	const ::yarp::os::Contactable* const operator[](const int& key);
	/**
	 * Returns an indicator of if the {@link yarp::os::Contactable} with the given identifier
	 * is part of this container.
	 * @param key The identifier of the {@link yarp::os::Contactable}.
	 * @return an indicator of if the {@link yarp::os::Contactable} with the given identifier
	 * is part of this container.
	 */
	bool contains(const unsigned int& key) const;
	/**
	 * Returns the number of {@link yarp::os::Contactable}s contained in this container.
	 * @return The number of {@link yarp::os::Contactable}s contained in this container.
	 */
	const size_t size() const;

	/**
	 * Opens the {@link yarp::os:Contactable}s according to {@link yarp::os::Contactable#open()}.
	 * @param errorLog Holds the identifiers of the {@link yarp::os::Contactable}s that couldn't
	 * be opened.
	 * @return An indicator of if all {@link yarp::os::Contactable}s could be opened.
	 */
	bool open(std::vector<unsigned int>* const errorLog = NULL);
	/**
	 * Interrupts the {@link yarp::os:Contactable}s according to {@link yarp::os::Contactable#interrupt()}.
	 */
	void interrupt();
	/**
	 * Closes the {@link yarp::os:Contactable}s according to {@link yarp::os::Contactable#close()}.
	 */
	void close();

};

}
}
}

#endif /* __VISLAB_UTIL_YARP_UTIL_CONTACTABLES_H_ */
