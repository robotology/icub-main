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

#ifndef __VISLAB_YARP_UTIL_NAMEMAP_H_
#define __VISLAB_YARP_UTIL_NAMEMAP_H_

#include <sstream>
#include <string>
#include <map>

#include <vislab/util/all.h>
#include <yarp/os/all.h>

namespace vislab {
namespace yarp {
namespace util {

/**
 * Wraps a {@link vislab::util::NameMap<T>} such that it operate on {@link ::yarp::os::ConstString}s.
 * Additionally it make the map thread safe using YARP's {@link ::yarp::os::Semaphore} object.
 *
 * @author Christian Wressnegger
 * @date 2009
 */
template<class T>
class SynchronizedNameMap: public vislab::util::NameMap<T> {
private:
	::yarp::os::Semaphore mutex;

public:
	/**
	 * The constructor.
	 */
	SynchronizedNameMap() {
	}

	/**
	 * @see vislab::util::NameMap<T>#add(T* const)
	 */
	void add(T* const entry) {
		mutex.wait();
		vislab::util::NameMap<T>::add(entry);
		mutex.post();
	}

	/**
	 * @see vislab::util::NameMap<T>#remove(const ::yarp::os::ConstString&)
	 */
	void remove(const ::yarp::os::ConstString& name) {
		mutex.wait();
		vislab::util::NameMap<T>::remove(name.c_str());
		mutex.post();
	}

	/**
	 * @see vislab::util::NameMap<T>#contains(const ::yarp::os::ConstString&)
	 */
	bool contains(const ::yarp::os::ConstString& name) {
		mutex.wait();
		bool b = vislab::util::NameMap<T>::contains(name.c_str());
		mutex.post();
		return b;
	}

	/**
	 * @see vislab::util::NameMap<T>#size()
	 */
	unsigned int size() {
		mutex.wait();
		unsigned int i = vislab::util::NameMap<T>::size();
		mutex.post();
		return i;
	}

	/**
	 * @see vislab::util::NameMap<T>#empty()
	 */
	bool empty() {
		mutex.wait();
		bool b = vislab::util::NameMap<T>::empty();
		mutex.post();
		return b;
	}

	/**
	 * @see vislab::util::NameMap<T>#find(const ::yarp::os::ConstString&)
	 */
	typename vislab::util::NameMap<T>::const_iterator find(const ::yarp::os::ConstString& name) {
		mutex.wait();
		typename vislab::util::NameMap<T>::const_iterator& itr = vislab::util::NameMap<T>::find(
				name.c_str());
		mutex.post();
		return itr;
	}

	/**
	 * @see vislab::util::NameMap<T>#operator[](const ::yarp::os::ConstString&)
	 */
	T& operator[](const ::yarp::os::ConstString& name) {
		mutex.wait();
		T& t = vislab::util::NameMap<T>::operator[](name.c_str());
		mutex.post();
		return t;
	}

	/**
	 * @see vislab::util::NameMap<T>#toString()
	 */
	::yarp::os::ConstString toString() {
		mutex.wait();
		const char* str = vislab::util::NameMap<T>::toString().c_str();
		mutex.post();
		return str;
	}

};

}
}
}

#endif /* __VISLAB_YARP_UTIL_NAMEMAP_H_ */
