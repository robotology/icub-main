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

#ifndef __VISLAB_UTIL_NAMEMAP_H_
#define __VISLAB_UTIL_NAMEMAP_H_

#include <sstream>
#include <string>
#include <stdexcept>
#include <map>

namespace vislab {
namespace util {

/**
 * Wraps a {@link std::map} such that the key is retrieved by the getName() function of class T.
 * Furthermore the toString() function relies on the identically named function of class T.
 *
 * @author Christian Wressnegger
 * @date 2009
 */
template<class T>
class NameMap {
protected:
	/** The underlying {@link std::map} managing the entries. */
	typename std::map<const std::string, T*> entries;

public:

	/** An iterator of the map. */
	class const_iterator {
		typename std::map<const std::string, T*>::const_iterator parentItr;

	public:
		/**
		 * The constructor.
		 */
		const_iterator() {
		}

		/**
		 * The constructor
		 * @param itr The {@link std::map} this iterator should rely on.
		 */
		const_iterator(typename std::map<const std::string, T*>::const_iterator itr) {
			parentItr = itr;
		}

		/**
		 * Returns the dereference operator.
		 * @return The class T the iterator represents.
		 */
		T& operator*() {
			return *(*parentItr).second;
		}

		/**
		 * Implements the "member by pointer" operator.
		 * @return A pointer to the class T the iterator represents.
		 */
		T* const operator->() const {
			return (*parentItr).second;
		}

		/**
		 * Increments this iterator.
		 * @return The subsequent iterator of this one.
		 */
		const_iterator& operator++() {
			parentItr++;
			return *this;
		}

		/**
		 * Decrements this iterator.
		 * @return The preceding iterator of this one.
		 */
		const_iterator& operator--() {
			parentItr--;
			return *this;
		}

		/**
		 * Implements the assignment operator.
		 * @param itr The iterator to be copied.
		 * @return A reference to this iterator.
		 */
		const_iterator& operator=(const const_iterator& itr) {
			parentItr = itr.parentItr;
			return *this;
		}

		/**
		 * Compares the iterators for equality.
		 * @param itr1 The first iterator to be compared with the second one.
		 * @param itr2 The second iterator to be compared with the first one.
		 * @return A boolean indicator.
		 */
		friend bool operator==(const const_iterator& itr1, const const_iterator& itr2) {
			return (itr1.parentItr == itr2.parentItr);
		}

		/**
		 * Compares the iterators for inequality.
		 * @param itr1 The first iterator to be compared with the second one.
		 * @param itr2 The second iterator to be compared with the first one.
		 * @return A boolean indicator.
		 */
		friend bool operator!=(const const_iterator& itr1, const const_iterator& itr2) {
			return !(itr1 == itr2);
		}
	};
	friend class const_iterator;

	/**
	 * The constructor.
	 */
	NameMap() {
	}
	/**
	 * The destructor.
	 */
	virtual ~NameMap() {
		typename std::map<const std::string, T*>::iterator itr;
		for (itr = entries.begin(); itr != entries.end(); ++itr) {
			delete (*itr).second;
		}
	}

	/**
	 * Adds a an instance of class T
	 * @param entry The instance of class T to be hosted by the manager.
	 */
	void add(T* const entry) {
		entries[entry->getName().c_str()] = entry;
	}
	/**
	 * Removes the an instance of class T with the given name.
	 * @param name The name of the command.
	 */
	void remove(const std::string& name) {
		entries.erase(name);
	}

	/**
	 * Determines of the {@link NameMap} contains an instance of class T with the given name.
	 * @param name The name of the instance of class T.
	 * @return An indicator for the existance of the instance of class T in question.
	 */
	bool contains(const std::string name) const {
		return entries.find(name) != entries.end();
	}

	/**
	 * Returns the number of entries.
	 * @return The number of entries.
	 */
	unsigned int size() const {
		return entries.size();
	}

	/**
	 * Returns an indicator stating if the map is empty or not.
	 * @return An indicator stating if the map is empty or not.
	 */
	bool empty() const {
		return size() <= 0;
	}

	/**
	 * Look for and returns an instance of class T with the given name.
	 * @param name The name of the instance of class T.
	 * @return An instance of class T with the given name.
	 */
	const_iterator find(const std::string name) {
		return const_iterator(entries.find(name));
	}

	/**
	 * Returns the instance of class T with the given name.
	 * @param name The name of the instance of class T to look for,
	 * @return The instance of class T with the given name.
	 */
	T& operator[](const std::string name) {
		if (entries.find(name) == entries.end()) {
			throw std::invalid_argument("There is no entry with this name");
		}
		return *entries[name];
	}

	/**
	 * Lists the instances of class T (name and description) hosted by this manager.
	 * @return A list of the instances of class T (name and description) hosted by this manager.
	 */
	std::string toString() const {
		std::ostringstream oss;
		if (entries.empty()) {
			oss << "No entries available";
		} else {
			typename std::map<const std::string, T*>::const_iterator itr = entries.begin();
			oss << (*itr).second->toString().c_str();
			itr++;
			for (; itr != entries.end(); ++itr) {
				oss << std::endl << (*itr).second->toString().c_str();
			}
		}
		return oss.str();
	}

	/**
	 * Returns an iterator to the beginning of the map.
	 * @return An iterator to the beginning of the map.
	 */
	const_iterator begin() {
		return const_iterator(entries.begin());
	}

	/**
	 * Returns a iterator to the end of the map.
	 * @return A iterator to the end of the map.
	 */
	const_iterator end() {
		return const_iterator(entries.end());
	}

};

}
}

#endif /* __VISLAB_UTIL_NAMEMAP_H_ */
