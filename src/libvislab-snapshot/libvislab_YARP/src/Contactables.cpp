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

#include "vislab/yarp/util/Contactables.h"

using namespace std;
using namespace yarp::os;

namespace vislab {
namespace yarp {
namespace util {

Contactables::~Contactables() {
	vector<Contactable*>::const_iterator itr;
	for (itr = contactables.begin(); itr != contactables.end(); ++itr) {
		delete (*itr);
	}
}

void Contactables::add(unsigned int& key, ConstString& name, Contactable* c) {
	// TODO: check for double entries based on name
	contactables.push_back(c);
	names.push_back(name);
	key = names.size() - 1;
	replaceDoubleSlash(names[key]);
}

const ConstString Contactables::getName(const int& key) {
	return contains(key) ? names[key] : "";
}

const Contactable* const Contactables::getContactable(const int& key) {
	return contains(key) ? contactables[key] : NULL;
}

const Contactable* const Contactables::operator[](const int& key) {
	return getContactable(key);
}

bool Contactables::contains(const unsigned int& key) const {
	return key < contactables.size();
}

const size_t Contactables::size() const {
	return contactables.size();
}

bool Contactables::open(vector<unsigned int>* const errorLog) {
	bool b = true;
	vector<ConstString>::const_iterator itr;
	for (unsigned int key = 0; key < names.size(); key++) {
		if (contactables[key] != NULL) {
			if (!contactables[key]->open(names[key].c_str())) {
				if (errorLog != NULL) {
					errorLog->push_back(key);
				}
				b = false;
			}
		}
	}
	return b;
}

void Contactables::interrupt() {
	vector<Contactable*>::const_iterator itr;
	for (itr = contactables.begin(); itr != contactables.end(); ++itr) {
		if ((*itr) != NULL) {
			(*itr)->interrupt();
		}
	}
}

void Contactables::close() {
	vector<Contactable*>::const_iterator itr;
	for (itr = contactables.begin(); itr != contactables.end(); ++itr) {
		if ((*itr) != NULL) {
			(*itr)->close();
		}
	}
}

}
}
}
