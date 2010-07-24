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

#include "vislab/util/common.h"

using namespace std;

namespace vislab {
namespace util {

void explode(const string str, const string separator, vector<string>& results) {
	string s = str;
	size_t found = str.find_first_of(separator);
	while (found != string::npos) {
		if (found > 0) {
			results.push_back(s.substr(0, found));
		}
		s = s.substr(found + 1);
		found = s.find_first_of(separator);
	}
	if (s.length() > 0) {
		results.push_back(s);
	}
}

}
}
