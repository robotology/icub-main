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

#include "vislab/yarp/sig.h"

#include <stdexcept>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;

namespace vislab {
namespace yarp {

// operators

Vector operator/(const Vector& left, const double right) {
	Vector result(left.size());
	for (int i = 0; i < result.size(); i++) {
		result[i] = left[i] / right;
	}
	return result;
}

Vector operator/(const Vector& left, const Vector& right) {
	if (left.size() != right.size()) {
		throw invalid_argument("Incompatible size");
	}

	Vector result(right.size());
	for (int i = 0; i < result.size(); i++) {
		result[i] = left[i] / right[i];
	}
	return result;
}

Vector operator*(const Vector& left, const Vector& right) {
	if (left.size() != right.size()) {
		throw invalid_argument("Incompatible size");
	}

	Vector result(right.size());
	for (int i = 0; i < result.size(); i++) {
		result[i] = left[i] * right[i];
	}
	return result;
}

bool operator!=(const Matrix& left, const Matrix& right) {
	return !(left == right);
}

void resize(::yarp::sig::Vector& v, int s, double def) {
	if (v.size() != s) {
		Vector tmp(v);
		v.resize(s, def);
		for (int i = 0; i < s && i < tmp.size(); i++) {
			v[i] = tmp[i];
		}
	}
}

void resize(::yarp::sig::Matrix& m, int r, int c) {
	if (m.rows() != r || m.cols() != c) {
		Matrix tmp(m);
		m.resize(r, c);
		for (int i = 0; i < r && i < tmp.rows(); i++) {
			for (int j = 0; j < c && j < tmp.cols(); j++) {
				m[i][j] = tmp[i][j];
			}
		}
	}
}

}
}
