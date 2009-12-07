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

#include "iCub/vislab/SimpleHomography.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace ctrl;

using namespace vislab::yarp;

//#define DEBUG

#if defined (DEBUG)
#include <iostream>
#include <vislab/yarp/util/all.h>
using namespace std;
using namespace vislab::yarp::util;
#endif

namespace vislab {
namespace math {

SimpleHomography::SimpleHomography(const CameraCalibration& c, const Vector* const offset) {
	if (offset == NULL) {
		this->offset.resize(3);
	} else {
		this->offset = *offset;
	}
	cameraCalibration = c;
	heightOffset = 0.0;
	forceRecomputation = true;
	H = eye(3, 3);
	T_rt = eye(4, 4);
	T_cr = eye(4, 4);

#ifdef DEBUG
	cout << "(fx, cx) = (" << c.fx << ", " << c.cx << ")" << endl;
	cout << "(fy, cy) = (" << c.fy << ", " << c.cy << ")" << endl;

	cout << "offset = ";
	if (offset == NULL) {
		cout << "NULL" << endl;
	} else {
		printVector(*offset, cout);
	}

	printMatrix(H, cout);
	printMatrix(T_rt, cout);
	printMatrix(T_cr, cout);
#endif
}

SimpleHomography::~SimpleHomography() {
}

void SimpleHomography::setHeightOffset(double d, bool forceRecompuation) {
	if (heightOffset != d) {
		heightOffset = d;
		if (forceRecompuation) {
			setBaseTransformation( T_cr);
		} else {
			this->forceRecomputation = true;
		}
	}
}

void SimpleHomography::setBaseTransformation(const Matrix& T) { // T: cam <- robot
	if (T_cr != T || forceRecomputation) {
		// body <- table transformation
		T_rt = eye(4, 4);
		for (int i = 0; i < 3 && i < offset.size(); i++) {
			T_rt[i][3] = -offset[i];
		}
		T_rt[2][3] -= heightOffset;
		T_rt = SE3inv(T_rt);

		Matrix T_ct = T * T_rt; // T_cr
		// cancel 3rd column...
		for (int r = 0; r < T_ct.rows(); r++) {
			T_ct[r][2] = T_ct[r][3];
		}
		// ...and 3rd line due to coordinate definitions
		T_ct = T_ct.submatrix(0, 2, 0, 2);

		H = luinv(T_ct);
		T_cr = T;

#ifdef DEBUG
		printMatrix(T);
		printMatrix(T_rt);
		printMatrix(T_ct);
		printMatrix(H);
#endif
	}
}

void SimpleHomography::project(const Vector& in, Vector& out) {

	if (in.size() < 2) {
		throw std::invalid_argument("The input vector has to contain 2D coordinates");
	}

	Vector p_c(3);
	p_c[0] = (in[0] - cameraCalibration.cx) / cameraCalibration.fx; // x_c/z_c
	p_c[1] = (in[1] - cameraCalibration.cy) / cameraCalibration.fy; // y_c/z_c
	p_c[2] = 1; // z_c/z_c

	Vector p_t = H * p_c;
	p_t.resize(p_t.length() + 1);

	// points at the "additional offset plane"
	p_t[0] = p_t[0] / p_t[2];
	p_t[1] = p_t[1] / p_t[2];
	p_t[2] = 0;
	p_t[3] = 1;

	out = T_rt * p_t;
	out.resize(3);

#ifdef DEBUG
	printVector(p_c);
	printVector(p_t);
	printVector(out);
#endif

}

}
}
