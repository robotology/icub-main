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

#include "iCub/vislab/EyeTableProjection.h"

using namespace yarp::os;
using namespace yarp::sig;

using namespace std;

using namespace iKin;
using namespace ctrl;

//#define DEBUG

#if defined (DEBUG)
#include <iostream>
#include <vislab/yarp/util/all.h>
using namespace std;
using namespace vislab::yarp::util;
#endif

namespace vislab {
namespace math {

CameraCalibration EyeTableProjection::getCameraCalibration(Property& p) {
	CameraCalibration c;

	c.fx = p.find("fx").asDouble();
	c.fy = p.find("fy").asDouble();
	c.cx = p.find("cx").asDouble();
	c.cy = p.find("cy").asDouble();
	c.fx = c.fx == 0.0 ? 1.0 : c.fx;
	c.fy = c.fy == 0.0 ? 1.0 : c.fy;

	return c;
}

EyeTableProjection::EyeTableProjection(const ConstString& eye, Property& calibration, const Vector* offset) :
	SimpleHomography(getCameraCalibration(calibration), offset), eye(eye.c_str()), camera(eye) {

	if (eye != "right" && eye != "left") {
		throw invalid_argument("You have to properly specify one of the eyes (\"left\" | \"right\"");
	}

	for (int j = 0; j < 3; j++) {
		this->eye.releaseLink(j);
	}
	iKinChain *chain = this->eye.asChain();
	(*chain)[6].setMax(35 * M_PI / 180.0);
}

EyeTableProjection::~EyeTableProjection() {
}

void EyeTableProjection::setBaseTransformation(const ::yarp::sig::Vector& torso3d,
		const ::yarp::sig::Vector& head6d) {

	Vector v(8);
	// order according to http://eris.liralab.it/wiki/ICubHeadKinematics
	v[0] = torso3d[2]; // pitch
	v[1] = torso3d[1]; // roll
	v[2] = torso3d[0]; // yaw
	v[3] = head6d[0]; // neck pitch
	v[4] = head6d[1]; // neck roll
	v[5] = head6d[2]; // neck yaw
	v[6] = head6d[3]; // tilt of the eye's
	v[7] = camera == "left" ? head6d[4] + head6d[5] / 2 : head6d[4] - head6d[5] / 2;

	// eye -> robot
	Matrix T = SE3inv(eye.getH(v));
	SimpleHomography::setBaseTransformation(T);
}

}
}
