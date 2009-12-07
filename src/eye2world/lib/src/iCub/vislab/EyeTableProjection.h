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

#ifndef __ICUB_VISLAB__VISLAB_PROJECTION_H_
#define __ICUB_VISLAB__VISLAB_PROJECTION_H_

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/iKinFwd.h>
#include <iCub/ctrlMath.h>

#include <stdexcept>

#include "SimpleHomography.h"

namespace vislab {
namespace math {

/**
 * @ingroup icub_eye2world
 */
class EyeTableProjection : public SimpleHomography {
private:
	iKin::iCubEye eye;
	const ::yarp::os::ConstString camera;

	static CameraCalibration getCameraCalibration(::yarp::os::Property& p);

public:
	EyeTableProjection(const ::yarp::os::ConstString& eye, ::yarp::os::Property& calibration,
			const ::yarp::sig::Vector* offset = NULL);
	virtual ~EyeTableProjection();

	void setBaseTransformation(const ::yarp::sig::Vector& torso3d, const ::yarp::sig::Vector& head6d);
};

}
}

#endif /* __ICUB_VISLAB__VISLAB_PROJECTION_H_ */
