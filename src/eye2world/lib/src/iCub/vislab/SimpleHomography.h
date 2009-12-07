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

#ifndef __ICUB_VISLAB__VISLAB_SIMPLEHOMOGRAPHY_H_
#define __ICUB_VISLAB__VISLAB_SIMPLEHOMOGRAPHY_H_

#include <stdexcept>

#include <vislab/yarp/all.h>

#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrlMath.h>

namespace vislab {
namespace math {

struct CameraCalibration {
	double fx, fy;
	double cy, cx;
};

class SimpleHomography {
protected:
	/** A 3d offset to additionally take in account. */
	::yarp::sig::Vector offset;
	/** The homography projecting the 2D coordinates. */
	::yarp::sig::Matrix H;
	/** Transformation from the table to the robot's base coordinates. */
	::yarp::sig::Matrix T_rt;
	/** Transformation from the camera to the robot's base coordinates. */
	::yarp::sig::Matrix T_cr;

	/** A offset according the z-axis starting at the projection plane. */
	double heightOffset;

	/**
	 * An indicator if it is necessary to recompute the internal transformation
	 * because of changed parameters.
	 */
	bool forceRecomputation;

	/** The camera's calibration. */
	CameraCalibration cameraCalibration;

public:
	/**
	 * The constructor.
	 * @param c The camera's calibration.
	 * @param offset The additional 3D offset relative to the robot's base coordinates.
	 */
	SimpleHomography(const CameraCalibration& c, const ::yarp::sig::Vector* const offset = NULL);

	/**
	 * The destructor.
	 */
	virtual ~SimpleHomography();


	/**
	 * Sets a additional offset according the z-axis starting at the projection plane.
	 * @param d The offset to be set.
	 * @param forceRecompuation An indicator if the transformation should be immediately
	 * recomputed or not.
	 */
	void setHeightOffset(double d, bool forceRecompuation = true);

	/**
	 * Sets the base transformation for the camera to the robot's base coordinates.
	 * @param T Transformation from the camera to the robot's base coordinates.
	 */
	void setBaseTransformation(const ::yarp::sig::Matrix& T);

	/**
	 * Projects the given 2D point to the projection plane.
	 * @param in The 2D input coordinates to be projected.
	 * @param out The 3D output of {@var in}.
	 */
	void project(const ::yarp::sig::Vector& in, ::yarp::sig::Vector& out);

};

}
}

#endif /* __ICUB_VISLAB__VISLAB_SIMPLEHOMOGRAPHY_H_ */
