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

#ifndef __VISLAB_YARP_UTIL_MOTIONSEQUENCE__H_
#define __VISLAB_YARP_UTIL_MOTIONSEQUENCE__H_

#include <vector>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>

#include "common.h"

namespace vislab {
namespace yarp {
namespace util {

/**
 * Represents a single motion (position, velocity, timing) as part of a sequence.
 *
 * @author Christian Wressnegger
 * @date 2009
 */
class Motion {

	bool isRelative;
	::yarp::sig::Vector position;
	::yarp::sig::Vector velocity;
	double timing;

	int numJoints;
	bool dynamicallyAdjustVectorSizes;

	void adjustVectorSizes();

public:
	/**
	 * The constructor.
	 */
	Motion();
	/**
	 * The constructor.
	 * @param numJoints The number of joint to be moved in the scope of this motion.
	 */
	Motion(unsigned int numJoints);
	/**
	 * The destructor.
	 */
	virtual ~Motion();

	/**
	 * Sets the position of the motion. Once the number of joints is fixed (MotionSequence#setNumJoints(const int))
	 * one is only able to pass position vectors of the specified number of joints.
	 * @param v The position vector
	 * @param isRelative Specifies if the position is relative.
	 */
	void setPosition(const ::yarp::sig::Vector& v, const bool isRelative = false);
	/**
	 * Sets the velocity of the motion. Once the number of joints is fixed (MotionSequence#setNumJoints(const int))
	 * one is only able to pass velocity vectors of the specified number of joints.
	 * @param v The velocity vector.
	 */
	void setVelocity(const ::yarp::sig::Vector& v);
	/**
	 * Sets the timing value of the motion.
	 * @param d The timing value.
	 */
	void setTiming(const double d);

	/**
	 * Returns the position of the motion.
	 * @return The position of the motion.
	 */
	const ::yarp::sig::Vector& getPosition() const;
	/**
	 * Returns the velocity of the motion.
	 * @return The velocity of the motion.
	 */
	const ::yarp::sig::Vector& getVelocity() const;
	/**
	 * Returns the timing constant of the motion.
	 * @return The timing constant of the motion.
	 */
	const double getTiming() const;

	/**
	 * Sets the number of joints.
	 * @param n The number of joints.
	 */
	void setNumJoints(const int n);
	/**
	 * Returns the number of Joints.
	 * @return The number of Joints.
	 */
	const int getNumJoints() const;

	/**
	 * Returns a human and especially machine readable representation of the {@link MotionSequence}.
	 * The format follows the way the robotMotorGui of the RobotCub project encodes motions:
	 * http://eris.liralab.it/iCub/dox/html/group__icub__robotMotorGui.html
	 * @return A human and especially machine readable representation of the {@link MotionSequence}.
	 */
	::yarp::os::ConstString toString();
};

/**
 * Represent a sequence of motions. Therefore it makes use of {@link Motion} objects.
 *
 * @author Christian Wressnegger
 * @date 2009
 */
class MotionSequence {

	std::vector<Motion> motionSequence;
	int numJoints;
	bool dynamicallyAdjustMotionSizes;

	void adjustMotionSizes();

public:
	/**
	 * The constructor.
	 */
	MotionSequence();
	/**
	 * The constructor.
	 * @param numJoints The number of joint to be moved in the scope of this motion.
	 */
	MotionSequence(unsigned int numJoints);

	/**
	 * Add a new {@link Motion} specification to the sequence. Once the number of joints is fixed
	 * (MotionSequence#setNumJoints(const int)) one is only able to pass {@link Motion} specifications
	 * with the specified number of joints.
	 * @param m The motion to be added.
	 */
	void addMotion(Motion& m);

	/**
	 * Sets the number of joint of the sequence.
	 * @param n The number of joints.
	 */
	void setNumJoints(const int n);
	/**
	 * Returns the number of joints of the sequence.
	 * @return The number of joints of the sequence.
	 */
	const int getNumJoints() const;
	/**
	 * Returns the {@link Motion} specification at the specified position.
	 * @param i The position of the {@link Motion} specification.
	 * @return The {@link Motion} specification at the specified position.
	 */
	Motion& operator[](const size_t i);
	/**
	 * Returns the {@link Motion} specification at the specified position.
	 * @param i The position of the {@link Motion} specification.
	 * @return The {@link Motion} specification at the specified position.
	 */
	const Motion& operator[](const size_t i) const;
	/**
	 * Returns the length of the sequence.
	 * @return The length of the sequence.
	 */
	const size_t length() const;
	/**
	 * Removes all {@link Motion}s stored in this {@link MotionSequence}.
	 */
	void clear();

	/**
	 * Extracts a sub-sequence of the current one. This sequence will <strong>include</strong>
	 * the given indices.
	 * @param start The start index.
	 * @param end The end index.
	 * @return The generated sub-sequence of length max(0, end - start) +1
	 */
	MotionSequence subsequence(const size_t start, const size_t end);

	/**
	 * Returns a human and especially machine readable representation of the {@link MotionSequence}.
	 * The format follows the way the robotMotorGui of the RobotCub project encodes motions:
	 * http://eris.liralab.it/iCub/dox/html/group__icub__robotMotorGui.html
	 * @return A human and especially machine readable representation of the {@link MotionSequence}.
	 */
	::yarp::os::ConstString toString();

	/**
	 * Writes the string obtained by toString() to a file with the given name.
	 * @param filename The name of the file.
	 * @return A success flag.
	 */
	bool toFile(::yarp::os::ConstString filename);

	/** A constant iterator of the sequence. */
	typedef std::vector<Motion>::const_iterator const_iterator;
	/** An iterator of the sequence. */
	typedef std::vector<Motion>::iterator iterator;
	/** A constant reverse iterator of the sequence. */
	typedef std::vector<Motion>::const_reverse_iterator const_reverse_iterator;
	/** A reverse iterator of the sequence. */
	typedef std::vector<Motion>::reverse_iterator reverse_iterator;

	/**
	 * Returns a constant iterator to the beginning of the sequence.
	 * @return A constant iterator to the beginning of the sequence.
	 */
	const_iterator begin() const {
		return motionSequence.begin();
	}

	/**
	 * Returns an iterator to the beginning of the sequence.
	 * @return An iterator to the beginning of the sequence.
	 */
	iterator begin() {
		return motionSequence.begin();
	}

	/**
	 * Returns a constant iterator to the end of the sequence.
	 * @return A constant iterator to the end of the sequence.
	 */
	const_iterator end() const {
		return motionSequence.end();
	}

	/**
	 * Returns a iterator to the end of the sequence.
	 * @return A iterator to the end of the sequence.
	 */
	iterator end() {
		return motionSequence.end();
	}

	/**
	 * Returns a constant reverse iterator to the beginning of the sequence.
	 * @return A constant reverse iterator to the beginning of the sequence.
	 */
	const_reverse_iterator rbegin() const {
		return motionSequence.rbegin();
	}

	/**
	 * Returns a reverse iterator to the beginning of the sequence.
	 * @return A reverse iterator to the beginning of the sequence.
	 */
	reverse_iterator rbegin() {
		return motionSequence.rbegin();
	}

	/**
	 * Returns a constant reverse iterator to the end of the sequence.
	 * @return A constant reverse iterator to the end of the sequence.
	 */
	const_reverse_iterator rend() const {
		return motionSequence.rend();
	}

	/**
	 * Returns a reverse iterator to the end of the sequence.
	 * @return A reverse iterator to the end of the sequence.
	 */
	reverse_iterator rend() {
		return motionSequence.rend();
	}
};

}
}
}

#endif /* __VISLAB_YARP_UTIL_MOTIONSEQUENCE__H_ */
