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

#ifndef __ICUB_VISLAB_HAND_H_
#define __ICUB_VISLAB_HAND_H_

#include "HandMetrics.h"

#include <vislab/util/all.h>
#include <vislab/yarp/util/all.h>
#include <vislab/yarp/all.h>

#include <iostream>
#include <set>
#include <map>

#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>
#include <yarp/dev/all.h>
#include <yarp/os/all.h>

namespace vislab {
namespace control {

/**
 * Provides an abstraction layer for the iCub's hand.
 *
 * @author Christian Wressnegger
 * @date 2009
 */
class Hand {

	static const int proximalJoints[];
	static const int distalJoints[];
	static const int thumbJoints[];
	static const int allFingerJoints[];
	static const int allButThumb[];
	static const int completeHand[];

	/** The speeds of the control before they were changed by this class. */
	double prevSpeed[HandMetrics::numAxes];

protected:

	/** Provides several metrics for this hand. */
	HandMetrics* handMetrics;

	/** The control board of the corresponding arm. */
	::yarp::dev::PolyDriver& controlBoard;
	/** The encoders of the hand. */
	::yarp::dev::IEncoders* encoders;
	/** The PID controller of the hand. */
	::yarp::dev::IPidControl* pidControl;
	/** The position controller of the hand. */
	::yarp::dev::IPositionControl* posControl;

	/** A lock for modifying the sets of enabled and disabled joints. */
	::yarp::os::Semaphore mutex;
	/** The set of disabled joints. */
	std::set<int> disabledJoints;
	/** The set of enabled joints. */
	std::set<int> enabledJoints;

	/**
	 * This function is supposed to create a {@link HandMetrics} implementation. Override its
	 * in order to change the {@link HandMetrics} used by this {@link Hand}.
	 */
	virtual void defineHandMetrics();

	/**
	 * Indicates if the motion that was specified was completed or not.
	 * @param joints The set of joints to be checked.
	 * @return An indicator if the motion that was specified was completed or not.
	 */
	virtual bool motionDone(const std::set<int> joints = COMPLETE_HAND);
	/**
	 * Stops the motion of joints that are "blocked". If so those joints will be removed
	 * from the set of active joints.
	 * @param activeJoints The set of active joints.
	 */
	virtual void stopBlockedJoints(std::set<int>& activeJoints);

public:
	/**
	 * Identifiers for joints of the hand according to http://eris.liralab.it/wiki/ICub_joints
	 */
	typedef enum hand_joints_t {
		WRIST_PROSUP = 4, WRIST_PITCH, WRIST_YAW, HAND_FINGER, /* 7 */
		THUMB_OPPOSE, THUMB_PROXIMAL, THUMB_DISTAL, /* 10 */
		INDEX_PROXIMAL, INDEX_DISTAL, MIDDLE_PROXIMAL, /* 13 */
		MIDDLE_DISTAL, PINKY
	} HandJoints;

	/** Lumps together all proximal joints of the hand. */
	static const std::set<int> PROXIMAL_JOINTS;
	/** Lumps together all distal joints of the hand. */
	static const std::set<int> DISTAL_JOINTS;
	/** Lumps together all thumb joints of the hand. */
	static const std::set<int> THUMB_JOINTS;
	/** Lumps together all joints of the hand that correspond to fingers. */
	static const std::set<int> ALL_FINGER_JOINTS;
	/** Lumps together all joints of the hand that correspond to fingers except the thumb. */
	static const std::set<int> ALL_BUT_THUMB;
	/** Lumps together all joints of the hand. */
	static const std::set<int> COMPLETE_HAND;

	/**
	 * The constructor.
	 * @param controlBoard The control board of the hand.
	 */
	Hand(::yarp::dev::PolyDriver& controlBoard);

	/**
	 * The destructor.
	 */
	virtual ~Hand();

	/**
	 * Returns the metrics object of this hand.
	 * @return The metrics object of this hand.
	 */
	HandMetrics& getMetrics();

	/**
	 * @deprecated [Use setEnable(const std::vector<int>&, const bool) instead]
	 */
	void disableJoints(const std::vector<int>& joints);
	/**
	 * @deprecated [Use setEnable(const std::vector<int>&, const bool) instead]
	 */
	void enableJoints(const std::vector<int>& joints);
	/**
	 * Enables or disables the specified joints according to the given boolean value.
	 * @param joints The IDs of the joints to be dis-/enabled.
	 * @param b The flag (default: true).
	 */
	void setEnable(const std::vector<int>& joints, bool b = true);
	/**
	 * Populates the given {@link std::vector} with the IDs of the currently disabled joints.
	 * @param joints The resulting {@link std::vector} holding the IDs of the disabled joints.
	 */
	void getDisabledJoints(std::vector<int>& joints);

	/**
	 * Sets the velocity of the given joint.
	 * @param d The velocity value.
	 * @param joint The joint to be effected.
	 */
	void setVelocity(const double d, const int joint);
	/**
	 * Sets the velocity of the given joints.
	 * @param d The velocity value.
	 * @param joints The joints to be effected (default: {@link COMPLETE_HAND}).
	 */
	void setVelocity(const double d, const std::set<int> joints = COMPLETE_HAND);
	/**
	 * Sets the velocity of the given joint.
	 * @param v The velocity values as {@link ::yarp::sig::Vector} of size 16 (number of joints of the arm)
	 * @param joint The joint to be effected.
	 */
	void setVelocity(const ::yarp::sig::Vector& v, const int joint);
	/**
	 * Sets the velocity of the given joints.
	 * @param v The velocity values as {@link ::yarp::sig::Vector} of size 16 (number of joints of the arm)
	 * @param joint The joints to be effected.
	 */
	void setVelocity(const ::yarp::sig::Vector& v, const std::set<int> joints = COMPLETE_HAND);

	/**
	 * Move the specified joints to the given position.
	 * @param v The position to move to  as {@link ::yarp::sig::Vector} of size 16 (number of joints of the arm).
	 * @param joints The joints to be moved (default: {@link COMPLETE_HAND}).
	 * @return Indicates if the command for moving the joints was successfully set.
	 */
	bool move(const ::yarp::sig::Vector& v, const std::set<int> joints = COMPLETE_HAND);
	/**
	 * Move the specified joint according to the given motion specification.
	 * @param m The motion specification to follow.
	 * @param joint The joint to be moved.
	 * @return Indicates if the command for moving the joints was successfully set.
	 */
	bool move(const vislab::yarp::util::Motion& m, const int joint);
	/**
	 * Move the specified joints according to the given motion specification.
	 * @param m The motion specification to follow.
	 * @param joints The joints to be moved (default: {@link COMPLETE_HAND}).
	 * @return Indicates if the command for moving the joints was successfully set.
	 */
	bool move(const vislab::yarp::util::Motion& m, const std::set<int> joints = COMPLETE_HAND);
	/**
	 * Move the specified joint to the given positions (one row after the other).
	 * @param m The positions to move to as {@link ::yarp::sig::Matrix} with 16 (number of joints of the arm) columns.
	 * @param joint The joint to be moved.
	 * @param invert Invert the specified motion.
	 * @return Indicates if the command for moving the joints was successfully set.
	 */
	bool move(const ::yarp::sig::Matrix& m, const int joint, const bool invert = false);
	/**
	 * Move the hand to the given positions (one row after the other).
	 * @param m The positions to move to as {@link ::yarp::sig::Matrix} with 16 (number of joints of the arm) columns.
	 * @param invert Invert the specified motion.
	 * @return Indicates if the command for moving the joints was successfully set.
	 */
	bool move(const ::yarp::sig::Matrix& m, const bool invert = false);
	/**
	 * Move the specified joints to the given positions (one row after the other).
	 * @param m The positions to move to as {@link ::yarp::sig::Matrix} with 16 (number of joints of the arm) columns.
	 * @param joints The joints to be moved.
	 * @param invert Invert the specified motion.
	 * @return Indicates if the command for moving the joints was successfully set.
	 */
	bool move(const ::yarp::sig::Matrix& m, const std::set<int> joints, const bool invert = false);
	/**
	 * Move the hand to the given motion sequence.
	 * @param seq The motion sequence to follow.
	 * @param invert Invert the specified motion.
	 * @return Indicates if the command for moving the joints was successfully set.
	 */
	bool move(const vislab::yarp::util::MotionSequence& seq, const bool invert = false);
	/**
	 * Move the specified joint to the given positions (one row after the other).
	 * @param seq The motion sequence to follow.
	 * @param joint The joint to be moved.
	 * @param invert Invert the specified motion.
	 * @return Indicates if the command for moving the joints was successfully set.
	 */
	bool move(const vislab::yarp::util::MotionSequence& seq, const int joint, const bool invert =
			false);
	/**
	 * Move the specified joints according to the given motion sequence.
	 * @param seq The motion sequence to follow.
	 * @param joints The joints to be moved.
	 * @param invert Invert the specified motion.
	 * @return Indicates if the command for moving the joints was successfully set.
	 */
	bool move(const vislab::yarp::util::MotionSequence& seq, const std::set<int> joints,
			const bool invert = false);
};

}
}

#endif /* __ICUB_VISLAB_HAND_H_ */
