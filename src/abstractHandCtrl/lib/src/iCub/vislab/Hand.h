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

/**
 * @defgroup libHandCtrl
 *
 * @ingroup icub_module
 *
 * Classes for controlling the hands of the iCub.
 *
 * \section lib_sec Dependencies
 *
 * - YARP (YARP_{OS,dev,sig,math})
 * - ACE
 * - OpenVislab (libvislab, libvislab_YARP): http://OpenVislab.sf.net
 *
 *
 * @author Christian Wressnegger
 * @date 2009
 *
 */

#ifndef __ICUB_VISLAB_HAND_H_
#define __ICUB_VISLAB_HAND_H_

#include "HandMetrics.h"

#include <vislab/util/all.h>
#include <vislab/yarp/util/all.h>
#include <vislab/yarp/all.h>

#include <iostream>
#include <set>
#include <map>
#include <cmath>
#include <stdexcept>

#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>
#include <yarp/dev/all.h>
#include <yarp/os/all.h>

namespace vislab {
namespace control {

/**
 * @ingroup libHandCtrl
 *
 * Provides an abstraction layer of the iCub's hands.
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

	static const int limits[][2];

	static const std::vector<std::pair<int, int> > initLimits(const int arr[][2], size_t len);
	static const std::vector<int> initRanges(const std::vector<std::pair<int, int> > v);

	/**
	 * This class implements a {@link RateThread} for recording the motions of the {@link Hand}.
	 *
	 * @author Christian Wressnegger
	 * @date 2009
	 */
	class Recorder: public ::yarp::os::RateThread {

		vislab::yarp::util::MotionSequence recording;
		vislab::yarp::util::MotionSequence curRecording;

		HandMetrics& handMetrics;

	public:
		/**
		 * The constructor.
		 * @param handMetric The metrics object for the {@link Hand}.
		 * @param period The time interval the motions should be sampled.
		 */
		Recorder(HandMetrics& handMetric, int period = 100);
		/**
		 * The destructor.
		 */
		virtual ~Recorder();
		/**
		 * @see RateThread#run()
		 */
		virtual void run();
		/**
		 * @see RateThread#start()
		 */
		bool start();
		/**
		 * @see RateThread#stop()
		 */
		void stop();

		/**
		 * Returns the last finished recording of the {@link Hand}.
		 * @return The last finished recording of the {@link Hand}.
		 */
		vislab::yarp::util::MotionSequence getRecording();
	};

	class JointMonitor: public ::yarp::os::RateThread {

		Hand& hand;
		//std::set<int> monitoredJoints;
		std::set<int> blockedJoints;

		//ACE_Auto_Event motionDoneEvent;
		::yarp::os::Semaphore lock;

	public:
		/**
		 * The constructor.
		 * @param hand The {@link Hand} to be monitored.
		 * @param period The time interval the joints should be checked.
		 */
		JointMonitor(Hand& hand, int period = 100);
		/**
		 * The destructor.
		 */
		virtual ~JointMonitor();
		/**
		 * @see RateThread#run()
		 */
		virtual void run();
		/**
		 * @see RateThread#start()
		 */
		bool start();
		/**
		 * @see RateThread#stop()
		 */
    void stop();

		//void monitor(std::set<int> joints, bool b = true);

		const std::set<int>& getBlockedJoints() const;

		void waitMotionDone();

	};
	friend class JointMonitor;

	/** The speeds of the control before they were changed by this class. */
	double prevSpeed[HandMetrics::numAxes];

	bool recording;
	vislab::yarp::util::MotionSequence recordedSequence;
	Recorder* recorder;
	JointMonitor* jointMonitor;

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
	 * Stops the motion of joints that are "blocked". If so those joints will be added to
	 * the set of blocked joints. Stopping motions is done by calling
	 * @param blockedJoints The set of blocked joints.
	 */
	virtual void stopBlockedJoints(std::set<int>* const blockedJoints = NULL);

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

	/** The limits of the hand's joints: (min, max) degrees. */
	static const std::vector<std::pair<int, int> > LIMITS;

	/** The range of the hand's joints. */
	static const std::vector<int> RANGES;

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
	 * Move the hand to the given position.
	 * @param v The position to move to  as {@link ::yarp::sig::Vector} of size 16 (number of joints of the arm).
	 * @return Indicates if the command for moving the joints was successfully set.
	 */
	bool move(const ::yarp::sig::Vector& v, const bool sync = true);

	/**
	 * Move the specified joints to the given position.
	 * @param v The position to move to  as {@link ::yarp::sig::Vector} of size 16 (number of joints of the arm).
	 * @param joints The joints to be moved.
	 * @return Indicates if the command for moving the joints was successfully set.
	 */
	bool move(const ::yarp::sig::Vector& v, const std::set<int> joints, const bool sync = true);
	/**
	 * Move the specified joint according to the given motion specification.
	 * @param m The motion specification to follow.
	 * @param joint The joint to be moved.
	 * @return Indicates if the command for moving the joints was successfully set.
	 */
	bool move(const vislab::yarp::util::Motion& m, const int joint, const bool sync = true);
	/**
	 * Move the hand according to the given motion specification.
	 * @param m The motion specification to follow.
	 * @param joints The joints to be moved (default: {@link COMPLETE_HAND}).
	 * @return Indicates if the command for moving the joints was successfully set.
	 */
	bool move(const vislab::yarp::util::Motion& m, const bool sync = true);
	/**
	 * Move the specified joints according to the given motion specification.
	 * @param m The motion specification to follow.
	 * @param joints The joints to be moved.
	 * @return Indicates if the command for moving the joints was successfully set.
	 */
	bool move(const vislab::yarp::util::Motion& m, const std::set<int> joints, const bool sync = true);
	/**
	 * Move the specified joint to the given positions (one row after the other).
	 * @param m The positions to move to as {@link ::yarp::sig::Matrix} with 16 (number of joints of the arm) columns.
	 * @param joint The joint to be moved.
	 * @param invert Invert the specified motion.
	 * @return Indicates if the command for moving the joints was successfully set.
	 */
	bool move(const ::yarp::sig::Matrix& m, const int joint, const bool sync = true, const bool invert = false);
	/**
	 * Move the hand to the given positions (one row after the other).
	 * @param m The positions to move to as {@link ::yarp::sig::Matrix} with 16 (number of joints of the arm) columns.
	 * @param invert Invert the specified motion.
	 * @return Indicates if the command for moving the joints was successfully set.
	 */
	bool move(const ::yarp::sig::Matrix& m, const bool sync = true, const bool invert = false);
	/**
	 * Move the specified joints to the given positions (one row after the other).
	 * @param m The positions to move to as {@link ::yarp::sig::Matrix} with 16 (number of joints of the arm) columns.
	 * @param joints The joints to be moved.
	 * @param invert Invert the specified motion.
	 * @return Indicates if the command for moving the joints was successfully set.
	 */
	bool move(const ::yarp::sig::Matrix& m, const std::set<int> joints, const bool sync = true, const bool invert = false);
	/**
	 * Move the hand to the given motion sequence.
	 * @param seq The motion sequence to follow.
	 * @param invert Invert the specified motion.
	 * @return Indicates if the command for moving the joints was successfully set.
	 */
	bool move(const vislab::yarp::util::MotionSequence& seq, const bool sync = true, const bool invert = false);
	/**
	 * Move the specified joint to the given positions (one row after the other).
	 * @param seq The motion sequence to follow.
	 * @param joint The joint to be moved.
	 * @param invert Invert the specified motion.
	 * @return Indicates if the command for moving the joints was successfully set.
	 */
	bool move(const vislab::yarp::util::MotionSequence& seq, const int joint, const bool sync = true, const bool invert =
			false);
	/**
	 * Move the specified joints according to the given motion sequence.
	 * @param seq The motion sequence to follow.
	 * @param joints The joints to be moved.
	 * @param invert Invert the specified motion.
	 * @return Indicates if the command for moving the joints was successfully set.
	 */
	bool move(const vislab::yarp::util::MotionSequence& seq, const std::set<int> joints, const bool sync = true,
			const bool invert = false);

	/**
	 * This function allows one to dis-/ enable the
	 * @param b
	 */
	void doControlledMovements(const bool b = true);

	/**
	 * En-/Disables the recording for the {@link Hand}s movements.
	 * @param b The indicator to decide if we start or stop the recording.
	 * @return An indicator for if the state was successfully changed or not.
	 */
	bool record(const bool b = true);
	/**
	 * Returns if the movements of the {@link Hand} are currently recorded or not.
	 * @return If the movements of the {@link Hand} are currently recorded or not.
	 */
	bool isRecording();
	/**
	 * Returns the last finished recording of the {@link Hand}.
	 * @return The last finished recording of the {@link Hand}.
	 */
	vislab::yarp::util::MotionSequence getRecording();
	/**
	 * Set the sampling rate for recording hand movements.
	 * @param t The sampling rate as milliseconds.
	 */
	void setSamplingRate(int t);
	/**
	 * Returns the sampling rate which is used for recording {@link Hand} movements.
	 * @return The sampling rate which is used for recording {@link Hand} movements.
	 */
	int getSamplingRate();

	void setMonitorRate(int t);
};

}
}

#endif /* __ICUB_VISLAB_HAND_H_ */
