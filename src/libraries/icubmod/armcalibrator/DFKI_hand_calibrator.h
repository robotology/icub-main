// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_calibrators
 * \defgroup DFKI_hand_calibrator DFKI_hand_calibrator 
 *
 * Implement calibration routines for the iCub arm(s) (vesion 1.0).
 *
 * Copyright (C) 2007 RobotCub Consortium.
 *
 * Authors: Lorenzo Natale, Francesco Nori and Marco Maggiali
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/modules/armcalibrator/DFKI_hand_calibrator.h
 */

#ifndef __DFKI_HAND_CALIBRATOR__
#define __DFKI_HAND_CALIBRATOR__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CalibratorInterfaces.h>
#include <yarp/dev/ControlBoardInterfaces.h>

namespace yarp {
    namespace dev
    {
        class DFKI_hand_calibrator;
    }
}

/**
 * \file DFKI_hand_calibrator.h 
 * A device driver to implement the calibration of the Arm of iCub.
 */

/**
 * @ingroup dev_impl
 * 
 * A calibrator interface implementation for the Arm of the robot iCub.
 */
class yarp::dev::DFKI_hand_calibrator : public ICalibrator, public DeviceDriver
{
public:
    /**
     * Default constructor.
     */
    DFKI_hand_calibrator();
    
    /**
     * Destructor.
     */
    ~DFKI_hand_calibrator();

    /**
     * Calibrate method. Call this to calibrate the complete device.
     * @param dd is a pointer to the DeviceDriver to calibrate which must implement
     * the position control interfaces of the standard control board devices.
     * @return true if calibration was successful, false otherwise.
     */
    bool calibrate(DeviceDriver *dd);

    /**
     * Open the device driver.
     * @param config is a reference to a Searchable object which contains the initialization
     * parameters.
     * @return true/false on success/failure.
     */
	virtual bool open (yarp::os::Searchable& config);

    /**
     * Close the device driver.
     * @return true/false on success/failure.
     */
	virtual bool close ();

    virtual bool park(DeviceDriver *dd, bool wait=true);

    virtual bool quitPark();

    virtual bool quitCalibrate();

private:
    void calibrateJoint(int j);
    void goToZero(int j);
    bool checkCalibrateJointEnded(int j);
    void checkGoneToZero(int j);
	bool checkGoneToZeroThreshold(int j);

    IControlCalibration2 *iCalibrate;
    IPidControl *iPids;
    IEncoders *iEncoders;
    IPositionControl *iPosition;
	IControlMode2 *iControlMode;

    unsigned char *type;
	double *param1;
	double *param2;
	double *param3;
	double *pos;
	double *vel;
	Pid    *original_pid;
	Pid    *limited_pid;
    double *homeVel;
    double *homePos;
    bool abortCalib;
    bool abortParking;
};
#endif
