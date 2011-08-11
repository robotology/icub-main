// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_calibrators
 * \defgroup icubheadcalibratorV2 icubheadcalibratorV2
 *
 * Implement calibration routines for the iCub head version 2.0 .
 *
 * Copyright (C) 2007 RobotCub Consortium.
 *
 * Authors: Lorenzo Natale, Francesco Nori and Marco Maggiali 
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/modules/headcalibratorV2/iCubHeadCalibratorV2.h
 */

#ifndef __ICUB_HEAD_CALIBRATORV2__
#define __ICUB_HEAD_CALIBRATORV2__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CalibratorInterfaces.h>
#include <yarp/dev/ControlBoardInterfaces.h>

namespace yarp {
    namespace dev
    {
        class iCubHeadCalibratorV2;
    }
}

/**
 * \file iCubHeadCalibratorV2.h 
 * A device driver to implement the calibration of the head of iCub.
 */

/**
 * @ingroup dev_impl
 * 
 * A calibrator interface implementation for the head of the robot iCub.
 */
class yarp::dev::iCubHeadCalibratorV2 : public ICalibrator, public DeviceDriver
{
public:
    /**
     * Default constructor.
     */
    iCubHeadCalibratorV2();
    
    /**
     * Destructor.
     */
    ~iCubHeadCalibratorV2();

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
    IAmplifierControl *iAmps;
    IPidControl *iPids;
    IEncoders *iEncoders;
    IPositionControl *iPosition;
	IControlMode *iControlMode;

    unsigned char *type;
	int    canID;
	double *param1;
	double *param2;
	double *param3;
	int    *maxPWM;
    double *currPos;
    double *currVel;
	Pid    *original_pid;
	Pid    *limited_pid;
    double *zeroVel;
	double *zeroPos;
    double *homeVel;
    double *homePos;
    bool abortCalib;
    bool abortParking;
};
#endif
