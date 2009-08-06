// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup icubtorsoonlycalibrator icubtorsoonlycalibrator
 *
 * Implement calibration routines for the iCub torso. Notice: this 
 * class calibrates only the torso of the robot, written and useful
 * for robots with no head. In normal circumstances use only iCubHeadCalibrator
 * which takes care of calibrating both the head and the torso in the 
 * same routine.
 *
 * Copyright (C) 2009 RobotCub Consortium.
 *
 * Authors: Lorenzo Natale, Francesco Nori and Marco Maggiali 
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __ICUB_TORSOONLY_CALIBRATOR__
#define __ICUB_TORSOONLY_CALIBRATOR__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CalibratorInterfaces.h>
#include <yarp/dev/ControlBoardInterfaces.h>

namespace yarp {
    namespace dev
    {
        class iCubTorsoOnlyCalibrator;
    }
}

/**
 * \file iCubTorsoOnlyCalibrator.h 
 * A device driver to implement the calibration of the torso of iCub.
 * Useful only when you want to skip calibration of the head. In normal
 * cirumstances you will be using iCubHeadCalibrator which performs 
 * calibration of both the head and the torso at the same time.
 */

class yarp::dev::iCubTorsoOnlyCalibrator : public ICalibrator, public DeviceDriver
{
public:
    /**
     * Default constructor.
     */
    iCubTorsoOnlyCalibrator();
    
    /**
     * Destructor.
     */
    ~iCubTorsoOnlyCalibrator();

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

    IControlCalibration2 *iCalibrate;
    IAmplifierControl *iAmps;
    IPidControl *iPids;
    IEncoders *iEncoders;
    IPositionControl *iPosition;

    unsigned char *type;
	double *param1;
	double *param2;
	double *param3;
	double *pos;
	double *vel;

    double *homePos;
    double *homeVel;

    bool abortParking;
    bool abortCalib;
};
#endif
