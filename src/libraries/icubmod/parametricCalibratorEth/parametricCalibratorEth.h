// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 20014 iCub Facility, Istituto Italiano di Tecnologia
 *
 * Authors: Alberto Cardellino, Marco Randazzo, Valentina Gaggero
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#ifndef __ICUB_PARAMETRIC_CALIBRATOR__
#define __ICUB_PARAMETRIC_CALIBRATOR__

#include <list>
#include <string>
#include <atomic>
#include <vector>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CalibratorInterfaces.h>
#include <yarp/dev/ControlBoardInterfaces.h>

namespace yarp {
    namespace dev
    {
        class parametricCalibratorEth;
    }
}

/**
 * \file eo_iCubArmCalibratorJ8.h
 * A device driver to implement the calibration of the Arm of iCub.
 */

/**
 * @ingroup icub_calibrators
 * @brief `parametricCalibrator`: implement calibration routines for the iCub arm(s) (version 1.2).
 * 
 * A calibrator interface implementation for the Arm of the robot iCub.
 */
class yarp::dev::parametricCalibratorEth : public ICalibrator, public DeviceDriver, public IRemoteCalibrator
{
public:
    /**
     * Default constructor.
     */
    parametricCalibratorEth();

    /**
     * Destructor.
     */
    ~parametricCalibratorEth();

    /**
     * Calibrate method. Call this to calibrate the complete device.
     * @param dd is a pointer to the DeviceDriver to calibrate which must implement
     * the position control interfaces of the standard control board devices.
     * @return true if calibration was successful, false otherwise.
     */
    bool calibrate(DeviceDriver *device) override;

    /**
     * Open the device driver.
     * @param config is a reference to a Searchable object which contains the initialization
     * parameters.
     * @return true/false on success/failure.
     */
    virtual bool open (yarp::os::Searchable& config) override;

    /**
     * Close the device driver.
     * @return true/false on success/failure.
     */
    virtual bool close ()  override;

    virtual bool park(DeviceDriver *dd, bool wait=true)  override;

    virtual bool quitPark()  override;

    virtual bool quitCalibrate()  override;

    // IRemoteCalibrator

    virtual  yarp::dev::IRemoteCalibrator *getCalibratorDevice()  override;

    virtual bool calibrateSingleJoint(int j)  override;

    virtual bool calibrateWholePart()  override;

    virtual bool homingSingleJoint(int j)  override;

    virtual bool homingWholePart()  override;

    virtual bool parkSingleJoint(int j, bool _wait=true)  override;

    virtual bool parkWholePart()  override;

private:

    // helper struct
    struct PositionSequence
    {
        int                 seq_num{0};         // progressive number indicating sequence ID
        std::vector<double> positions;          // vector of positions,  one for each joint
        std::vector<double> velocities;         // vector of velocities, one for each joint
    };

    bool calibrate();
    bool calibrateJoint(int j);
    bool goToStartupPosition(int j);
    bool checkCalibrateJointEnded(std::list<int> set);
    bool checkGoneToZeroThreshold(int j);
    bool checkHwFault(int j);

    yarp::dev::PolyDriver *dev2calibrate;
    yarp::dev::IControlCalibration *iCalibrate;
    yarp::dev::IPidControl *iPids;
    yarp::dev::IEncoders *iEncoders;
    yarp::dev::IPositionControl *iPosition;
    yarp::dev::IControlMode *iControlMode;
    yarp::dev::IAmplifierControl *iAmp;

    std::list<std::list<int> > joints;

    int n_joints;

    yarp::dev::CalibrationParameters* calibParams;
    int    *startupMaxPWM;
    double *currPos;
    double *currVel;
    double *original_max_pwm;
    double *limited_max_pwm;

    double *startupPosThreshold;
    PositionSequence legacyStartupPosition;     // upgraded old array to new struct; to be removed when old method will be deprecated
    PositionSequence legacyParkingPosition;     // upgraded old array to new struct; to be removed when old method will be deprecated

    bool    abortCalib;
    bool    abortParking;
    std::atomic<bool>    isCalibrated;
    bool    skipCalibration;
    int    *disableHomeAndPark;
    int    *disableStartupPosCheck;
    bool    clearHwFault;

    int    *timeout_goToZero;
    int    *timeout_calibration;

    int                 totJointsToCalibrate;
    std::string         deviceName;
    yarp::os::Bottle    calibJointsString;      // joints handled by this caibrator as a string for error messages
    std::list<int>      calibJoints;            // joints handled by this caibrator as a list

    // store the positions for a custom parking sequence
    int  currentParkingSeq_step;
    std::vector<PositionSequence> parkingSequence;
    bool useLegacyParking;                    // calibrator is using new parking sequence
    bool parseSequenceGroup(yarp::os::Searchable &config, std::string sequence, std::vector<PositionSequence> &seqList);
    bool moveAndCheck(PositionSequence &data);

    // to be removed
    bool moveAndCheck_legacy(PositionSequence &data, std::vector<bool> &cannotPark, bool wait);
};
#endif
