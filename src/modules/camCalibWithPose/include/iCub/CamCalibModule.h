// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __CAMCALIBMODULE__
#define __CAMCALIBMODULE__

// std
#include <mutex>
#include <stdio.h>
#include <map>

// opencv
#include <opencv2/core/core_c.h>

// yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

// iCub
#include <iCub/PinholeCalibTool.h>
#include <iCub/SphericalCalibTool.h>
#include <iCub/CalibToolFactory.h>
#include <iCub/ICalibTool.h>

/**
 *
 * Camera Calibration Port class
 *
 */
class CamCalibPort : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >
{
private:
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *portImgOut;
    ICalibTool     *calibTool;

    bool verbose;
    double t0;
    double currSat;
    double roll;
    double pitch;
    double yaw;
    std::mutex m;
    bool leftEye; // true for left eye, false for right eye
    double maxDelay;

    std::map<double, yarp::os::Bottle> m_h_encs_map;
    std::map<double, yarp::os::Bottle> m_t_encs_map;
    std::map<double, yarp::os::Bottle> m_imu_map;
    yarp::os::Bottle m_last_h_encs;
    yarp::os::Bottle m_last_t_encs;
    yarp::os::Bottle m_last_imu;
    yarp::os::Bottle m_curr_h_encs;
    yarp::os::Bottle m_curr_t_encs;
    yarp::os::Bottle m_curr_imu;
    bool useIMU;
    bool useTorso;
    bool useEyes;
    bool useLast;

    bool updatePose(double time);
    bool selectBottleFromMap(double time,
                             std::map<double, yarp::os::Bottle> *datamap,
                             yarp::os::Bottle *bottle,
                             bool verbose = false);
    virtual void onRead(yarp::sig::ImageOf<yarp::sig::PixelRgb> &yrpImgIn);
public:
    int filter_enable;
    double cf1;
    double cf2;
    double r_xv[2];
    double p_xv[2];
    double y_xv[2];
    double r_yv[2];
    double p_yv[2];
    double y_yv[2];

public:
    CamCalibPort();

    void setSaturation(double satVal);
    void setPointers(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *_portImgOut, ICalibTool *_calibTool);
    void setVerbose(const bool sw) { verbose=sw; }
    void setLeftEye(bool eye) { leftEye = eye; }
    void setMaxDelay(double delay) { maxDelay = delay; }

    void setTorsoEncoders(double time, const yarp::os::Bottle &t_encs) { m.lock();  m_last_t_encs = t_encs; m_t_encs_map[time] = t_encs; m.unlock(); }
    void setHeadEncoders(double time, const yarp::os::Bottle &h_encs) { m.lock(); m_last_h_encs = h_encs;  m_h_encs_map[time] = h_encs; m.unlock(); }
    void setImuData(double time, const yarp::os::Bottle &imu) { m.lock(); m_last_imu = imu; m_imu_map[time] = imu; m.unlock(); }
    void setUseIMU(bool useIMU) { this->useIMU = useIMU; }
    void setUseTorso(bool useTorso) { this->useTorso = useTorso; }
    void setUseEyes(bool useEyes) { this->useEyes = useEyes; }
    void setUseLast(bool useLast) { this->useLast = useLast; }

    yarp::os::BufferedPort<yarp::os::Bottle> rpyPort;
};


class TorsoEncoderPort : public yarp::os::BufferedPort<yarp::os::Bottle> {
private:
    virtual void onRead(yarp::os::Bottle &t_encs);
public:
    CamCalibPort *_prtImgIn;
};

class HeadEncoderPort : public yarp::os::BufferedPort<yarp::os::Bottle> {
private:
    virtual void onRead(yarp::os::Bottle &h_encs);
public:
    CamCalibPort *_prtImgIn;
};

class ImuPort : public yarp::os::BufferedPort<yarp::os::Bottle> {
private:
    virtual void onRead(yarp::os::Bottle &imu);
public:
    CamCalibPort *_prtImgIn;
    int imuCount;
};


/**
 *
 * Camera Calibration Module class
 *
 * \see icub_camcalib
 *
 */
class CamCalibModule : public yarp::os::RFModule {

private:

    CamCalibPort    _prtImgIn;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >  _prtImgOut;
    yarp::os::Port  _configPort;
    TorsoEncoderPort  _prtTEncsIn;
    HeadEncoderPort _prtHEncsIn;
    ImuPort _prtImuIn;
    ICalibTool *    _calibTool;
    std::string strGroup;

public:

    CamCalibModule();
    ~CamCalibModule();

    /** Passes config on to CalibTool */
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
    virtual bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    virtual double getPeriod();
};


#endif
