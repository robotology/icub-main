// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* Copyright (C) 2013  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Alberto Cardellino
 * email: alberto.cardellino@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __3DM_GX3__
#define __3DM_GX3__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/RateThread.h>
#include <string.h>
#include <dataTypes.h>
#include <termios.h> // terminal io (serial port) interface


namespace yarp{
    namespace dev{
        class imu3DM_GX3;
    }
}

struct XSensMTxParameters
{
    std::string comPortString;
    short comPort;
};

/**
 *
 * Driver for 3DM_GX3 IMU unit from MicroStrain
 * @author Alberto Cardellino
 */
class yarp::dev::imu3DM_GX3 :  public yarp::dev::IGenericSensor,
                                public yarp::dev::IPreciselyTimed,
                                public yarp::dev::DeviceDriver,
                                public yarp::os::RateThread
{
private:

    int                 fd_ser;
    int                 baudrate;
    int                 nchannels;

    // data specific for this imu
    typedef std::map<int, imu_cmd_t*> cmd_map_t;
    cmd_map_t cmd_ptr_map;

    // Data structure specific for each command
    imu_cmd_t C2_cmd;
    imu_cmd_t C8_cmd;
    imu_cmd_t CB_cmd;
    imu_cmd_t CC_cmd;
    imu_cmd_t CE_cmd;
    imu_cmd_t CF_cmd;
    imu_cmd_t DF_cmd;

    yarp::os::Semaphore data_mutex;
    yarp::os::Semaphore sync_mutex;

    std::string         comPortName;
    data_3DM_GX3_t      rawData;
    yarp::os::Stamp     lastStamp;

public:
    imu3DM_GX3();
    ~imu3DM_GX3();

    // IGenericSensor interface.
    virtual bool read(yarp::sig::Vector &out);
    virtual bool getChannels(int *nc);
    virtual bool open(yarp::os::Searchable &config);
    virtual bool calibrate(int ch, double v);
    virtual bool close();
    void sample_setting(void);

    virtual yarp::os::Stamp getLastInputStamp();

    // Open the device
    bool open(const XSensMTxParameters &par);

private:
    bool threadInit();
    void run();
    void threadRelease();

private:      // Device specific

    void stop_continuous(void);
    // command 0xC2
    void get_Acc_Ang(float acc[3], float angRate[3], uint64_t *time);
    // command 0xC8
    void get_Acc_Ang_Orient(float acc[3], float angRate[3], float orientMat[9], uint64_t *time);
    // command 0xCB
    void get_Acc_Ang_Mag(float acc[3], float angRate[3], float mag[3], uint64_t *time);
    // command 0xCC
    void get_Acc_Ang_Mag_Orient(float acc[3], float angRate[3], float mag[3], float orientMat[9], uint64_t *time);
    // command 0xCE
    void get_Euler(float euler[3], uint64_t *time);
    // command 0xCF
    void get_Euler_AngularRate(float euler[3], float angRate[3], uint64_t *time);
    // command 0xDF
    void get_Quaternion(float quat[4], uint64_t *time);

    // serial handling
    inline int flush() {return tcflush(fd_ser, TCIOFLUSH);
    }
};


#endif  // __3DM_GX3__
