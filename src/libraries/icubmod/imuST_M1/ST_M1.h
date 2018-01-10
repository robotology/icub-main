// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* Copyright (C) 2015  iCub Facility, Istituto Italiano di Tecnologia
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

#ifndef __imuST_M1__
#define __imuST_M1__

#include <string.h>
#include <termios.h> // terminal io (serial port) interface

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/RateThread.h>

#include <ST_M1_dataType.h>

#include <arpa/inet.h>

namespace yarp{
    namespace dev{
        class imuST_M1;
    }
}


// union IMUData {
//     char packet[53];
//     Pippo pippo;
// };

/**
 * @ingroup icub_hardware_modules
 * @brief `imuST_M1` : driver for 3DM_GX3 IMU unit from MicroStrain
 * @author Alberto Cardellino
 *
 * | YARP device name |
 * |:-----------------:|
 * | `imuST_M1` |
 */
class yarp::dev::imuST_M1 :     public yarp::dev::IGenericSensor,
                                public yarp::dev::IPreciselyTimed,
                                public yarp::dev::DeviceDriver,
                                public yarp::os::RateThread
{
private:

    bool                opened;
    int                 fd_ser;
    int                 baudrate;
    int                 nchannels;
    short               comPort;
    std::string         comPortString;

    // data specific for this imu
//     typedef std::map<int, imu_cmd_t*> cmd_map_t;
//     cmd_map_t cmd_ptr_map;
    int expected_packet_size;
    int expected_payload_size;
    char *buffer;
//     IMUData             data;

    int progressiv_num;
    double *temp_euler;
    double *temp_acc;
    double *temp_gyro;
    double *temp_mag;
    double  temp_data[12];

    yarp::os::Semaphore data_mutex;
    yarp::os::Semaphore sync_mutex;

    std::string         comPortName;
    ST_IMU_Frame        rawData;
    yarp::os::Stamp     lastStamp;

public:
    imuST_M1();
    ~imuST_M1();

    // IGenericSensor interface.
    virtual bool read(yarp::sig::Vector &out);
    virtual bool getChannels(int *nc);
    virtual bool open(yarp::os::Searchable &config);
    virtual bool calibrate(int ch, double v);
    virtual bool close();
    void sample_setting(void);

    virtual yarp::os::Stamp getLastInputStamp();

private:
    bool threadInit();
    void run();
    void threadRelease();
    Pippo *pippo;
    Pippo  outVals;
    bool verbose;
    
private:      // Device specific
    float       *euler_float;

//     void stop_continuous(void);
//     // command 0xC2
//     void get_Acc_Ang(float acc[3], float angRate[3], uint64_t *time);
//     // command 0xC8
//     void get_Acc_Ang_Orient(float acc[3], float angRate[3], float orientMat[9], uint64_t *time);
//     // command 0xCB
//     void get_Acc_Ang_Mag(float acc[3], float angRate[3], float mag[3], uint64_t *time);
//     // command 0xCC
//     void get_Acc_Ang_Mag_Orient(float acc[3], float angRate[3], float mag[3], float orientMat[9], uint64_t *time);
//     // command 0xCE
//     void get_Euler(float euler[3], uint64_t *time);
//     // command 0xCF
//     void get_Euler_AngularRate(float euler[3], float angRate[3], uint64_t *time);
//     // command 0xDF
//     void get_Quaternion(float quat[4], uint64_t *time);
//
//     // serial handling
//     inline int flush() {return tcflush(fd_ser, TCIOFLUSH); }
};


#endif  // __imuST_M1__
