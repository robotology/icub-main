/*
 * Copyright (C) 2015 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iostream>
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions

#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>
#include <yarp/os/LogStream.h>
#include <ST_M1.h>

using namespace yarp::dev;
using namespace yarp::os;


imuST_M1::imuST_M1() : RateThread(6)
{
    /* data to be transfered are:
            1  (request Euler) +
            19 (answer Euler) +
            1  (acc, request gyro, magn) +
            43 (answer) = 64 bytes
       serial rate is 115.200 Kbits/s -> max rate is 181Hz - 5.52ms (if device support it... check)
       choose 6ms as period to let device have a breath.
   */

    // does nchannels has to be a parameter? I don't think so since ServerInertial has this fixed interface
    nchannels = 12;
    opened = false;

};

imuST_M1::~imuST_M1()
{
    close();
};


// IGenericSensor interface.
bool imuST_M1::open(yarp::os::Searchable &config)
{
    yTrace();
    bool ret = true;

    Value *baudrate, *serial;

    if(!config.check("serial", serial))
    {
        std::cout << "Can't find 'serial' name in config file";
        return false;
    }

    comPortName = serial->toString();

    int errNum = 0;
    /* open serial */
    printf("\n\nSerial opening %s\n\n\n", comPortName.c_str());
    fd_ser = ::open(comPortName.c_str(), O_RDWR | O_NOCTTY );
    if (fd_ser < 0) {
        printf("can't open %s, %s\n", comPortName.c_str(), strerror(errno));
        return false;
    }

    //Get the current options for the port...
    struct termios options;
    tcgetattr(fd_ser, &options);

    //set the baud rate to 115200
    int baudRate = B115200;
    cfsetospeed(&options, baudRate);
    cfsetispeed(&options, baudRate);

    //set the number of data bits.
    options.c_cflag &= ~CSIZE;  // Mask the character size bits
    options.c_cflag |= CS8;

    //set the number of stop bits to 1
    options.c_cflag &= ~CSTOPB;

    //Set parity to None
    options.c_cflag &=~PARENB;

    //set for non-canonical (raw processing, no echo, etc.)
    options.c_iflag = IGNPAR; // ignore parity check
    options.c_oflag = 0; // raw output
    options.c_lflag = 0; // raw input

    //Time-Outs -- won't work with NDELAY option in the call to open
    options.c_cc[VMIN]  = 0;   // block reading until RX x characers. If x = 0, it is non-blocking.
    options.c_cc[VTIME] = 100;   // Inter-Character Timer -- i.e. timeout= x*.1 s

    //Set local mode and enable the receiver
    options.c_cflag |= (CLOCAL | CREAD);

    tcflush(fd_ser,TCIOFLUSH);

    //Set the new options for the port...
    if ( tcsetattr(fd_ser, TCSANOW, &options) != 0)
    { //For error message
        printf("Configuring comport failed\n");
        return false;
    }
//     this->start();

    opened = true;
    char buff[20];
    buff[0] = 0x20;
    buff[1] = 0x01;
    buff[2] = 0x00;
    int nbytes_w = ::write(fd_ser, (void*)buff, 3);

    memset(buff, 0x00, 20);
    int nbytes_r = ::read(fd_ser,  (void*)buff, 3);

    printf("Received response 0x%0X, 0x%0X, 0x%0X\n", buff[0], buff[1], buff[2]);


    buff[0] = 0x20;
    buff[1] = 0x02;
    buff[2] = 0x08;
    buff[3] = 0x01;
    nbytes_w = ::write(fd_ser, (void*)buff, 4);

    memset(buff, 0x00, 20);
    nbytes_r = ::read(fd_ser,  (void*)buff, 3);

    printf("Received response 0x%0X, 0x%0X, 0x%0X\n", buff[0], buff[1], buff[2]);

    sleep(1);
    buff[0] = 0x20;
    buff[1] = 0x02;
    buff[2] = 0x08;
    buff[3] = 0x00;
    nbytes_w = ::write(fd_ser, (void*)buff, 4);

    memset(buff, 0x00, 20);
    nbytes_r = ::read(fd_ser,  (void*)buff, 3);

    printf("Received response 0x%0X, 0x%0X, 0x%0X\n", buff[0], buff[1], buff[2]);

    sleep(1);
    buff[0] = 0x20;
    buff[1] = 0x01;
    buff[2] = 0x19;
    nbytes_w = ::write(fd_ser, (void*)buff, 3);

    memset(buff, 0x00, 20);
    nbytes_r = ::read(fd_ser,  (void*)buff, 4);

    printf("Received response 0x%0X, 0x%0X, 0x%0X, 0x%0X\n", buff[0], buff[1], buff[2], buff[3]);

    return true;
}


void imuST_M1::sample_setting(void)
{
//    /* int nbytes;
//     uint8_t reply[20];
//     uint8_t buff[20] =
//     {
//         CMD_SAMPLING_SETTING,
//         0xA8, 0xB9, // confirm user intent
//         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
//     };
//
//     nbytes = ::write(fd_ser, (void*)buff, 20);
//     nbytes = ::read(fd_ser, (void*)reply, 19);
//
//     memcpy(buff+4, reply+1, 10);
//
//     // Function selector : change parameter but do not send a treply
//     buff[3]  = 3;
//     // Data conditioning function selector byte 7-8 : bit 12 enable quaternion ... remember big endian
//     buff[6] |= 16;
//
//     //printHex(buff, 20);
//
//     nbytes = write(fd_ser, (void*)buff, 20);
//
//     usleep(100000);*/
}


bool imuST_M1::close()
{
    yTrace();
    if(opened)
    {
    // 	this->stop();

        char buff[20];
        buff[0] = 0x20;
        buff[1] = 0x01;
        buff[2] = 0x01;
        int nbytes_w = ::write(fd_ser, (void*)buff, 3);

        memset(buff, 0x00, 20);
        int nbytes_r = ::read(fd_ser,  (void*)buff, 3);

        printf("Received response 0x%0X, 0x%0X, 0x%0X\n", buff[0], buff[1], buff[2]);

        if(fd_ser != 0)
            ::close(fd_ser);
        fd_ser = 0;
        printf("Closed %s\n", comPortName.c_str());
    }
    opened = false;
    return true;
}


bool imuST_M1::getChannels(int *nc)
{
    *nc = nchannels;
    return true;
}


bool imuST_M1::calibrate(int ch, double v)
{
    // Send command to zero gyro bias?? Be careful it must be used only if the device is absolutely stationary!!
    printf("Not implemented yet\n");
    return false;
}


bool imuST_M1::read(yarp::sig::Vector &out)
{
    float tmp_eul[3];
    float tmp_acc[3];
    float tmp_gyro[3];
    float tmp_mag[3];

    uint64_t imu_timeStamp;

//     get_Euler(tmp_eul, &imu_timeStamp);
//     get_Acc_Ang_Mag(tmp_acc, tmp_gyro, tmp_mag, &imu_timeStamp);

    int out_idx = 0;
    for(int i=0; i<3; i++, out_idx++)
        out[out_idx] = (double) tmp_eul[i];

    for(int i=0; i<3; i++, out_idx++)
        out[out_idx] = (double) tmp_acc[i];

    for(int i=0; i<3; i++, out_idx++)
        out[out_idx] = (double) tmp_gyro[i];

    for(int i=0; i<3; i++, out_idx++)
        out[out_idx] = (double) tmp_mag[i];

    lastStamp.update();
    return true;
}

yarp::os::Stamp imuST_M1::getLastInputStamp()
{
    return lastStamp;
}

///////////////////////////////////////////////////////////////////////////////
//                                  Thread                                   //
///////////////////////////////////////////////////////////////////////////////

bool imuST_M1::threadInit()
{
    // if board was configured to automatically send data on wakeup, this will make it silent
//     stop_continuous();
    sample_setting();
    printf("Started imu-3DM-GX3-25 thread\n");
    return true;
}

void imuST_M1::run()
{
//     int nbytes;
//     static data_3DM_GX3_t   th_data;
//     imu_cmd_t * tmp_cmd = NULL;
//
//     // ask data to device in polling, cycling within the map
//     cmd_map_t::iterator it = cmd_ptr_map.begin();
//     while(it != cmd_ptr_map.end())
//     {
//         tmp_cmd = it->second;
//         // transmit command
//         nbytes = ::write(fd_ser, (void*)&tmp_cmd->cmd, sizeof(uint8_t));
//
//         // receive response
//         nbytes = ::read(fd_ser, (void*)&th_data.buffer, (size_t)tmp_cmd->expSize);
//
//         if ( nbytes < 0 )
//         {
//             printf("Error while reading imu response to command XXX\n");
//             // skip parsing and try with the next command
//         }
//         else if ( nbytes != tmp_cmd->expSize )
//         {
//             printf("read %d instead of %d\n", nbytes, tmp_cmd->expSize);
//             // skip parsing and try with the next command
//         }
//         else
//         {
//             // process data ... at least swap bytes !!!
//             if ( tmp_cmd->process ) {
//                 // compute checksum and swap bytes
//                 if ( ! (bool)tmp_cmd->process(th_data) )
//                 {
//                     printf("Checksum FAILURE\n");
//                     continue;
//                 }
//             }
//
//             // copy for consumer ....
//             data_mutex.wait();
//             memcpy((void*)&tmp_cmd->data.buffer, &th_data.buffer, tmp_cmd->expSize);
//             data_mutex.post();
//         }
//         it++;
//     }
}

void imuST_M1::threadRelease()
{
//     stop_continuous();
}

///////////////////////////////////////////////////////////////////////////////
//                             Device specific                               //
///////////////////////////////////////////////////////////////////////////////
#if 0

void imuST_M1::stop_continuous(void)
{

    int nbytes;
    const uint8_t buff[] = {
        CMD_STOP_CONTINUOUS,
        0x75, 0xB4 // confirms user intent
    };

    nbytes = write(fd_ser, (void*)buff, sizeof(buff));
    // !!! need to clean serial buffer ...
    sleep(1);
//     flush();
}

void imuST_M1::get_Acc_Ang(float acc[3], float angRate[3], uint64_t *time) {

    acc_angRate_t * aa;

    data_mutex.wait();
    aa = &C2_cmd.data.aa;
    if (acc) {
        memcpy((void*)acc, aa->acc._bytes, sizeof(float)*3);
    }
    if (angRate) {
        memcpy((void*)angRate, aa->angRate._bytes, sizeof(float)*3);
    }
    data_mutex.post();
}

void imuST_M1::get_Acc_Ang_Orient(float acc[3], float angRate[3], float orientMat[9], uint64_t *time) {

    acc_ang_orient_t * aaom;

    data_mutex.wait();
    aaom = &C8_cmd.data.aaom;
    if (acc) {
        memcpy((void*)acc, aaom->acc._bytes, sizeof(float)*3);
    }
    if (angRate) {
        memcpy((void*)angRate, aaom->angRate._bytes, sizeof(float)*3);
    }
    if (orientMat) {
        memcpy((void*)orientMat, aaom->orientMat._bytes, sizeof(float)*9);
    }
    data_mutex.post();
}

void imuST_M1::get_Acc_Ang_Mag(float acc[3], float angRate[3], float mag[3], uint64_t *time) {
    acc_ang_mag_t * aam;

    data_mutex.wait();
    aam = &CB_cmd.data.aam;
    if (acc) {
        memcpy((void*)acc, aam->acc._bytes, sizeof(float)*3);
    }
    if (angRate) {
        memcpy((void*)angRate, aam->angRate._bytes, sizeof(float)*3);
    }
    if (mag) {
        memcpy((void*)mag, aam->mag._bytes, sizeof(float)*3);
    }
    data_mutex.post();
}

void imuST_M1::get_Acc_Ang_Mag_Orient(float acc[3], float angRate[3], float mag[3], float orientMat[9], uint64_t *time) {

    acc_ang_mag_orient_t * aamom;

    data_mutex.wait();
    aamom = &CC_cmd.data.aamom;
    if (acc) {
        memcpy((void*)acc, aamom->acc._bytes, sizeof(float)*3);
    }
    if (angRate) {
        memcpy((void*)angRate, aamom->angRate._bytes, sizeof(float)*3);
    }
    if (mag) {
        memcpy((void*)mag, aamom->mag._bytes, sizeof(float)*3);
    }
    if (orientMat) {
        memcpy((void*)orientMat, aamom->orientMat._bytes, sizeof(float)*9);
    }
    data_mutex.post();
}

void imuST_M1::get_Euler(float euler[3], uint64_t *time) {

    eul_t * eu;

    data_mutex.wait();
    eu = &CE_cmd.data.eu;
    if (euler) {
        memcpy((void*)euler, eu->eul._bytes, sizeof(float)*3);
    }
    data_mutex.post();
}


void imuST_M1::get_Euler_AngularRate(float euler[3], float angRate[3], uint64_t *time) {

    eul_angRate_t * ea;

    data_mutex.wait();
    ea = &CE_cmd.data.ea;
    if (euler) {
        memcpy((void*)euler, ea->eul._bytes, sizeof(float)*3);
    }
    if (angRate) {
        memcpy((void*)angRate, ea->angRate._bytes, sizeof(float)*3);
    }
    data_mutex.post();
}

void imuST_M1::get_Quaternion(float quat[4], uint64_t *time) {

    quat_t * q;

    data_mutex.wait();
    q = &DF_cmd.data.quat;
    if (quat) {
        memcpy((void*)quat, q->quat._bytes, sizeof(float)*4);
    }
    data_mutex.post();
}
#endif
