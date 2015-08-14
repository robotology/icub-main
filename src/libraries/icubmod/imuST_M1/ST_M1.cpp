/*
 * Copyright (C) 2015 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iostream>
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions

#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>
#include <yarp/os/LogStream.h>
#include <ST_M1.h>

using namespace yarp::dev;
using namespace yarp::os;


imuST_M1::imuST_M1() : RateThread(6)
{
    nchannels = 12;
    opened = false;
    progressiv_num = -1;
    temp_euler  = &temp_data[0];
    temp_acc    = &temp_data[3];
    temp_gyro   = &temp_data[6];
    temp_mag    = &temp_data[9];
}

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
//
//     //set the baud rate to 115200
    int baudRate = B115200;
    cfsetospeed(&options, baudRate);
    cfsetispeed(&options, baudRate);

    //set the number of data bits.
//     options.c_cflag &= ~CSIZE;  // Mask the character size bits
//     options.c_cflag |= CS8;
//
//     //set the number of stop bits to 1
//     options.c_cflag &= ~CSTOPB;
//
//     //Set parity to None
//     options.c_cflag &=~PARENB;

    //set for non-canonical (raw processing, no echo, etc.)
    options.c_iflag = IGNPAR; // ignore parity check
    options.c_oflag = 0; // raw output
    options.c_lflag = 0; // raw input

    //Time-Outs -- won't work with NDELAY option in the call to open
    options.c_cc[VMIN]  = 53;   // block reading until RX x characers. If x = 0, it is non-blocking.
    options.c_cc[VTIME] = 10;   // Inter-Character Timer -- i.e. timeout= x*.1 s

    //Set local mode and enable the receiver
    options.c_cflag |= (CLOCAL | CREAD);

    tcflush(fd_ser,TCIOFLUSH);

    //Set the new options for the port...
    if ( tcsetattr(fd_ser, TCSANOW, &options) != 0)
    { //For error message
        printf("Configuring comport failed\n");
        return false;
    }


    uint32_t  dummy;
    float *pf = reinterpret_cast<float*>(&dummy);

    dummy = 4225604;

    char *dummy_p = (char*)&dummy;
    printf("\ntest number %d = 0x%02X.%02X.%02X.%02X\n", dummy, dummy_p[0], dummy_p[1], dummy_p[2], dummy_p[3]);

//     uint32_t converted;
    double start = yarp::os::Time::now();

//     for (int i=0; i<1000000; i++)

        dummy = htonl(dummy);
    char *dummy_p2 = (char*)&dummy;
    printf("\ntime %f --> converted number %d = 0x%02X.%02X.%02X.%02X\n", yarp::os::Time::now() - start, dummy, dummy_p2[0], dummy_p2[1], dummy_p2[2], dummy_p2[3]);

    float val = *pf;
    char *dummy_p3 = (char*)&val;
    printf("\nfloat number %f <--> %f = 0x%02X.%02X.%02X.%02X\n", *pf, val, dummy_p3[0], dummy_p3[1], dummy_p3[2], dummy_p3[3]);
    printf("\n\n");

//     return false;




    opened = true;
    char buff[20];
    buff[0] = 0x20;
    buff[1] = 0x01;
    buff[2] = 0x00;
    int nbytes_w = ::write(fd_ser, (void*)buff, 3);

    memset(buff, 0x00, 20);
    int nbytes_r = ::read(fd_ser,  (void*)buff, 3);

    printf("Line %d: Received response 0x%0X, 0x%0X, 0x%0X\n", __LINE__, buff[0], buff[1], buff[2]);

    // turn led on
    buff[0] = 0x20;
    buff[1] = 0x02;
    buff[2] = 0x08;
    buff[3] = 0x01;
    nbytes_w = ::write(fd_ser, (void*)buff, 4);

    memset(buff, 0x00, 20);
    nbytes_r = ::read(fd_ser,  (void*)buff, 3);

    printf("Line %d: Received response 0x%0X, 0x%0X, 0x%0X\n", __LINE__, buff[0], buff[1], buff[2]);

    sleep(1);
    // turn led  off
    buff[0] = 0x20;
    buff[1] = 0x02;
    buff[2] = 0x08;
    buff[3] = 0x00;
    nbytes_w = ::write(fd_ser, (void*)buff, 4);

    memset(buff, 0x00, 20);
    nbytes_r = ::read(fd_ser,  (void*)buff, 3);

    printf("Line %d: Received response 0x%0X, 0x%0X, 0x%0X\n", __LINE__, buff[0], buff[1], buff[2]);

    buff[0] = 0x20;
    buff[1] = 0x01;
    buff[2] = 0x19;
    nbytes_w = ::write(fd_ser, (void*)buff, 3);

    memset(buff, 0x00, 20);
    nbytes_r = ::read(fd_ser,  (void*)buff, 4);

    printf("Line %d: Received response 0x%0X, 0x%0X, 0x%0X\n,  0x%0X\n", __LINE__, buff[0], buff[1], buff[2], buff[3]);

    sleep(2);

    buff[0] = 0x20;
    buff[1] = 0x03;
    buff[2] = 0x21;
    buff[3] = 0x02;
    buff[4] = 0x01;

    nbytes_w = ::write(fd_ser, (void*)buff, 5);

    memset(buff, 0x00, 20);
    nbytes_r = ::read(fd_ser,  (void*)buff, 6);

    printf("Line %d: gyro parameter fullscale 0x%0X.%0X.%0X.%0X.%0X.%0X\n", __LINE__, buff[0], buff[1], buff[2], buff[3], buff[4], buff[5]);  //  0x01 --> 500 dps


    buff[0] = 0x20;
    buff[1] = 0x03;
    buff[2] = 0x21;
    buff[3] = 0x02;
    buff[4] = 0x06;

    nbytes_w = ::write(fd_ser, (void*)buff, 5);

    memset(buff, 0x00, 20);
    nbytes_r = ::read(fd_ser,  (void*)buff, 7);

    printf("Line %d: gyro parameter scale factor 0x%0X.%0X.%0X.%0X.%0X.%0X.%0X\n", __LINE__, buff[0], buff[1], buff[2], buff[3], buff[4], buff[5], buff[6]);


    this->start();

    return true;
}

int euler_base      = 25;

void imuST_M1::sample_setting(void)
{
    printf("Setting sample config\n");
    uint8_t buff[7];
    buff[0] = 0x20;
    buff[1] = 0x05;
    buff[2] = 0x50;
    buff[3] = 0x9D;   // enable AHRS algorithm for "Euler angles", acclerometer, gyroscope, temperature data (data calibrated) (anche pressione)
    buff[4] = 0x18;   // set frequency to 50HZ (the only one usable if AHRS is set), set output port to USB
    buff[5] = 0x00;   //  bytes 5&6 to zeros means continuous mode
    buff[6] = 0x00;

    int nbytes_w = ::write(fd_ser, (void*)buff, 7);

    memset(buff, 0x00, 7);
    int nbytes_r = ::read(fd_ser,  (void*)buff, 4);
    printf("Received response 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", buff[0], buff[1], buff[2], buff[3]);

    sleep(1);
    printf("Reading back sample config\n");
    // read back the settings
    buff[0] = 0x20;
    buff[1] = 0x01;
    buff[2] = 0x51;
    nbytes_w = ::write(fd_ser, (void*)buff, 3);
    memset(buff, 0x00, 7);
    nbytes_r = ::read(fd_ser,  (void*)buff, 7);
    printf("Received response 0x%0X, 0x%0X, 0x%0X, 0x%0X, 0x%0X, 0x%0X, 0x%0X\n", buff[0], buff[1], buff[2], buff[3], buff[4], buff[5], buff[6]);

    printf("Reading libbraries config\n");
    // read back the settings
    buff[0] = 0x20;
    buff[1] = 0x01;
    buff[2] = 0x18;
    nbytes_w = ::write(fd_ser, (void*)buff, 3);
    memset(buff, 0x00, 7);
    nbytes_r = ::read(fd_ser,  (void*)buff, 4);
    printf("Line %d: Received response 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", __LINE__, buff[0], buff[1], buff[2], buff[3]);

    /* data to be transfered are:
            3  header +
            2  frame progressive number +
            6  accelerometer data +
            6  gyroscope data +
            6  magnetometer data +
            2  temperature +
            12 RPY data +
            16 quaternion =
            ------------------
            53 total

       serial rate is 115.200 Kbits/s -> max rate is 50Hz when RPY or quaternion is reequested
   */
    expected_packet_size = (1 + 1 + 1) + (2 + 6 + 6 + 6 + 2 + 12 + 16);
    expected_payload_size = expected_packet_size - 2;

    buffer = new char [100];
    pippo = (Pippo*) &buffer[accel_base];
    euler_float = reinterpret_cast<float*>(&(outVals.euler_raw[0]));
}


bool imuST_M1::close()
{
    yTrace();
    if(opened)
    {
    	this->stop();

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
    if(out.size() != 12)
        out.resize(12);

    data_mutex.wait();
    uint64_t imu_timeStamp;

    int out_idx = 0;
    for(int i=0; i<3; i++, out_idx++)
        out[out_idx] = (double) euler_float[i];

    for(int i=0; i<3; i++, out_idx++)
        out[out_idx] = (double) outVals.accel[i] * 9.81 / 1000.0f;

    for(int i=0; i<3; i++, out_idx++)
        out[out_idx] = (double) outVals.gyro[i] * 500.0f /  (2<<15);

    for(int i=0; i<3; i++, out_idx++)
        out[out_idx] = (double) outVals.magn[i];

    data_mutex.post();

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
    // configure the type of data we want from the boards
    sample_setting();
    uint8_t buff[10];

    // start acquisition
    buff[0] = 0x20;
    buff[1] = 0x01;
    buff[2] = 0x52;

    int nbytes_w = ::write(fd_ser, (void*)buff, 3);

    memset(buff, 0x00, 7);
    int nbytes_r = ::read(fd_ser,  (void*)buff, 3);

    printf("Line %d: Received response 0x%0X, 0x%0X, 0x%0X \n", __LINE__, buff[0], buff[1], buff[2]);

    printf("Started imu-3DM-GX3-25 thread\n");
    return true;
}

void imuST_M1::run()
{
//     memset(buffer, 0x00, expected_packet_size);

    int nbytes_r = ::read(fd_ser,  (void*)buffer, expected_packet_size);

//     buffer = &data.packet[0];
    // check message is correct constructed
    if(buffer[0] == 0xC0)
    {
        yError("Device return error code in the frame header!! \n");
        // cleanup(?)
        return;
    }

    if(nbytes_r != expected_packet_size)
    {
        yError("Number of bytes read is different from expected size: read %d vs expected %d\n", nbytes_r, expected_packet_size);
        // cleanup(?)
        return;
    }

    if(buffer[0] != 0x40)
    {
        yError("Wrong starting byte in the header\n");
//         cleanup(?)
        return;
    }

    if(buffer[1] != expected_payload_size)
    {
        yError("Payload size doen't match the expected one\n");
//         cleanup(?)
        return;
    }

    if(buffer[2] != 0x52)
    {
        yError("Message ID is wrong\n");
//         cleanup(?)
        return;
    }

//     uint16_t progressiv_num = ;
    int16_t tmp = *(int16_t*) &(buffer[3]); // << 8 + buffer[4];
//    int16_t tmp2 = *tmp;
   tmp = ntohs(tmp);
   printf("progressiv num = %d\n", tmp);
   if( tmp != progressiv_num+1 )
   {
       yError("Progressive number check doen't match\n");
       progressiv_num = tmp;
       // cleanup(?)
       return;
   }

    progressiv_num = tmp;

    // here the message should be fine

    int16_t temperature;

    data_mutex.wait();
    pippo->accel[0] = ntohs(pippo->accel[0]);
    pippo->accel[1] = ntohs(pippo->accel[1]);
    pippo->accel[2] = ntohs(pippo->accel[2]);

    pippo->gyro[0]  = ntohs(pippo->gyro[0]);
    pippo->gyro[1]  = ntohs(pippo->gyro[1]);
    pippo->gyro[2]  = ntohs(pippo->gyro[2]);

    pippo->magn[0]  = ntohs(pippo->magn[0]);
    pippo->magn[1]  = ntohs(pippo->magn[1]);
    pippo->magn[2]  = ntohs(pippo->magn[2]);

    pippo->temp     = ntohs(pippo->temp);

    pippo->euler_raw[0] = ntohl(pippo->euler_raw[0]);
    pippo->euler_raw[1] = ntohl(pippo->euler_raw[1]);
    pippo->euler_raw[2] = ntohl(pippo->euler_raw[2]);

    mempcpy(&outVals, pippo, sizeof(Pippo));
    data_mutex.post();

    printf("euler  %f <--> %f <--> %f\n\n", euler_float[0], euler_float[1], euler_float[2]);
    printf("accel: %4d  <--> %4d <--> %4d\n", pippo->accel[0], pippo->accel[1], pippo->accel[2]);
    printf("gyro:  %4d  <--> %4d <--> %4d\n", pippo->gyro[0], pippo->gyro[1], pippo->gyro[2]);
    printf("magn:  %4d  <--> %4d <--> %4d\n", pippo->magn[0], pippo->magn[1], pippo->magn[2]);
    printf("temp:  %4d \n", pippo->temp);

}

void imuST_M1::threadRelease()
{
    // start acquisition
    printf("Stopping acquisition imu-3DM-GX3-25 thread\n");

    uint8_t buff[7];
    buff[0] = 0x20;
    buff[1] = 0x01;
    buff[2] = 0x53;

    int nbytes_w = ::write(fd_ser, (void*)buff, 3);

    memset(buff, 0x00, 7);
    int nbytes_r = ::read(fd_ser,  (void*)buff, 3);

    printf("Line %d: Received response 0x%0X, 0x%0X, 0x%0X \n", __LINE__, buff[0], buff[1], buff[2]);
}
