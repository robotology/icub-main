// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006 Radu Bogdan Rusu, Alexis Maldonado
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <string>

#include "MTComm.h"
#include "XSensMTx.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
//using ACE_OS::printf;

#define M_PI             3.14159265358979323846264338328
#define CTRL_RAD2DEG    (180.0/M_PI)
#define CTRL_DEG2RAD    (M_PI/180.0)

constexpr size_t rpyStartIdx   = 0;
constexpr size_t accelStartIdx = 3;
constexpr size_t gyroStartIdx  = 6;
constexpr size_t magnStartIdx  = 9;


class XSensMTxResources: public Thread
{
public:
    explicit XSensMTxResources(): _semaphore(1)
    {
        _bStreamStarted=false;
        _bError=false;
            
        _last=0;

        _last=new double [12];
        for(int k=0;k<12;k++)
            _last[k]=0.0;
    }

    virtual ~XSensMTxResources()
    {
        if (isRunning())
            stop();

        _bStreamStarted=false;

        if (_last!=0)
            {
                delete [] _last;
                _last=0;
            }
    }

    bool _bStreamStarted;
    bool _bError;

    CMTComm mtcomm;
    
    double *_last;
    yarp::os::Stamp _lastStamp;

    Semaphore _semaphore;

    virtual void run ();
};

void XSensMTxResources::run ()
{   
    unsigned char data[MAXMSGLEN];
    float euler_data[3] = {0};
    float accel_data[3] = {0};
    float gyro_data [3] = {0};
    float magn_data [3] = {0};
    short datalen;
    int received;

    while (!Thread::isStopping ())
        {
            // Get data from the MTx device
            received = mtcomm.readDataMessage (data, datalen);
            // Parse and get value (EULER ORIENTATION)
            mtcomm.getValue (VALUE_ORIENT_EULER, euler_data, data, BID_MASTER);
            // Parse and get calibrated acceleration values
            mtcomm.getValue (VALUE_CALIB_ACC, accel_data, data, BID_MASTER);
            // Parse and get calibrated gyro values
            mtcomm.getValue (VALUE_CALIB_GYR, gyro_data, data, BID_MASTER);
            // Parse and get calibrated magnetometer values
            mtcomm.getValue (VALUE_CALIB_MAG, magn_data, data, BID_MASTER);
	    
            _semaphore.wait ();
            
			//euler_data are expressed in deg
            _last[0]  = euler_data[0]; //roll
            _last[1]  = euler_data[1]; //pitch
            _last[2]  = euler_data[2]; //yaw

            _last[3]  = accel_data[0]; //accel-X
            _last[4]  = accel_data[1]; //accel-Y
            _last[5]  = accel_data[2]; //accel-Z
	    
			//gyro_data are expressed in rad/s, so they have to be converted in deg/s
            _last[6]  = gyro_data[0]*CTRL_RAD2DEG;  //gyro-X
            _last[7]  = gyro_data[1]*CTRL_RAD2DEG;  //gyro-Y
            _last[8]  = gyro_data[2]*CTRL_RAD2DEG;  //gyro-Z
	    
            _last[9]  = magn_data[0];  //magn-X
            _last[10] = magn_data[1];  //magn-Y
            _last[11] = magn_data[2];  //magn-Z

            if (received == MTRV_OK)
                _bError=false;
            else
                _bError=true;
		
            _lastStamp.update();
            _semaphore.post ();
        }
}

inline XSensMTxResources& RES(void *res) { return *(XSensMTxResources *)res; }

/**
 * Driver for XSens's MTx IMU unit.
 * @author Radu Bogdan Rusu, Alexis Maldonado
 */ 
XSensMTx::XSensMTx() : system_resources{nullptr},
                       nchannels{12},
                       m_sensorName{"sensor_imu_xsens"},
                       m_frameName{"sensor_imu_xsens"}
{
}

XSensMTx::~XSensMTx()
{
    // stop thread first
    if (system_resources!=0)
        {
            delete ((XSensMTxResources *)(system_resources));
            system_resources=0;
        }
}

bool XSensMTx::read(Vector &out)
{
    XSensMTxResources &d= RES(system_resources);
    bool ret;
    
    if (d._bStreamStarted)
        {
            d._semaphore.wait();
            
            // Euler+accel+gyro+magn orientation values
            for (int i = 0; i < nchannels; i++)
                out[i]=d._last[i];

            lastStamp=d._lastStamp;
            d._semaphore.post();

            ret=!d._bError;
        }
    else
        ret=false;

    return ret;
}

bool XSensMTx::getChannels(int *nc)
{
    *nc=nchannels;
    return true;
}

bool XSensMTx::calibrate(int ch, double v)
{
    printf("Not implemented yet\n");
    return false;
}

bool XSensMTx::start()
{
    XSensMTxResources &d=RES(system_resources);

    d.start();
    d._bStreamStarted=true;

    return true;
}

bool XSensMTx::stop()
{
    XSensMTxResources &d=RES(system_resources);

    if (d.isRunning())
        d.stop();

    d._bStreamStarted=false;

    return true;
}

bool XSensMTx::open(yarp::os::Searchable &config)
{
    XSensMTxParameters par;
     
#ifdef WIN32
    par.comPort = config.check ("serial", Value(11),
        "numeric identifier of comport").asInt();
#else
    par.comPortString = config.check("serial",Value("/dev/ttyUSB0"),
                                     "device name of comport").asString().c_str();
#endif
    if (config.check("sensor_name") && config.find("sensor_name").isString())
    {
        m_sensorName = config.find("sensor_name").asString();
    }
    if (config.check("frame_name") && config.find("frame_name").isString())
    {
        m_frameName = config.find("frame_name").asString();
    }

    return open(par);
}

bool XSensMTx::open(const XSensMTxParameters &par)
{
    if (system_resources!=0)
        return false;

    system_resources=(void *) (new XSensMTxResources);

    XSensMTxResources &d=RES(system_resources);

    // Open the MTx device
#ifdef WIN32
    if (d.mtcomm.openPort (par.comPort) != MTRV_OK)
        {
            fprintf(stderr, "Failed to open com port %d\n", 
                    par.comPort);

            return false;
        }
#else
    if (d.mtcomm.openPort (par.comPortString.c_str ()) != MTRV_OK)
        {
            fprintf(stderr, "Failed to open com port %s\n", 
                    par.comPortString.c_str());
            return false;
        }
#endif

    int outputSettings = OUTPUTSETTINGS_ORIENTMODE_EULER;
    
    unsigned long tmpOutputMode, tmpOutputSettings;
    unsigned short tmpDataLength;
		
    // Put MTi/MTx in Config State. Here sometimes there are problems if the device was not properly closed.
    int count = 0;
    for (count = 0; count <10; count++ )
    {
        if(d.mtcomm.writeMessage (MID_GOTOCONFIG) != MTRV_OK)
        {
            printf ("MRCHECK Unable to connect to XSensMtX device, attempt %d.\n", count);
            yarp::os::Time::delay(0.010);
        }
        else 
            break;
    }
    if (count >= 10)
    {
        printf ("XSensMtX init check: no device connected.\n");
        return false;
    }
    else
        printf ("XSensMtX init check: device ok.\n");

    unsigned short numDevices;
    // Get current settings and check if Xbus Master is connected
    if (d.mtcomm.getDeviceMode(&numDevices) != MTRV_OK) {
        if (numDevices == 1)
            printf ("MTi / MTx has not been detected\nCould not get device mode.\n");
        else
            printf ("Not just MTi / MTx connected to Xbus, %d devices found.\nCould not get all device modes.\n", numDevices);
        return false;
    }

    // Check if Xbus Master is connected
    d.mtcomm.getMode (tmpOutputMode, tmpOutputSettings, tmpDataLength, BID_MASTER);
    if (tmpOutputMode == OUTPUTMODE_XM)
        {
            // If Xbus Master is connected, attached Motion Trackers should not send sample counter
            printf ("Sorry, this driver only talks to one MTx device.\n");
            return false;
        }

    int outputMode = OUTPUTMODE_CALIB + OUTPUTMODE_ORIENT;
    // Set output mode and output settings for the MTi/MTx
    if (d.mtcomm.setDeviceMode(outputMode, outputSettings, BID_MASTER) != MTRV_OK) {
        printf ("Could not set device mode(s)\n");
        return false;
    }

    // Put MTi/MTx in Measurement State
    d.mtcomm.writeMessage (MID_GOTOMEASUREMENT);

    // start thread
    return XSensMTx::start();
}

bool XSensMTx::close()
{
    // stop thread
    if (system_resources==0)
        return false; //the device was never opened, or there was an error 

    XSensMTx::stop();

    XSensMTxResources &d=RES(system_resources);

    // Close the MTx device
    if (d.mtcomm.close () != MTRV_OK)
        {
            delete ((XSensMTxResources *)(system_resources));
            system_resources=0;
            return false;
        }
    else
        {
            delete ((XSensMTxResources *)(system_resources));
            system_resources=0;
            return true;
        }
}

yarp::os::Stamp XSensMTx::getLastInputStamp()
{
    return lastStamp;
}


size_t XSensMTx::getNrOfThreeAxisLinearAccelerometers() const
{
    return 1;
}


yarp::dev::MAS_status XSensMTx::getThreeAxisLinearAccelerometerStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool XSensMTx::getThreeAxisLinearAccelerometerName(size_t sens_index, std::string& name) const
{
    return genericGetSensorName(sens_index, name);
}

bool XSensMTx::getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string& frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool XSensMTx::getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return genericGetMeasure(sens_index, out, timestamp, accelStartIdx);
}


size_t XSensMTx::getNrOfThreeAxisGyroscopes() const
{
    return 1;
}


yarp::dev::MAS_status XSensMTx::getThreeAxisGyroscopeStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool XSensMTx::getThreeAxisGyroscopeName(size_t sens_index, std::string& name) const
{
    return genericGetSensorName(sens_index, name);
}

bool XSensMTx::getThreeAxisGyroscopeFrameName(size_t sens_index, std::string& frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool XSensMTx::getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return genericGetMeasure(sens_index, out, timestamp, gyroStartIdx);
}

size_t XSensMTx::getNrOfOrientationSensors() const
{
    return 1;
}

yarp::dev::MAS_status XSensMTx::getOrientationSensorStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool XSensMTx::getOrientationSensorName(size_t sens_index, std::string& name) const
{
    return genericGetSensorName(sens_index, name);
}

bool XSensMTx::getOrientationSensorFrameName(size_t sens_index, std::string& frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool XSensMTx::getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy, double& timestamp) const
{
    return genericGetMeasure(sens_index, rpy, timestamp, rpyStartIdx);
}

size_t XSensMTx::getNrOfThreeAxisMagnetometers() const
{
    return 1;
}

yarp::dev::MAS_status XSensMTx::getThreeAxisMagnetometerStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool XSensMTx::getThreeAxisMagnetometerName(size_t sens_index, std::string& name) const
{
    return genericGetSensorName(sens_index, name);
}

bool XSensMTx::getThreeAxisMagnetometerFrameName(size_t sens_index, std::string& frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool XSensMTx::getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return genericGetMeasure(sens_index, out, timestamp, magnStartIdx);
}
yarp::dev::MAS_status XSensMTx::genericGetStatus(size_t sens_index) const
{
    if (sens_index != 0)
    {
        yError() << "xsens: sens_index must be equal to 0, since there is  only one sensor in consideration";
        return yarp::dev::MAS_status::MAS_ERROR;
    }

    return yarp::dev::MAS_status::MAS_OK;
}

bool XSensMTx::genericGetSensorName(size_t sens_index, std::string& name) const
{
    if (sens_index != 0)
    {
        yError() << "xsens: sens_index must be equal to 0, since there is  only one sensor in consideration";
        return false;
    }

    name = m_sensorName;
    return true;
}

bool XSensMTx::genericGetFrameName(size_t sens_index, std::string& frameName) const
{
    if (sens_index != 0)
    {
        yError() << "xsens: sens_index must be equal to 0, since there is  only one sensor in consideration";
        return false;
    }

    frameName = m_frameName;
    return true;

}

bool XSensMTx::genericGetMeasure(size_t sens_index, yarp::sig::Vector &out, double &timestamp, size_t startIdx) const {

    if (sens_index != 0)
    {
        yError() << "xsens: sens_index must be equal to 0, since there is  only one sensor in consideration";
        return false;
    }

    auto &d= RES(system_resources);
    out.resize(3);
    d._semaphore.wait();
    out[0] = d._last[startIdx];
    out[1] = d._last[startIdx + 1];
    out[2] = d._last[startIdx + 2];

    timestamp = d._lastStamp.getTime();
    d._semaphore.post();
    return true;
}
