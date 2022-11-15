#ifndef PASS_THROUGH_INERTIAL_H
#define PASS_THROUGH_INERTIAL_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/dev/PolyDriver.h>


namespace yarp {
    namespace dev {
        class PassThroughInertial;
    }
}

class yarp::dev::PassThroughInertial :      public DeviceDriver,
                                            public yarp::dev::IThreeAxisGyroscopes,
                                            public yarp::dev::IThreeAxisLinearAccelerometers,
                                            public yarp::dev::IThreeAxisMagnetometers,
                                            public yarp::dev::IOrientationSensors,
                                            public yarp::dev::IMultipleWrapper
{
    bool ownSubdevice {false};
protected:
    yarp::dev::PolyDriver proxyDevice;
    yarp::dev::IThreeAxisGyroscopes * proxyIGyro;
    yarp::dev::IThreeAxisLinearAccelerometers * proxyIAccel;
    yarp::dev::IThreeAxisMagnetometers * proxyIMagn;
    yarp::dev::IOrientationSensors * proxyIOrient;

public:
    //CONSTRUCTOR
    PassThroughInertial();
    virtual ~PassThroughInertial();

    //DEVICE DRIVER
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    /* IThreeAxisGyroscopes methods */
    size_t getNrOfThreeAxisGyroscopes() const override;
    yarp::dev::MAS_status getThreeAxisGyroscopeStatus(size_t sens_index) const override;
    bool getThreeAxisGyroscopeName(size_t sens_index, std::string &name) const override;
    bool getThreeAxisGyroscopeFrameName(size_t sens_index, std::string &frameName) const override;
    bool getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IThreeAxisLinearAccelerometers methods */
    size_t getNrOfThreeAxisLinearAccelerometers() const override;
    yarp::dev::MAS_status getThreeAxisLinearAccelerometerStatus(size_t sens_index) const override;
    bool getThreeAxisLinearAccelerometerName(size_t sens_index, std::string &name) const override;
    bool getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string &frameName) const override;
    bool getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IThreeAxisMagnetometers methods */
    size_t getNrOfThreeAxisMagnetometers() const override;
    yarp::dev::MAS_status getThreeAxisMagnetometerStatus(size_t sens_index) const override;
    bool getThreeAxisMagnetometerName(size_t sens_index, std::string &name) const override;
    bool getThreeAxisMagnetometerFrameName(size_t sens_index, std::string &frameName) const override;
    bool getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IOrientationSensors methods */
    size_t getNrOfOrientationSensors() const override;
    yarp::dev::MAS_status getOrientationSensorStatus(size_t sens_index) const override;
    bool getOrientationSensorName(size_t sens_index, std::string &name) const override;
    bool getOrientationSensorFrameName(size_t sens_index, std::string &frameName) const override;
    bool getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy, double& timestamp) const override;

    /* IMultipleWrapper methods */
    bool attachAll(const yarp::dev::PolyDriverList &p) override;
    bool detachAll() override;
};
#endif // PASS_THROUGH_INERTIAL_H
