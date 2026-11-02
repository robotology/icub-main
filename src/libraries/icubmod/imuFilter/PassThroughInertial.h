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
    yarp::dev::ReturnValue getNrOfThreeAxisGyroscopes(size_t& num) const override;
    yarp::dev::MAS_status getThreeAxisGyroscopeStatus(size_t sens_index) const override;
    yarp::dev::ReturnValue getThreeAxisGyroscopeName(size_t sens_index, std::string &name) const override;
    yarp::dev::ReturnValue getThreeAxisGyroscopeFrameName(size_t sens_index, std::string &frameName) const override;
    yarp::dev::ReturnValue getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IThreeAxisLinearAccelerometers methods */
    yarp::dev::ReturnValue getNrOfThreeAxisLinearAccelerometers(size_t& num) const override;
    yarp::dev::MAS_status getThreeAxisLinearAccelerometerStatus(size_t sens_index) const override;
    yarp::dev::ReturnValue getThreeAxisLinearAccelerometerName(size_t sens_index, std::string &name) const override;
    yarp::dev::ReturnValue getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string &frameName) const override;
    yarp::dev::ReturnValue getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IThreeAxisMagnetometers methods */
    yarp::dev::ReturnValue getNrOfThreeAxisMagnetometers(size_t& num) const override;
    yarp::dev::MAS_status getThreeAxisMagnetometerStatus(size_t sens_index) const override;
    yarp::dev::ReturnValue getThreeAxisMagnetometerName(size_t sens_index, std::string &name) const override;
    yarp::dev::ReturnValue getThreeAxisMagnetometerFrameName(size_t sens_index, std::string &frameName) const override;
    yarp::dev::ReturnValue getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IOrientationSensors methods */
    yarp::dev::ReturnValue getNrOfOrientationSensors(size_t& num) const override;
    yarp::dev::MAS_status getOrientationSensorStatus(size_t sens_index) const override;
    yarp::dev::ReturnValue getOrientationSensorName(size_t sens_index, std::string &name) const override;
    yarp::dev::ReturnValue getOrientationSensorFrameName(size_t sens_index, std::string &frameName) const override;
    yarp::dev::ReturnValue getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy, double& timestamp) const override;

    /* IMultipleWrapper methods */
    bool attachAll(const yarp::dev::PolyDriverList &p) override;
    bool detachAll() override;
};
#endif // PASS_THROUGH_INERTIAL_H
