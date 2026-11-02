#include "PassThroughInertial.h"
#include <yarp/os/Property.h>
#include <yarp/os/LogStream.h>

namespace yarp {
namespace dev {

PassThroughInertial::PassThroughInertial() : proxyIGyro{nullptr},
                                             proxyIAccel{nullptr},
                                             proxyIMagn{nullptr},
                                             proxyIOrient{nullptr}
{
}

PassThroughInertial::~PassThroughInertial()
{
    this->close();
}

bool PassThroughInertial::open(yarp::os::Searchable& config) //TODO: still to be discussed
{
    bool ok = true;

    if (config.check("subdevice") && config.find("subdevice").isString())
    {
        ownSubdevice = true;
        ok &= config.check("proxy_remote");
        ok &= config.check("proxy_local");
        ok &= config.find("proxy_remote").isString();
        ok &= config.find("proxy_local").isString();
        if(!ok)
        {
            yError()<<"PassThroughInertial: proxy conf malformed";
            return false;
        }
        auto subdeviceName = config.find("subdevice").asString();
        yarp::os::Property masClientProp;
        masClientProp.fromString(config.toString());
        masClientProp.put("device", subdeviceName); // "multipleanalogsensorsclient"
        masClientProp.put("local", config.find("proxy_local").asString());
        masClientProp.put("remote", config.find("proxy_remote").asString());

        ok &= proxyDevice.open(masClientProp);
        if (!ok) {
            yError()<<"PassThroughInertial: failed to open the proxy device";
            return false;
        }

        ok &= proxyDevice.view(proxyIGyro);
        ok &= proxyDevice.view(proxyIAccel);
        ok &= proxyDevice.view(proxyIMagn);
        ok &= proxyDevice.view(proxyIOrient);

        if (!ok) {
            yError()<<"PassThroughInertial: the view of one interface failed";
            return false;
        }

    }

    yInfo("Finish PassThroughInertial::open");
    return true;
}

bool PassThroughInertial::close()
{
    if (ownSubdevice)
        return proxyDevice.close();
    else
        return detachAll();
}


yarp::dev::ReturnValue PassThroughInertial::getNrOfThreeAxisGyroscopes(size_t& num) const
{
    if (!proxyIGyro)
        return ReturnValue_error_not_ready;
    return proxyIGyro->getNrOfThreeAxisGyroscopes(num);
}


yarp::dev::MAS_status PassThroughInertial::getThreeAxisGyroscopeStatus(size_t sens_index) const
{
    if (!proxyIGyro)
        return yarp::dev::MAS_ERROR;
    return proxyIGyro->getThreeAxisGyroscopeStatus(sens_index);
}

yarp::dev::ReturnValue PassThroughInertial::getThreeAxisGyroscopeName(size_t sens_index, std::string& name) const
{
    if (!proxyIGyro)
        return ReturnValue_error_not_ready;
    return proxyIGyro->getThreeAxisGyroscopeName(sens_index, name);
}

yarp::dev::ReturnValue PassThroughInertial::getThreeAxisGyroscopeFrameName(size_t sens_index, std::string& frameName) const
{
    if (!proxyIGyro)
        return ReturnValue_error_not_ready;
    return proxyIGyro->getThreeAxisGyroscopeFrameName(sens_index, frameName);
}

yarp::dev::ReturnValue PassThroughInertial::getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    if (!proxyIGyro)
        return ReturnValue_error_not_ready;
    return proxyIGyro->getThreeAxisGyroscopeMeasure(sens_index, out, timestamp);
}


yarp::dev::ReturnValue PassThroughInertial::getNrOfThreeAxisLinearAccelerometers(size_t& num) const
{
    if (!proxyIAccel)
        return ReturnValue_error_not_ready;
    return proxyIAccel->getNrOfThreeAxisLinearAccelerometers(num);
}


yarp::dev::MAS_status PassThroughInertial::getThreeAxisLinearAccelerometerStatus(size_t sens_index) const
{
    if (!proxyIAccel)
        return yarp::dev::MAS_ERROR;
    return proxyIAccel->getThreeAxisLinearAccelerometerStatus(sens_index);
}

yarp::dev::ReturnValue PassThroughInertial::getThreeAxisLinearAccelerometerName(size_t sens_index, std::string& name) const
{
    if (!proxyIAccel)
        return ReturnValue_error_not_ready;
    return proxyIAccel->getThreeAxisLinearAccelerometerName(sens_index, name);
}

yarp::dev::ReturnValue PassThroughInertial::getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string& frameName) const
{
    if (!proxyIAccel)
        return ReturnValue_error_not_ready;
    return proxyIAccel->getThreeAxisLinearAccelerometerFrameName(sens_index, frameName);
}

yarp::dev::ReturnValue PassThroughInertial::getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    if (!proxyIAccel)
        return ReturnValue_error_not_ready;
    return proxyIAccel->getThreeAxisLinearAccelerometerMeasure(sens_index, out, timestamp);
}


yarp::dev::ReturnValue PassThroughInertial::getNrOfOrientationSensors(size_t& num) const
{
    if (!proxyIOrient)
        return ReturnValue_error_not_ready;
    return proxyIOrient->getNrOfOrientationSensors(num);
}

yarp::dev::MAS_status PassThroughInertial::getOrientationSensorStatus(size_t sens_index) const
{
    if (!proxyIOrient)
        return yarp::dev::MAS_ERROR;
    return proxyIOrient->getOrientationSensorStatus(sens_index);
}

yarp::dev::ReturnValue PassThroughInertial::getOrientationSensorName(size_t sens_index, std::string& name) const
{
    if (!proxyIOrient)
        return ReturnValue_error_not_ready;
    return proxyIOrient->getOrientationSensorName(sens_index, name);
}

yarp::dev::ReturnValue PassThroughInertial::getOrientationSensorFrameName(size_t sens_index, std::string& frameName) const
{
    if (!proxyIOrient)
        return ReturnValue_error_not_ready;
    return proxyIOrient->getOrientationSensorFrameName(sens_index, frameName);
}

yarp::dev::ReturnValue PassThroughInertial::getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy, double& timestamp) const
{
    if (!proxyIOrient)
        return ReturnValue_error_not_ready;
    return proxyIOrient->getOrientationSensorMeasureAsRollPitchYaw(sens_index, rpy, timestamp);
}

yarp::dev::ReturnValue PassThroughInertial::getNrOfThreeAxisMagnetometers(size_t& num) const
{
    if (!proxyIMagn)
        return ReturnValue_error_not_ready;
    return proxyIMagn->getNrOfThreeAxisMagnetometers(num);
}

yarp::dev::MAS_status PassThroughInertial::getThreeAxisMagnetometerStatus(size_t sens_index) const
{
    if (!proxyIMagn)
        return yarp::dev::MAS_ERROR;
    return proxyIMagn->getThreeAxisMagnetometerStatus(sens_index);
}

yarp::dev::ReturnValue PassThroughInertial::getThreeAxisMagnetometerName(size_t sens_index, std::string& name) const
{
    if (!proxyIMagn)
        return ReturnValue_error_not_ready;
    return proxyIMagn->getThreeAxisMagnetometerName(sens_index, name);
}

yarp::dev::ReturnValue PassThroughInertial::getThreeAxisMagnetometerFrameName(size_t sens_index, std::string& frameName) const
{
    if (!proxyIMagn)
        return ReturnValue_error_not_ready;
    return proxyIMagn->getThreeAxisMagnetometerFrameName(sens_index, frameName);
}

yarp::dev::ReturnValue PassThroughInertial::getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    if (!proxyIMagn)
        return ReturnValue_error_not_ready;
    return proxyIMagn->getThreeAxisMagnetometerMeasure(sens_index, out, timestamp);
}

bool PassThroughInertial::attachAll(const yarp::dev::PolyDriverList& p)
{
    if (ownSubdevice)
    {
        yError()<<"PassThroughInertial: holding a subdevice, attach not allowed";
        return false;
    }
    // Attach the device
    if (p.size() > 1)
    {
        yError("PassThroughInertial: this device only supports exposing a "
               "single PassThroughInertial device on YARP ports, but %d devices have been passed in attachAll.", p.size());
        yError("PassThroughInertial: please use the multipleanalogsensorsremapper device to combine several device in a new device.");
        detachAll();
        return false;
    }

    if (p.size() == 0)
    {
        yError("PassThroughInertial: no device passed to attachAll, please pass a device to expose on YARP ports.");
        return false;
    }

    yarp::dev::PolyDriver* poly = p[0]->poly;

    if (!poly)
    {
        yError("PassThroughInertial: null pointer passed to attachAll.");
        return false;
    }

    bool ok  = true;
    // View all the interfaces
    ok &= poly->view(proxyIGyro);
    ok &= poly->view(proxyIAccel);
    ok &= poly->view(proxyIMagn);
    ok &= poly->view(proxyIOrient);

    if(!ok)
    {
        detachAll();
        return false;
    }

    yInfo()<<"PassThroughInertial: attach finished with success";


    return ok;
}

bool PassThroughInertial::detachAll()
{
    proxyIGyro = nullptr;
    proxyIMagn = nullptr;
    proxyIOrient = nullptr;
    proxyIAccel = nullptr;
    return true;
}


} // namespace dev
} // namespace yarp
