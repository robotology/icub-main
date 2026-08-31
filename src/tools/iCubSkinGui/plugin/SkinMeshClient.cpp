// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Marco Randazzo, Marco Maggiali, Alessandro Scalzo, Jacopo Losi
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "include/SkinMeshClient.h"

#include <algorithm>

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#define SKIN_THRESHOLD 15.0

namespace {

YARP_LOG_COMPONENT(SKINMESHCLIENT, "yarp.tools.skinmeshclient")

constexpr double noSkinPatchWarningPeriod = 60.0;

std::string makeLocalPrefix(yarp::os::Searchable& config)
{
    std::string prefix = "/skinGui";
    if (config.check("name"))
    {
        prefix = "/";
        prefix += config.find("name").asString();
    }

    if (config.check("robotPart"))
    {
        prefix += "/";
        prefix += config.find("robotPart").asString();
    }

    return prefix;
}

}

SkinMeshClient::SkinMeshClient(Searchable& config,int period)
{
    m_period = static_cast<double>(period) / 1000.0;
    m_config.fromString(config.toString());
    yCDebug(SKINMESHCLIENT) << "SkinMeshClient running at" << static_cast<int>(1000.0 * getPeriod()) << "ms.";
    mbSimpleDraw=false;

    sensorsNum=0;
    for (int t=0; t<MAX_SENSOR_NUM; ++t)
    {
        sensor[t]=NULL;
    }
}

bool SkinMeshClient::configureFromSearchable(yarp::os::Searchable& config)
{
    if (m_configured)
    {
        return true;
    }

    mbSimpleDraw=config.check("light");

    const std::string localPrefix = makeLocalPrefix(config);
    const std::string part_virtual = localPrefix + "_virtual:i";

    // Ideally, we would use a --virtual flag. since this would make the skinmanager xml file unflexible,
    // let's keep the code structure without incurring in any problem whatsoever
    // if (config.check("virtual"))
    if (true)
    {
        skin_port_virtual.open(part_virtual);
    }

    if (!configureSkinClient(config, localPrefix))
    {
        return false;
    }

    int width =config.check("width") ? config.find("width").asInt32() : 320;
    int height=config.check("height") ? config.find("height").asInt32() : 320;

    bool useCalibration = config.check("useCalibration");
    if (useCalibration)
    {
        yCInfo(SKINMESHCLIENT) << "Using calibrated skin values (0-255)";
    }
    else
    {
        yCDebug(SKINMESHCLIENT) << "Using raw skin values (255-0)";
    }

    Bottle *color = config.find("color").asList();
    unsigned char r=255, g=0, b=0;
    if(color)
    {
        if(color->size()<3 || !color->get(0).isInt32() || !color->get(1).isInt32() || !color->get(2).isInt32())
        {
            yCError(SKINMESHCLIENT) << "Error while reading the parameter color: three integer values should be specified"
                                    << color->toString();
        }
        else
        {
            r = color->get(0).asInt32();
            g = color->get(1).asInt32();
            b = color->get(2).asInt32();
            yCInfo(SKINMESHCLIENT) << "Using specified color:" << r << g << b;
        }
    }
    else
    {
        yCDebug(SKINMESHCLIENT) << "Using red as default color.";
    }
    defaultColor.push_back(r);
    defaultColor.push_back(g);
    defaultColor.push_back(b);

    skinThreshold = config.check("skinThreshold")?config.find("skinThreshold").asFloat64():SKIN_THRESHOLD;
    yCDebug(SKINMESHCLIENT) << "Skin threshold set to" << skinThreshold;

    yarp::os::Bottle sensorSetConfig=config.findGroup("SENSORS").tail();

    for (int t=0; t<sensorSetConfig.size(); ++t)
    {
        yarp::os::Bottle sensorConfig(sensorSetConfig.get(t).toString());
        std::string type(sensorConfig.get(0).asString());

        if (type=="triangle"       ||
            type=="fingertip"      ||
            type=="fingertip2L"    ||
            type=="fingertip2R"    ||
            type=="fingertip3L"    ||
            type=="fingertip3R"    ||
            type=="fingertip4L"    ||
            type=="fingertip4R"    ||
            type=="fingertipMID"   ||
            type=="triangle_10pad" ||
            type=="quad16"         ||
            type=="palmR"          ||
            type=="fakePalm"       ||
            type=="palmL"          ||
            type == "cer_sh_pdl"   ||
            type == "cer_sh_pdr"   ||
            type == "cer_sh_pp"    ||
            type == "cer_sh_td"    ||
            type == "cer_sh_tp")
        {
            int    id=sensorConfig.get(1).asInt32();
            double xc=sensorConfig.get(2).asFloat64();
            double yc=sensorConfig.get(3).asFloat64();
            double th=sensorConfig.get(4).asFloat64();
            double gain=sensorConfig.get(5).asFloat64();
            int    lrMirror=sensorConfig.get(6).asInt32();
            int    layoutNum=sensorConfig.get(7).asInt32();

            yCDebug(SKINMESHCLIENT) << type << id << gain;

            if (id>=0 && id<MAX_SENSOR_NUM)
            {
                if (sensor[id])
                {
                    yCError(SKINMESHCLIENT) << "Triangle" << id << "already exists.";
                }
                else
                {
                    if (type=="triangle")
                    {
                        sensor[id]=new Triangle(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type=="triangle_10pad")
                    {
                        sensor[id]=new Triangle_10pad(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type=="fingertip")
                    {
                        sensor[id]=new Fingertip(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type=="fingertip2L")
                    {
                        sensor[id]=new Fingertip2L(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type=="fingertip2R")
                    {
                        sensor[id]=new Fingertip2R(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type=="fingertip3L")
                    {
                        sensor[id]=new Fingertip3L(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type=="fingertip3R")
                    {
                        sensor[id]=new Fingertip3R(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type=="fingertip4L")
                    {
                        sensor[id]=new Fingertip4L(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type=="fingertip4R")
                    {
                        sensor[id]=new Fingertip4R(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    if (type=="fingertipMID")
                    {
                        sensor[id]=new FingertipMID(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type=="quad16")
                    {
                        sensor[id]=new Quad16(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type == "cer_sh_pdl")
                    {
                        sensor[id] = new CER_SH_PDL(xc, yc, th, gain, layoutNum, lrMirror);
                    }
                    else if (type=="palmR")
                    {
                        sensor[id]=new PalmR(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type=="fakePalm")
                    {
                        sensor[id]=new fakePalm(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type=="palmL")
                    {
                        sensor[id]=new PalmL(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type == "cer_sh_pdl")
                    {
                        sensor[id] = new CER_SH_PDL(xc, yc, th, gain, layoutNum, lrMirror);
                    }
                    else if (type == "cer_sh_pdr")
                    {
                        sensor[id] = new CER_SH_PDR(xc, yc, th, gain, layoutNum, lrMirror);
                    }
                    else if (type == "cer_sh_pp")
                    {
                        sensor[id] = new CER_SH_PP(xc, yc, th, gain, layoutNum, lrMirror);
                    }
                    else if (type == "cer_sh_td")
                    {
                        sensor[id] = new CER_SH_TD(xc, yc, th, gain, layoutNum, lrMirror);
                    }
                    else if (type == "cer_sh_tp")
                    {
                        sensor[id] = new CER_SH_TP(xc, yc, th, gain, layoutNum, lrMirror);
                    }

                    sensor[id]->setCalibrationFlag(useCalibration);
                    ++sensorsNum;
                }
            }
            else
            {
                yCWarning(SKINMESHCLIENT) << id << "is invalid triangle Id [0:" << MAX_SENSOR_NUM - 1 << "].";
            }
        }
        else
        {
            yCWarning(SKINMESHCLIENT) << "Sensor type" << type << "unknown, discarded.";
        }
    }

    int max_tax=0;
    for (int t=0; t<MAX_SENSOR_NUM; ++t)
    {

        if (sensor[t])
        {
            sensor[t]->min_tax=max_tax;
            max_tax = sensor[t]->min_tax+sensor[t]->get_nTaxels();
            sensor[t]->max_tax=max_tax-1;
            sensor[t]->setColor(r, g, b);
        }
        else
        {
            //this deals with the fact that some traingles can be not present,
            //but they anyway broadcast an array of zeros...
            max_tax += 12;
        }
    }

    resize(width,height);

    m_configured = true;
    return true;
}

bool SkinMeshClient::configureSkinClient(yarp::os::Searchable& config, const std::string& localPrefix)
{
    const std::string local = config.check("local") ? config.find("local").asString() : localPrefix;
    const bool externalConnection = config.check("externalConnection") ? config.find("externalConnection").asBool() : true;
    m_externalConnection = externalConnection;

    if (!externalConnection && !config.check("remote"))
    {
        yCError(SKINMESHCLIENT) << "SkinMeshClient: externalConnection false requires a remote MAS server prefix.";
        return false;
    }

    yarp::os::Property clientOptions;
    clientOptions.put("device", "multipleanalogsensorsclient");
    clientOptions.put("local", local);
    clientOptions.put("remote", externalConnection ? local : config.find("remote").asString());
    clientOptions.put("timeout", config.check("timeout") ? config.find("timeout").asFloat64() : getPeriod() * 2.0);
    clientOptions.put("externalConnection", externalConnection);
    if (config.check("carrier")) //TODO: check carrier if it is the correct one - for now we are using default tcp
    {
        clientOptions.put("carrier", config.find("carrier").asString());
    }

    if (!_multipleAnalogSensorsClientDevice.open(clientOptions))
    {
        yCError(SKINMESHCLIENT) << "SkinMeshClient: unable to open multipleanalogsensorsclient with options"
                 << clientOptions.toString();
        return false;
    }

    yarp::dev::ISkinPatches* clientSkinPatches = nullptr;
    if (!_multipleAnalogSensorsClientDevice.view(clientSkinPatches) || clientSkinPatches == nullptr)
    {
        yCError(SKINMESHCLIENT) << "SkinMeshClient: unable to view ISkinPatches from multipleanalogsensorsclient.";
        return false;
    }

    if (externalConnection)
    {
        remappedMASInterfaces.iskinPatchesSensors = clientSkinPatches;
        yCInfo(SKINMESHCLIENT) << "Waiting for external MAS streaming connection on" << local + "/measures:i";
        yCInfo(SKINMESHCLIENT) << "Internal MultipleAnalogSensorsRemapper is not used with externalConnection true"
                               << "because MultipleAnalogSensorsClient metadata is unavailable in that mode.";
        return true;
    }

    yarp::os::Bottle skinPatchNames;
    size_t skinPatchCount;
    if (!clientSkinPatches->getNrOfSkinPatches(skinPatchCount))
    {
        yCError(SKINMESHCLIENT) << "SkinMeshClient: unable to retrieve the number of skin patches from the multipleanalogsensorsclient.";
        return false;
    }
    yCDebug(SKINMESHCLIENT) << "Number of skin patches available from the multipleanalogsensorsclient:" << skinPatchCount;

    if (skinPatchCount == 0)
    {
        yCError(SKINMESHCLIENT) << "SkinMeshClient: no skin patches available from the multipleanalogsensorsclient.";
        return false;
    }

    for (size_t skinPatchIndex = 0; skinPatchIndex < skinPatchCount; ++skinPatchIndex)
    {
        std::string skinPatchName;
        if (!clientSkinPatches->getSkinPatchName(skinPatchIndex, skinPatchName))
        {
            yCError(SKINMESHCLIENT) << "SkinMeshClient: unable to retrieve name for skin patch"
                                        << skinPatchIndex
                                        << "from multipleanalogsensorsclient.";
            return false;
        }

        yCDebug(SKINMESHCLIENT) << "Name of found skin patch:" << skinPatchName;
        skinPatchNames.addString(skinPatchName);
    }

    if (skinPatchNames.size() != static_cast<int>(skinPatchCount))
    {
        yCError(SKINMESHCLIENT) << "SkinMeshClient: inconsistent skin patch metadata. Expected"
                                    << skinPatchCount
                                    << "names but collected"
                                    << skinPatchNames.size();
        return false;
    }

    yarp::os::Property remapperOptions;
    remapperOptions.put("device", "multipleanalogsensorsremapper");

    yarp::os::Bottle skinPatchNamesOption;
    yarp::os::Bottle& skinPatchNamesList = skinPatchNamesOption.addList();
    for (int i = 0; i < skinPatchNames.size(); ++i)
    {
        skinPatchNamesList.addString(skinPatchNames.get(i).asString());
    }
    remapperOptions.put("SkinPatchesNames", skinPatchNamesOption.get(0));

    if (!_multipleAnalogSensorsRemapperDevice.open(remapperOptions))
    {
        yCError(SKINMESHCLIENT) << "SkinMeshClient: unable to open multipleanalogsensorsremapper with options"
                                    << remapperOptions.toString();
        return false;
    }

    if (!_multipleAnalogSensorsRemapperDevice.view(remappedMASInterfaces.imultwrap) ||
        remappedMASInterfaces.imultwrap == nullptr)
    {
        yCError(SKINMESHCLIENT) << "SkinMeshClient: unable to view IMultipleWrapper from multipleanalogsensorsremapper.";
        return false;
    }

    yarp::dev::PolyDriverList clientDeviceList;
    clientDeviceList.push(&_multipleAnalogSensorsClientDevice, "skin_mas_client");
    if (!remappedMASInterfaces.imultwrap->attachAll(clientDeviceList))
    {
        yCError(SKINMESHCLIENT) << "SkinMeshClient: unable to attach multipleanalogsensorsclient to remapper.";
        return false;
    }

    if (!_multipleAnalogSensorsRemapperDevice.view(remappedMASInterfaces.iskinPatchesSensors) ||
        remappedMASInterfaces.iskinPatchesSensors == nullptr)
    {
        yCError(SKINMESHCLIENT) << "SkinMeshClient: unable to view ISkinPatches from multipleanalogsensorsremapper.";
        return false;
    }

    size_t remappedSkinPatchCount;
    if (!remappedMASInterfaces.iskinPatchesSensors->getNrOfSkinPatches(remappedSkinPatchCount))
    {
        yCError(SKINMESHCLIENT) << "SkinMeshClient: unable to retrieve the number of skin patches from the multipleanalogsensorsremapper.";
        return false;
    }
    if (remappedSkinPatchCount != skinPatchCount)
    {
        yCError(SKINMESHCLIENT) << "SkinMeshClient: MultipleAnalogSensorsRemapper exposes"
                                    << remappedSkinPatchCount
                                    << "skin patches, expected"
                                    << skinPatchCount;
        return false;
    }

    yCInfo(SKINMESHCLIENT) << "Reading skin patches through MultipleAnalogSensorsRemapper:" << skinPatchNames.toString();
    return true;
}

bool SkinMeshClient::readSkinPatches(yarp::sig::Vector& skinValue) const
{
    if (remappedMASInterfaces.iskinPatchesSensors == nullptr)
    {
        return false;
    }

    size_t skinPatchCount;
    if (!remappedMASInterfaces.iskinPatchesSensors->getNrOfSkinPatches(skinPatchCount))
    {
        yCError(SKINMESHCLIENT) << "SkinMeshClient: unable to retrieve the number of skin patches from the multipleanalogsensorsclient.";
        return false;
    }
    if (skinPatchCount == 0)
    {
        const double now = yarp::os::Time::now();
        if (now - m_lastNoSkinPatchWarningTime >= noSkinPatchWarningPeriod)
        {
            if (m_externalConnection)
            {
                yCWarning(SKINMESHCLIENT) << "SkinMeshClient: no skin patches available yet from the MAS client."
                                              << "Waiting for the external streaming connection.";
            }
            else
            {
                //TODO: decide this implementation detail: if externalConnection is false, it means that we are using the internal remapper, which should always expose the same skin patches as the client, so if we have no skin patches it means that the client has no skin patches, and this is something that we want to warn about. If externalConnection is true, instead, we are relying on an external streaming connection, so it might be the case that the client has skin patches but we haven't established the streaming connection yet, so in that case we want to warn about the fact that we have no skin patches available yet and we are waiting for the streaming connection to be established.
                // thus in this case it should an Error and the sw should exit, but for now let's keep it as a warning and let's wait for the streaming connection, because this is what happens in the current implementation of the skinmanager xml file, and we don't want to break it without a good reason.
                yCWarning(SKINMESHCLIENT) << "SkinMeshClient: no skin patches available from the active MAS skin patches interface.";
            }
            m_lastNoSkinPatchWarningTime = now;
        }
        return false;
    }

    std::vector<size_t> skinPatchSizes(skinPatchCount, 0);
    size_t totalSkinPatchSize = 0;
    for (size_t skinPatchIndex = 0; skinPatchIndex < skinPatchCount; ++skinPatchIndex)
    {
        skinPatchSizes[skinPatchIndex] = remappedMASInterfaces.iskinPatchesSensors->getSkinPatchSize(skinPatchIndex);
        totalSkinPatchSize += skinPatchSizes[skinPatchIndex];
    }

    skinValue.resize(totalSkinPatchSize, 0.0);
    size_t skinValueOffset = 0;
    bool gotAnyPatch = false;

    for (size_t skinPatchIndex = 0; skinPatchIndex < skinPatchCount; ++skinPatchIndex)
    {
        if (remappedMASInterfaces.iskinPatchesSensors->getSkinPatchStatus(skinPatchIndex) != yarp::dev::MAS_OK)
        {
            skinValueOffset += skinPatchSizes[skinPatchIndex];
            continue;
        }

        yarp::sig::Vector skinPatchValue;
        double timestamp = 0.0;
        if (!remappedMASInterfaces.iskinPatchesSensors->getSkinPatchMeasure(skinPatchIndex, skinPatchValue, timestamp))
        {
            skinValueOffset += skinPatchSizes[skinPatchIndex];
            continue;
        }

        const size_t taxelsToCopy = std::min(skinPatchSizes[skinPatchIndex], skinPatchValue.size());
        for (size_t i = 0; i < taxelsToCopy; ++i)
        {
            skinValue[skinValueOffset + i] = skinPatchValue[i];
        }

        skinValueOffset += skinPatchSizes[skinPatchIndex];
        gotAnyPatch = true;
    }

    return gotAnyPatch;
}

bool SkinMeshClient::start()
{
    if (!configureFromSearchable(m_config))
    {
        close();
        return false;
    }

    return runModuleThreaded() == 0;
}

void SkinMeshClient::stop()
{
    stopModule(true);
}

bool SkinMeshClient::configure(yarp::os::ResourceFinder &rf)
{
    return configureFromSearchable(rf);
}

double SkinMeshClient::getPeriod()
{
    return m_period;
}

bool SkinMeshClient::updateModule()
{
    std::lock_guard<std::mutex> lck(mtx);

    bool gotRealData=false;
    bool gotVirtualData=false;
    yarp::sig::Vector skin_value;
    yarp::sig::Vector skin_value_virtual;
    std::vector<unsigned char> skin_color_virtual;

    if (readSkinPatches(skin_value))
    {
        yCTrace(SKINMESHCLIENT) << "Reading real contacts from ISkinPatches...";
        gotRealData=true;
    }

    // Read data from the virtual contacts
    if (Bottle *input_virtual=skin_port_virtual.read(false))
    {
        Bottle *data_virtual  = input_virtual->get(0).asList();
        Bottle *color_virtual = input_virtual->get(1).asList();
        if (data_virtual != nullptr && color_virtual != nullptr && color_virtual->size() >= 3)
        {
            yCTrace(SKINMESHCLIENT) << "Reading from virtual contacts...";
            gotVirtualData=true;

            skin_value_virtual.resize(data_virtual->size(),0.0);

            for (int i=0; i<data_virtual->size(); i++)
            {
                skin_value_virtual[i] = data_virtual->get(i).asFloat64();
            }

            for (int i = 0; i<color_virtual->size(); i++)
            {
                skin_color_virtual.push_back(color_virtual->get(i).asInt32());
            }

            yCTrace(SKINMESHCLIENT) << "Virtual contacts:" << skin_value_virtual.toString(3,3);
            yCTrace(SKINMESHCLIENT) << "Virtual contacts color:" << skin_color_virtual[0] << skin_color_virtual[1] << skin_color_virtual[2];
        }
    }

    if (!gotRealData && !gotVirtualData)
    {
        for (int sensorId = 0; sensorId < MAX_SENSOR_NUM; sensorId++)
        {
            if (sensor[sensorId] != nullptr)
            {
                sensor[sensorId]->clearActivation();
            }
        }
        return true;
    }

    for (int sensorId=0; sensorId<MAX_SENSOR_NUM; sensorId++)
    {
        if (sensor[sensorId]==0) continue;

        // First, let's see if this touchSensor is over threshold in the real skin
        bool isRealSensorOverThreshold=false;
        if (gotRealData)
        {
            for (int i=sensor[sensorId]->min_tax; i<=sensor[sensorId]->max_tax && i<static_cast<int>(skin_value.size()); i++)
            {
                if (skin_value[i]>skinThreshold)
                {
                    isRealSensorOverThreshold = true;
                    break;
                }
            }
        }

        // Second, let's see if this touchSensor is over threshold in the virtual skin
        bool isVirtualSensorOverThreshold=false;
        if (gotVirtualData)
        {
            for (int i=sensor[sensorId]->min_tax; i<=sensor[sensorId]->max_tax && i<static_cast<int>(skin_value_virtual.size()); i++)
            {
                if (skin_value_virtual[i]>skinThreshold)
                {
                    isVirtualSensorOverThreshold = true;
                    break;
                }
            }
        }

        // Then, let's process the sensor with either the real or the virtual (or none)
        // EVerything will be clearly decoupled, a couple lines more do not hurt

        // If there are both, handle them
        if (gotRealData && gotVirtualData)
        {
            // Then, let's process the sensor with either the real or the virtual skin
            for (int i=sensor[sensorId]->min_tax; i<=sensor[sensorId]->max_tax; i++)
            {
                int curr_tax = i-sensor[sensorId]->min_tax;

                // The GUI layout can describe more taxels than the current MAS sample carries,
                // especially while clients are connecting or when only a subset of patches is available.
                if (isRealSensorOverThreshold && i<static_cast<int>(skin_value.size()))
                {
                    sensor[sensorId]->setActivationFromPortData(skin_value[i],curr_tax);
                    sensor[sensorId]->setColor(defaultColor[0],defaultColor[1],defaultColor[2]);
                }
                else if(isVirtualSensorOverThreshold && i<static_cast<int>(skin_value_virtual.size()))
                {
                    sensor[sensorId]->setActivationFromPortData(skin_value_virtual[i],curr_tax);
                    sensor[sensorId]->setColor(skin_color_virtual[0],skin_color_virtual[1],skin_color_virtual[2]);
                }
                else if (i<static_cast<int>(skin_value.size()))
                {
                    sensor[sensorId]->setActivationFromPortData(skin_value[i],curr_tax);
                    sensor[sensorId]->setColor(defaultColor[0],defaultColor[1],defaultColor[2]);
                }
            }
        }
        else if (gotRealData || gotVirtualData)
        {
            for (int i=sensor[sensorId]->min_tax; i<=sensor[sensorId]->max_tax; i++)
            {
                int curr_tax = i-sensor[sensorId]->min_tax;

                if (gotRealData && i<static_cast<int>(skin_value.size()))
                {
                    sensor[sensorId]->setActivationFromPortData(skin_value[i],curr_tax);
                    sensor[sensorId]->setColor(defaultColor[0],defaultColor[1],defaultColor[2]);
                }
                else if (gotVirtualData && i<static_cast<int>(skin_value_virtual.size()))
                {
                    sensor[sensorId]->setActivationFromPortData(skin_value_virtual[i],curr_tax);
                    sensor[sensorId]->setColor(skin_color_virtual[0],skin_color_virtual[1],skin_color_virtual[2]);
                }
            }
        }
    }

    return true;
}

bool SkinMeshClient::close()
{
    yCDebug(SKINMESHCLIENT) << "SkinMeshClient releasing...";

    if (remappedMASInterfaces.imultwrap != nullptr)
    {
        remappedMASInterfaces.imultwrap->detachAll();
    }
    remappedMASInterfaces.imultwrap = nullptr;
    remappedMASInterfaces.iskinPatchesSensors = nullptr;

    _multipleAnalogSensorsRemapperDevice.close();
    _multipleAnalogSensorsClientDevice.close();

    skin_port.close();
    skin_port_virtual.close();

    m_configured = false;
    yCDebug(SKINMESHCLIENT) << "SkinMeshClient released.";
    return true;
}
