/*
 * Copyright (C) 2006-2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */
/**
@ingroup icub_module
\defgroup wholeBodyPlayer wholeBodyPlayer

A module to replay the encoder data coming from yarpdataplayer(http://yarp.it/yarpdataplayer.html).

\section intro_sec Description


\section tested_os_sec Tested OS
Windows, Linux

\author Nicolo' Genesio
*/



#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/PolyDriver.h>

#include <atomic>
#include <vector>
#include <cmath>
#include <memory>

constexpr double tolerance = 200.0; //degrees

class ReplayPort : public  yarp::os::BufferedPort<yarp::os::Bottle>
{
public:
    ReplayPort(std::string partName, yarp::dev::IPositionDirect* iPosDir, yarp::dev::IEncoders* iEnc) :
                                                                            m_partName(std::move(partName)),
                                                                            m_posDir(iPosDir),
                                                                            m_enc(iEnc)
    {
        if (m_posDir && m_enc) {
            m_posDir->getAxes(&m_numAxes);
            m_currState.resize(m_numAxes);
            m_nextState.resize(m_numAxes);
        }
        yarp::os::BufferedPort<yarp::os::Bottle>::useCallback();
    }

    using yarp::os::TypedReaderCallback<yarp::os::Bottle>::onRead;
    void onRead(yarp::os::Bottle& datum) override
    {
        if (!interrupted && m_posDir && !datum.isNull()) {

            bool ok = m_enc->getEncoders(m_currState.data());
            for (size_t i=0; i<datum.size(); i++) {
                m_nextState[i] = datum.get(i).asDouble();
                auto delta = std::fabs(m_nextState[i] - m_currState[i]);
                ok &= delta < tolerance; // TODO improve the check calculating the distance between the 2 vector !!!
                if (!ok) {
                    yError()<<"ReplayPort: joint"<<i<<"of"<<m_partName<<"is too far to the target position";
                    yError()<<"Desired: "<<datum.get(i).asDouble()<<"current: "<<m_currState[i]<<delta<<"the port"
                    <<this->getName()<<"will be closed for safety reason";
                    interrupted =true;
                    return;

                }
            }

            m_posDir->setPositions(m_nextState.data());
//            yDebug()<<"ReplayPort: part"<<m_partName;
//            yDebug()<<"Desired: "<<m_nextState<<"current:"<<m_currState;
        }
    }

    std::atomic_bool interrupted{false};
private:
    double now{0.0};
    std::string m_partName{};
    yarp::dev::IPositionDirect* m_posDir{nullptr};
    yarp::dev::IEncoders* m_enc{nullptr};

    std::vector<double> m_currState, m_nextState;
    int m_numAxes{0};
};

struct Replayer {
    std::unique_ptr<ReplayPort> m_replayPort{nullptr};
    std::unique_ptr<yarp::dev::PolyDriver> m_remoteControlBoard{nullptr};

    bool open(const std::string& robot, const std::string& part, const std::string& moduleName="wholeBodyPlayer") {
        yarp::os::Property conf {{"device", yarp::os::Value("remote_controlboard")},
                                 {"remote", yarp::os::Value("/"+robot+"/"+part)},
                                 {"local",  yarp::os::Value("/"+moduleName+"/"+part+"/remoteControlBoard")}};

        yarp::dev::IControlMode* iCM{nullptr};
        yarp::dev::IPositionDirect* m_iPosDir{nullptr};
        yarp::dev::IEncoders* m_enc{nullptr};

        m_remoteControlBoard = std::make_unique<yarp::dev::PolyDriver>();

        bool ok = m_remoteControlBoard->open(conf);
        if (!ok) {
            yError()<<"Replayer: failed to open the remote control board for part"<<part;
            return false;
        }
        ok &= m_remoteControlBoard->view(m_iPosDir);
        ok &= m_remoteControlBoard->view(m_enc);
        ok &= m_remoteControlBoard->view(iCM);

        if (ok)
        {
            int nJoints;
            ok &= m_iPosDir->getAxes(&nJoints);
            std::vector<int> cms (nJoints, VOCAB_CM_POSITION_DIRECT);
            ok &= iCM->setControlModes(cms.data());
            if (ok) {
                m_replayPort = std::make_unique<ReplayPort>(part, m_iPosDir, m_enc);
                ok &= m_replayPort->open("/"+moduleName+"/"+part+"/state:i");
            }
        }
        return ok;
    }

    void close() {
        m_replayPort->close();
        m_remoteControlBoard->close();
    }
};


class WholeBodyPlayerModule : public yarp::os::RFModule {
public:
    ~WholeBodyPlayerModule() override = default;

    double getPeriod() override;

    bool updateModule() override;

    bool configure(yarp::os::ResourceFinder& rf) override;

    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply) override; // NOT SURE I NEED IT. I need to call this->attach(portRpc)

    bool interruptModule() override;

    bool close() override;

private:
    std::vector<Replayer> m_replayerVec;

};