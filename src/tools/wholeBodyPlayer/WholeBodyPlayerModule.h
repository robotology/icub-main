/*
 * Copyright (C) 2006-2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */
/**
 *
 * @ingroup icub_tools
 * \defgroup wholeBodyPlayer wholeBodyPlayer
 * A module to replay the encoder data coming from yarpdataplayer(http://yarp.it/yarpdataplayer.html).
 *
 * \section disclaimer Disclaimer
 * \warning This software has been fully tested only on simulators, it is in a beta state for the use on the real robots.
 * \section intro_sec Description
 * This module opens a remote_controlboard (http://yarp.it/classyarp_1_1dev_1_1RemoteControlBoard.html) for each <part>
 * and commands it in position direct control to specific positions received by the input port related to the part.
 *
 * The position direct control ensure a more reliable repeatability of the movement but on the other hand can be
 * dangerous if used without guards. In this sense this module checks if the jump from the current state of each joint and
 * the target position is not greater of a tolerance. In this case for security reasons it pauses yarpdataplayer, moves
 * the joint(s) in position control for reaching the target, then send a play command to the yarpdataplayer and resume
 * the position direct control. If the reaching of the target fails also in position control, then a fatal error is
 * triggered and the module closes.
 *
 * \section params Parameters
 * --robot  The name of the robot to be controlled (e.g icub, icubSim, cer). icub is the default value.
 * --name   The prefix to be given to the ports of the module. wholeBodyPlayer is the default value.
 * --parts  List of parts to be controlled. It has to be from one to all the following parts: "(head torso left_arm right_arm left_leg right_arm)"
 *
 * \section ports Ports
 * This module open one port for each part controlled, from which it receive data from yarpdataplayer.
 * They have this name:
 *
 * /<name>/<part>/state:i
 *
 * \section tested_os_sec Tested OS
 * Windows, Linux
 * \author Nicolo' Genesio
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