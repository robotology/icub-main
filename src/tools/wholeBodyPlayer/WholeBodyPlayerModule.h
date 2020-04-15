/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
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
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/PolyDriver.h>

#include <atomic>
#include <vector>
#include <cmath>
#include <memory>
#include <mutex>
#include <yarp/os/RpcClient.h>

constexpr double tolerance = 5.0; //degrees

enum class state : std::uint8_t {
    ok = 0,
    error,
    fatal_error
};

class ReplayPort : public  yarp::os::BufferedPort<yarp::os::Bottle>
{
public:
    ReplayPort(std::string partName, yarp::dev::IPositionDirect* iPosDir,
               yarp::dev::IEncoders* iEnc, yarp::dev::IControlMode* iCM,
               yarp::dev::IPositionControl* iPosControl,
               yarp::dev::IControlLimits* iControlLimits, bool simulator=false) :
                                                                            m_partName(std::move(partName)),
                                                                            m_posDir(iPosDir),
                                                                            m_enc(iEnc),
                                                                            m_CM(iCM),
                                                                            m_posControl(iPosControl),
                                                                            m_controlLimits(iControlLimits),
                                                                            m_simulator(simulator)

    {
        if (m_posDir && m_enc && m_CM && m_controlLimits) {
            // Get num of axes
            m_posDir->getAxes(&m_numAxes);
            // Set in position direct
            std::vector<int> cms (m_numAxes, VOCAB_CM_POSITION_DIRECT);
            m_CM->setControlModes(cms.data());
            // Allocate vectors
            m_currState.resize(m_numAxes);
            m_nextState.resize(m_numAxes);
            max.resize(m_numAxes);
            min.resize(m_numAxes);
            for (size_t i = 0; i<m_numAxes; i++) {
               m_controlLimits->getLimits(i,&min[i],&max[i]);
            }
        }
        yarp::os::BufferedPort<yarp::os::Bottle>::useCallback();
        yarp::os::BufferedPort<yarp::os::Bottle>::setStrict(true);
        m_isArm = m_partName.find("arm") != std::string::npos;
    }

    using yarp::os::TypedReaderCallback<yarp::os::Bottle>::onRead;
    void onRead(yarp::os::Bottle& datum) override
    {
        if (m_state==state::ok && m_posDir && !datum.isNull()) {

            bool ok = m_enc->getEncoders(m_currState.data());
            m_mutex.lock();
            for (size_t i=0; i<datum.size(); i++) {
                m_nextState[i] = datum.get(i).asDouble();
                if (m_nextState[i]<min[i] || m_nextState[i]> max[i]) {
                    yWarning()<<"ReplayPort: trying to move joint"<<i<<"of"<<m_partName<<" to "<<m_nextState[i]<<", it exceeds the limits, skipping...";
                    continue;
                }
                auto delta = std::fabs(m_nextState[i] - m_currState[i]);
                ok &= delta < tolerance; // TODO improve the check calculating the distance between the 2 vector !!!
                if (!ok && !m_simulator && (!m_isArm || i < 5)) { // 5 is for ignoring the hands in the security check
                    yWarning()<<"ReplayPort: joint"<<i<<"of"<<m_partName<<"is too far to the target position";
                    yWarning()<<"Desired: "<<datum.get(i).asDouble()<<"current: "<<m_currState[i]
                    <<"delta: "<<delta<<"Trying to reach "
                                                                                                        "it in Position Control, "
                                                                                                        "the playback will be paused";
                    m_state = state::error;
                    ok = true;

                }
            }
            m_mutex.unlock();
            if (m_state == state::ok) {
                m_posDir->setPositions(m_nextState.data());
            }
        }
    }

    bool positionMoveFallback(){
        bool ok = true;
        std::vector<int> cms (m_numAxes, VOCAB_CM_POSITION);
        ok &= m_CM->setControlModes(cms.data());

        if (!ok) {
            m_state = state::fatal_error;
            return ok;
        }

        m_mutex.lock();
        ok &= m_posControl->positionMove(m_nextState.data());
        m_mutex.unlock();

        if (!ok) {
            m_state = state::fatal_error;
            return ok;
        }
        bool done{false};
        ok &= m_posControl->checkMotionDone(&done);

        if (!ok) {
            m_state = state::fatal_error;
            return ok;
        }

        while (!done) {
            ok &= m_posControl->checkMotionDone(&done);
            if (!ok) {
                m_state = state::fatal_error;
                return ok;
            }
            yarp::os::Time::delay(0.01);
        }

        std::vector<int> cmDir (m_numAxes, VOCAB_CM_POSITION_DIRECT);
        ok &= m_CM->setControlModes(cmDir.data());
        if (!ok) {
            m_state = state::fatal_error;
            return ok;
        }
        return ok;
    }

    std::atomic<state> m_state{state::ok};
private:
    double now{0.0};
    std::vector<double> min, max;
    std::string m_partName{};
    yarp::dev::IPositionDirect* m_posDir{nullptr};
    yarp::dev::IEncoders* m_enc{nullptr};
    yarp::dev::IPositionControl* m_posControl{nullptr};
    yarp::dev::IControlMode* m_CM{nullptr};
    yarp::dev::IControlLimits* m_controlLimits{nullptr};
    std::mutex m_mutex;


    std::vector<double> m_currState, m_nextState;
    int m_numAxes{0};
    bool m_simulator;
    bool m_isArm{false};
};

struct Replayer {
    std::unique_ptr<ReplayPort> m_replayPort{nullptr};
    std::unique_ptr<yarp::dev::PolyDriver> m_remoteControlBoard{nullptr};

    bool open(const std::string& robot, const std::string& part, const std::string& moduleName="wholeBodyPlayer") {
        yarp::os::Property conf {{"device", yarp::os::Value("remote_controlboard")},
                                 {"remote", yarp::os::Value("/"+robot+"/"+part)},
                                 {"local",  yarp::os::Value("/"+moduleName+"/"+part+"/remoteControlBoard")}};

        yarp::dev::IControlMode* iCM{nullptr};
        yarp::dev::IPositionDirect* iPosDir{nullptr};
        yarp::dev::IPositionControl* iPosControl{nullptr};
        yarp::dev::IEncoders* iEnc{nullptr};
        yarp::dev::IControlLimits* iControlLimits{nullptr};
        bool simulator{false};

        if(robot == "icubSim")
            simulator=true;

        m_remoteControlBoard = std::make_unique<yarp::dev::PolyDriver>();

        bool ok = m_remoteControlBoard->open(conf);
        if (!ok) {
            yError()<<"Replayer: failed to open the remote control board for part"<<part;
            return false;
        }
        ok &= m_remoteControlBoard->view(iPosDir);
        ok &= m_remoteControlBoard->view(iEnc);
        ok &= m_remoteControlBoard->view(iCM);
        ok &= m_remoteControlBoard->view(iPosControl);
        ok &= m_remoteControlBoard->view(iControlLimits);

        if (ok)
        {
            m_replayPort = std::make_unique<ReplayPort>(part, iPosDir, iEnc, iCM, iPosControl,iControlLimits, simulator);
            ok &= m_replayPort->open("/"+moduleName+"/"+part+"/state:i");
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
    yarp::os::RpcClient   m_rpcPort;
    yarp::os::Bottle reqPause{"pause"}, reqPlay{"play"}, response;


};