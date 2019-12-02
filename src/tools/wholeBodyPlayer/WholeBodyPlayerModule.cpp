/*
 * Copyright (C) 2006-2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "WholeBodyPlayerModule.h"
#include <algorithm>
#include <yarp/dev/GenericVocabs.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;


static const std::vector<std::string> partsVec {"head","torso","left_arm","right_arm","left_leg","right_leg"};

double WholeBodyPlayerModule::getPeriod() {
    return 0.001;
}

bool WholeBodyPlayerModule::updateModule() {

    for (auto& rep : m_replayerVec){
        if (rep.m_replayPort->m_state == state::fatal_error) {
            yError()<<"wholeBodyPlayer: the port"<<rep.m_replayPort->getName()<<"is closed because something went wrong.. closing";
            return false;
        } else if (rep.m_replayPort->m_state == state::error) {
            bool ok{true};
            response.clear();
            // pause
            ok &= m_rpcPort.write(reqPause, response);
            if (!ok || response.get(0).asVocab() != VOCAB_OK) {
                yError()<<"wholeBodyPlayer: the port"<<rep.m_replayPort->getName()<<"is closed because the pause request failed.. closing";
                return false;
            }

            // position move fallback
            ok &= rep.m_replayPort->positionMoveFallback();
            if (!ok) {
                yError()<<"wholeBodyPlayer: the port"<<rep.m_replayPort->getName()<<"is closed because the fallback failed.. closing";
                return false;
            }

            // start
            response.clear();
            ok &= m_rpcPort.write(reqPlay, response);
            if (!ok || response.get(0).asVocab() != VOCAB_OK) {
                yError()<<"wholeBodyPlayer: the port"<<rep.m_replayPort->getName()<<"is closed because the start request failed.. closing";
                return false;
            }
            rep.m_replayPort->m_state = state::ok;
        }
    }

    return true;
}

bool WholeBodyPlayerModule::configure(yarp::os::ResourceFinder& rf) {
    auto robot = rf.check("robot",Value("icub")).asString();
    auto name = rf.check("name",Value("wholeBodyPlayerModule")).asString();

    auto partsBot = rf.find("parts").asList();
    if (!partsBot || partsBot->isNull())
    {
       yError()<<"wholeBodyPlayerModule: missing parts parameter, please specify as list";
       return false;
    }

    m_replayerVec.resize(partsBot->size());

    for (size_t i = 0; i<partsBot->size(); i++) {
        auto partStr = partsBot->get(i).asString();
        if(partsVec.end() == std::find(partsVec.begin(), partsVec.end(), partStr))
        {
            yError()<<"wholeBodyPlayerModule: the part"<<partStr<<"is not available";
            return false;
        }
        if (!m_replayerVec[i].open(robot, partStr, name))
        {
            yError()<<"wholeBodyPlayerModule: failed to open one replayer.. closing.";
            return false;
        }
    }

    if (!m_rpcPort.open("/"+name+"/rpc:o")) {
        yError()<<"wholeBodyPlayerModule: failed to open"<<m_rpcPort.getName();
        return false;
    }

    if (!Network::connect(m_rpcPort.getName(), "/yarpdataplayer/rpc:i")) {
        yError()<<"wholeBodyPlayerModule: failed to connect to the yarpdataplayer, is it running?";
        return false;
    }

    return true;
}

bool WholeBodyPlayerModule::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply) {
    return true;
}

bool WholeBodyPlayerModule::interruptModule() {
    for (auto& rep : m_replayerVec){
        rep.m_replayPort->interrupt();
    }
    m_rpcPort.interrupt();
    return true;
}

bool WholeBodyPlayerModule::close() {
    for (auto& rep : m_replayerVec){
        rep.close();
    }
    m_rpcPort.close();
    return true;
}
