/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Prediction module implementation for wrapping an executable around IMachineLearner classes.
 *
 */

#include <stdexcept>
#include <cassert>

#include <yarp/IOException.h>

#include "iCub/PredictModule.h"

namespace iCub {
namespace contrib {
namespace learningmachine {

bool PredictProcessor::read(ConnectionReader& connection) {
    assert(this->getMachine() != (IMachineLearner *) 0);
    Vector input, prediction;
    bool ok = input.read(connection);
    if(!ok) {
        return false;
    }
    try {
        prediction = this->getMachine()->predict(input);
    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }

    ConnectionWriter* replier = connection.getWriter();
    if(replier != (ConnectionWriter*) 0) {
        prediction.write(*replier);
    }
    return true;
}


void PredictModule::exitWithHelp(std::string error) {
    int errorCode = 0;
    if(error != "") {
        std::cout << "Error: " << error << std::endl;
        errorCode = 1;
    }
    std::cout << "Available options for prediction module" << std::endl;
    std::cout << "--help                 Display this help message" << std::endl;
    std::cout << "--port pfx             Prefix for registering the ports" << std::endl;
    std::cout << "--modelport port       Model port of the training module" << std::endl;
    exit(errorCode);
}

void PredictModule::registerAllPorts() {
    this->registerPort(this->model_in, this->portPrefix + "/model:i");
    this->registerPort(this->predict_inout, this->portPrefix + "/predict:io");
    this->predict_inout.setStrict();
    this->registerPort(this->cmd_in, this->portPrefix + "/cmd:i");
}

void PredictModule::unregisterAllPorts() {
    this->model_in.close();
    this->cmd_in.close();
    this->predict_inout.close();
}

bool PredictModule::interruptModule() {
    this->cmd_in.interrupt();
    this->predict_inout.interrupt();
    this->model_in.interrupt();
    return true;
}

bool PredictModule::open(Searchable& opt) {
    // read for the general specifiers:
    Value* val;

    // check for help request
    if(opt.check("help")) {
        this->exitWithHelp();
    }

    // check for port specifier: portSuffix
    if(opt.check("port", val)) {
        this->portPrefix = val->asString().c_str();
    }

    // register ports before connecting
    this->registerAllPorts();

    // check for model input port specifier and connect if found
    if(opt.check("modelport", val)) {
        Network::connect(val->asString().c_str(), this->model_in.where().getName().c_str());
    }

    // add reader for models
    this->model_in.setReader(*this->machinePortable);

    // add replier for incoming data (prediction requests)
    this->predictProcessor.setMachinePortable(this->machinePortable);
    this->predict_inout.setReplier(this->predictProcessor);

    // attach to the incoming command port
    this->attach(cmd_in);

    return true;
}



bool PredictModule::respond(const Bottle& cmd, Bottle& reply) {
    bool success = false;

    try {
        switch(cmd.get(0).asVocab()) {
            case VOCAB4('h','e','l','p'): // print help information 
                // why this vocab?
                reply.addVocab(Vocab::encode("help"));

                reply.addString("Training module configuration options");
                reply.addString("  help                  Displays this message");
                reply.addString("  reset                 Resets the machine to its current state");
                reply.addString("  stat                  Outputs statistics of the machine");
                reply.addString("  load fname            Loads a machine from a file");
                //reply.addString(this->getMachine()->getConfigHelp().c_str());
                success = true;
                break;

            case VOCAB4('c','l','e','a'): // clear the machine
            case VOCAB3('c','l','r'):
            case VOCAB4('r','e','s','e'):
            case VOCAB3('r','s','t'):
                // NOTE TO SELF: possibly set the machine to a null pointer for prediction
                this->getMachine()->reset();
                reply.addString("Machine reset.");
                success = true;
                break;

            case VOCAB4('i','n','f','o'): // information
            case VOCAB4('s','t','a','t'): // print statistics
                {
                reply.addVocab(Vocab::encode("help"));
                reply.addString("Machine Information: ");
                reply.addString(this->getMachine()->getInfo().c_str());
                success = true;
                break;
                }

            case VOCAB4('l','o','a','d'): // load
                { // prevent identifier initialization to cross borders of case
                reply.add(Value::makeVocab("help"));
                std::string replymsg = std::string("Loading machine from '") + cmd.get(1).asString().c_str() + "'... " ;
                if(!cmd.get(1).isString()) {
                    replymsg += "failed";
                } else {
                    this->getMachinePortable()->readFromFile(cmd.get(1).asString().c_str());
                    replymsg += "succeeded";
                }
                reply.addString(replymsg.c_str());
                success = true;
                break;
                }

            default:
                break;

        }
    } catch(const std::exception& e) {
        std::string msg = std::string("Error: ") + e.what();
        reply.addString(msg.c_str());
        success = true;
    }
    return success;
}



} // learningmachine
} // contrib
} // iCub
