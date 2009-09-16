/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Transform module implementation for wrapping an executable around an 
 * ITransformer. 
 */

#include "iCub/TransformModule.h"
#include <yarp/IOException.h>
#include <stdexcept>

namespace iCub {
namespace contrib {
namespace learningmachine {

void TransformModule::exitWithHelp(std::string error) {
    int errorCode = 0;
    if(error != "") {
        std::cout << "Error: " << error << std::endl;
        errorCode = 1;
    }
    std::cout << "Available options" << std::endl;
    std::cout << "--help                 Display this help message" << std::endl;
    std::cout << "--transformer type     Desired type of transformer" << std::endl;
    std::cout << "--trainport port       Data port for the training samples" << std::endl;
    std::cout << "--predictport port     Data port for the prediction samples" << std::endl;
    std::cout << "--port pfx             Prefix for registering the ports" << std::endl;
    exit(errorCode);
}


void TransformModule::registerAllPorts() {
    //this->registerPort(this->model_out, "/" + this->portPrefix + "/model:o");

    this->registerPort(this->train_in, "/" + this->portPrefix + "/train:i");
    this->train_in.setStrict();

    this->registerPort(this->train_out, "/" + this->portPrefix + "/train:o");
    this->train_out.setStrict();

    this->registerPort(this->predict_inout, "/" + this->portPrefix + "/predict:io");
    this->predict_inout.setStrict();

    this->registerPort(this->predict_relay_inout, "/" + this->portPrefix + "/predict_relay:io");
    //this->predict_relay_inout.setStrict();

    this->registerPort(this->cmd_in, "/" + this->portPrefix + "/cmd:i");
}

void TransformModule::unregisterAllPorts() {
    this->cmd_in.close();
    this->train_in.close();
    this->train_out.close();
    this->predict_inout.close();
    this->predict_relay_inout.close();
}

bool TransformModule::interruptModule() {
    cmd_in.interrupt();
    train_in.interrupt();
    train_out.interrupt();
    predict_inout.interrupt();
    predict_relay_inout.interrupt();
    return true;
}



bool TransformModule::open(Searchable& opt) {
    // read for the general specifiers:
    Value* val;
    std::string transformerName;

    // check for help request
    if(opt.check("help")) {
        this->exitWithHelp();
    }

    // check for port specifier: portSuffix
    if(opt.check("port", val)) {
        this->portPrefix = val->asString().c_str();
    }

    // check for transformer specifier: transformerName
    // NOTE: mandatory!
    if(opt.check("transformer", val)) {
        transformerName = val->asString().c_str();
    } else {
        this->exitWithHelp("no transformer type specified");
    }
    
    // construct transformer
    this->transformer = this->support->getTransformerFactory().create(transformerName);

    // send configuration options to the transformer
    this->getTransformer()->configure(opt);

    // add processor for incoming data (training samples)
    this->trainProcessor.setTransformer(this->transformer);
    this->trainProcessor.setOutputPort(&this->train_out);
    this->train_in.useCallback(trainProcessor);

    // add replier for incoming data (prediction requests)
    this->predictProcessor.setTransformer(this->transformer);
    this->predictProcessor.setOutputPort(&this->predict_relay_inout);
    this->predict_inout.setReplier(this->predictProcessor);

    // register ports
    this->registerAllPorts();

    // check for train data port
    if(opt.check("trainport", val)) {
        Network::connect(this->train_out.where().getName().c_str(), val->asString().c_str());
    } else {
        // add message here if necessary
    }

    // check for predict data port
    if(opt.check("predictport", val)) {
        Network::connect(this->predict_relay_inout.where().getName().c_str(), val->asString().c_str());
    } else {
        // add message here if necessary
    }

    // attach to the incoming command port
    this->attach(cmd_in);

    return true;
}


bool TransformModule::respond(const Bottle& cmd, Bottle& reply) {
    // NOTE: the module class spawns a new thread, which implies that exception
    // handling needs to be done in this thread, so not the 'main' thread.
    bool success = false;

    try {
        switch(cmd.get(0).asVocab()) {
            case VOCAB4('h','e','l','p'): // print help information 
                reply.add(Value::makeVocab("help"));

                reply.addString("Transform module configuration options");
                reply.addString("  help                  Displays this message");
                reply.addString("  reset                 Resets the machine to its current state");
                reply.addString("  stat                  Outputs statistics of the machine");
                reply.addString("  set key val           Sets a configuration option for the transformer");
                reply.addString(this->getTransformer()->getConfigHelp().c_str());
                success = true;
                break;

            case VOCAB4('c','l','e','a'): // clear the machine
            case VOCAB3('c','l','r'):
            case VOCAB4('r','e','s','e'):
            case VOCAB3('r','s','t'):
                this->getTransformer()->reset();
                reply.addString("Transformer reset.");
                success = true;
                break;

            case VOCAB4('s','t','a','t'): // print statistics
                { // prevent identifier initialization to cross borders of case
                reply.add(Value::makeVocab("help"));
                reply.addString("Transformer Stats: ");
                reply.addString(this->getTransformer()->getStats().c_str());
                success = true;
                break;
                }

            case VOCAB3('s','e','t'): // set a configuration option for the machine
                { // prevent identifier initialization to cross borders of case
                Bottle property;
                /*
                 * This is a simple hack to enable multiple parameters The need for this hack lies
                 * in the fact that a group can only be found using findGroup if it is a nested 
                 * list in a Bottle. If the Bottle itself is the list, then the group will _not_ 
                 * be found.
                 */
                property.addList() = cmd.tail();
                std::string replymsg = "Setting configuration option ";
                if(this->getTransformer()->configure(property)) {
                    replymsg += "succeeded";
                } else {
                    replymsg += "failed; please check key and value type.";
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

/*
bool TransformModule::updateModule() {
    bool ok = true;
    return ok;
}*/



} // learningmachine
} // contrib
} // iCub
