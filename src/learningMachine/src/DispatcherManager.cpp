/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation for the YARP-aware managing interface for the EventDispatcher
 *
 */

#include <stdexcept>
#include <sstream>

#include <yarp/os/Vocab.h>

#include "iCub/DispatcherManager.h"
#include "iCub/IEventListener.h"

namespace iCub {
namespace learningmachine {


DispatcherManager::DispatcherManager() {
    // cache pointer to event dispatcher
    this->dispatcher = &(EventDispatcher::instance());
    // cache pointer to event listener factory
    this->factory = &(EventListenerFactory::instance());
}

bool DispatcherManager::respond(const Bottle& cmd, Bottle& reply) {
    bool success = false;

    try {
        switch(cmd.get(0).asVocab()) {
            case VOCAB4('h','e','l','p'): // print help information 
                reply.add(Value::makeVocab("help"));

                reply.addString("Event Manager configuration options");
                reply.addString("  help                  Displays this message");
                reply.addString("  add type [type2 ...]  Adds one or more event listeners");
                reply.addString("  remove [all|idx]      Removes event listener at an index or all");
                reply.addString("  set [all|idx]         Configures a listener");
                reply.addString("  stats                 Prints information");
                success = true;
                break;

            case VOCAB3('a','d','d'): // add
                { // prevent identifier initialization to cross borders of case
                Bottle list = cmd.tail();
                for(int i = 0; i < list.size(); i++) {
                    IEventListener* listener = this->factory->clone(list.get(i).asString().c_str());
                    listener->start();
                    this->dispatcher->addListener(listener);
                }
                reply.addString("Successfully added listener(s)");
                success = true;
                break;
                }

            case VOCAB4('r','e','m','o'): // remove
            case VOCAB3('d','e','l'): // del(ete) 
                { // prevent identifier initialization to cross borders of case
                if(cmd.get(1).isInt() && cmd.get(1).asInt() >= 1 && cmd.get(1).asInt() <= this->dispatcher->countListeners()) {
                    this->dispatcher->removeListener(cmd.get(1).asInt()-1);
                    reply.addString("Successfully removed listener.");
                    success = true;
                } else if(cmd.get(1).asString() == "all") {
                    this->dispatcher->clear();
                    reply.addString("Successfully removed all listeners.");
                    success = true;
                } else {
                    throw std::runtime_error("Illegal index!");
                }
                break;
                }

            case VOCAB3('s','e','t'): // set
                { // prevent identifier initialization to cross borders of case
                Bottle property;
                property.addList() = cmd.tail().tail(); // see comment in TrainModule

                std::string replymsg = "Setting configuration option ";
                if(cmd.get(1).isInt() && cmd.get(1).asInt() >= 1 && cmd.get(1).asInt() <= this->dispatcher->countListeners()) {
                    if(this->dispatcher->getAt(cmd.get(1).asInt()-1).configure(property)) {
                        replymsg += "succeeded";
                    } else {
                        replymsg += "failed; please check key and value type.";
                    }
                    reply.addString(replymsg.c_str());
                    success = true;
                } else if(cmd.get(1).asString() == "all") {
                    for(int i = 0; i < this->dispatcher->countListeners(); i++) {
                        if(i > 0) {
                            replymsg += ", ";
                        }

                        if(this->dispatcher->getAt(i).configure(property)) {
                            replymsg += "succeeded";
                        } else {
                            replymsg += "failed";
                        }
                    }
                    replymsg += ".";
                    reply.addString(replymsg.c_str());
                    success = true;
                } else {
                    throw std::runtime_error("Illegal index!");
                }
                break;
                }

            case VOCAB4('i','n','f','o'): // information
            case VOCAB4('s','t','a','t'): // statistics
            case VOCAB4('l','i','s','t'): // list
                { // prevent identifier initialization to cross borders of case
                reply.add(Value::makeVocab("help"));
                std::ostringstream buffer;
                buffer << "Event Manager Information (" << this->dispatcher->countListeners() << " listeners)";
                reply.addString(buffer.str().c_str());
                for(int i = 0; i < this->dispatcher->countListeners(); i++) {
                    buffer.str(""); // why isn't there a proper reset method?
                    buffer << "  [" << (i + 1) << "] " << this->dispatcher->getAt(i).getInfo();
                    reply.addString(buffer.str().c_str());
                }
                
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
} // iCub

