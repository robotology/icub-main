// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 * @file saliencyBlobFinderModule.cpp
 * @brief module class implementation of the blob finder module.
 */

#include <iCub/saliencyBlobFinderModule.h>

using namespace std;

saliencyBlobFinderModule::saliencyBlobFinderModule() {
    //
    blobFinder = 0;
    threadRate = 100;
    minBoundingArea = 225;
    reply=new Bottle();
}

bool saliencyBlobFinderModule::configure(ResourceFinder &rf) {
    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/blobFinder/icub/left_cam"), 
                           "module name (string)").asString();

    /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());

    /*
    * get the thread rate which will define the period of the processing thread
    */
    threadRate = rf.check("ratethread", Value(33), "processing ratethread (int)").asInt();
    cout << "Module started with the parameter ratethread: " << threadRate << endl;

    /*
    * gets the minBounding area for blob neighbours definition
    */
    minBoundingArea = rf.check("minBoundingArea", Value(225), "minBoundingArea (int)").asInt();

    if (!cmdPort.open(getName())) {
        cout << getName() << ": Unable to open port " << endl;
        return false;
    }

    attach(cmdPort);                  // attach to port

    //initialization of the main thread
    blobFinder = new blobFinderThread(threadRate);
    blobFinder->setName(this->getName().c_str());
    blobFinder->setMinBoundingArea(minBoundingArea);
    blobFinder->start();

    cout << "waiting for connection of the input port" << endl;
    return true;
}

/** 
* tries to interrupt any communications or resource usage
*/
bool saliencyBlobFinderModule::interruptModule() {
    // LATER: check this code for proper quit/release/exit.
    cout << "module interrupted" << endl;
    cmdPort.interrupt();
    blobFinder->interrupt();
    return true;
}

bool saliencyBlobFinderModule::close() {
    cout << "closing command port" << endl;
    cmdPort.close();
    blobFinder->stop();
    return true;
}

bool saliencyBlobFinderModule::updateModule() {
    return true;
}

bool saliencyBlobFinderModule::respond(const Bottle &command, Bottle &reply) {
    // 
    bool ok = false;
    bool rec = false; // is the command recognized?

    mutex.wait();
    
    switch (command.get(0).asVocab()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
        {
            reply.addString("help");
            reply.addString("\n");
            reply.addString("get fn \t: general get command \n");
            reply.addString("\n");
            reply.addString("set s1 <s> \t: general set command \n");
            reply.addString("\n");
            reply.addString("NOTE: capitalization of command name is mandatory\n");
            reply.addString("set Mdb : set maximum dimension allowed for blobs\n");
            reply.addString("set mdb : set minimum dimension allowed for blobs\n");
            reply.addString("set mBA : set the minimum bounding area\n");
            reply.addString("\n");
            reply.addString("get Mdb : get maximum dimension allowed for blobs\n");
            reply.addString("get mdb : get minimum dimension allowed for blobs\n");
            reply.addString("get mBA : get the minimum bounding area\n");
            ok = true;
        }
        break;

    case COMMAND_VOCAB_SET:
        rec = true;
        {
            switch(command.get(1).asVocab()) {
            case COMMAND_VOCAB_MBA:
                {
                    double w = command.get(2).asDouble();
                    cout << "set mBA: " << w << endl;
                    if (0 != blobFinder)
                        blobFinder->setMinBoundingArea(int(w+.5));
                    ok=true;
                }
                break;
            case COMMAND_VOCAB_MAXDB:
                {
                    int w = command.get(2).asInt();
                    if (0 != blobFinder)
                        blobFinder->setMaxBlobSize(w);
                    ok=true;
                }
                break;
            case COMMAND_VOCAB_MINDB:
                {
                    int w = command.get(2).asInt();
                    if(0 != blobFinder)
                        blobFinder->setMinBlobSize(w);
                    ok=true;
                }
                break;
            default:
                cout << "received an unknown request after a SALIENCE_VOCAB_SET" << endl;
                break;
            }
        }
        break;

    case COMMAND_VOCAB_GET:
        rec = true;
        {
            reply.addVocab(COMMAND_VOCAB_IS);
            reply.add(command.get(1));
            switch(command.get(1).asVocab()) {

            case COMMAND_VOCAB_MAXDB:
                {
                    int nb = blobFinder->getMaxBlobSize();
                    reply.addInt(nb);
                    ok = true;
                }
            break;
            case COMMAND_VOCAB_MINDB:
                {
                    int nb = blobFinder->getMinBlobSize();
                    reply.addInt(nb);
                    ok = true;
                }
            break;
            /* LATER: implement case COMMAND_VOCAB_MBA */

            default:
                cout << "received an unknown request after a SALIENCE_VOCAB_GET" << endl;
                break;
            }
        }
        break;

    }
    mutex.post();

    if (!rec)
        ok = RFModule::respond(command,reply);
    
    if (!ok) {
        reply.clear();
        reply.addVocab(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab(COMMAND_VOCAB_OK);

    return ok;
} 	


