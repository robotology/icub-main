// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Author: Giorgio Metta
 * Copyright (C) 2009 The RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/**
 * @file logpolarRemapper.cpp
 * @brief Test module of the logpolarGrabber using the ClientLogpolarGrabber class to connect
 * to a remote ServerLogpolarGrabber device driver.
 */

/**
 *
@ingroup icub_module
\defgroup icub_logpolarRemapper logpolarRemapper

This is a simple module that connects to the logpolar grabber (ServerLogpolarGrabber) and reconstruct 
the logpolar image to cartesian.

\section intro_sec Description
This is a test and simple module that connects to a ServerLogpolarGrabber, reads
the logpolar stream from the server, remaps it to cartesian using the ClientLogpolarGrabber
class (client side of the device). It uses the RFModule to implement the standard
module. It creates also ports for rpc commands (to the module) and rpc commands (to implement
the remote interfaces).

\section lib_sec Libraries
The logpolarRemapper depends on the iCubDev, icubmod and logPolar libraries.

\section parameters_sec Parameters
\code
 --width: the width of the reconstructed image. This need not be the original image size.
 --height: the height of the reconstructed image.
 --name: the name of the module. This is used to create port names (e.g. /name:rpc, /name:out).
 --remote: the name of the remote grabber ports to connect to (e.g. /grabber).
\endcode

\section portsa_sec Ports Accessed
If the remote grabber base name is "grabber" then logpolarRemapper connects to:
 - /grabber
 - /grabber/logpolar
 - /grabber/fovea

\section portsc_sec Ports Created
Input ports names are constructed from the "name" parameter, assuming this is set to "remapper", they are:
 - /remapper: stream of incoming cartesian images (not used by the current implementation).
 - /remapper/logpolar: stream of incoming logpolar images.
 - /remapper/fovea: stream of incoming foveal images (not used by the current implementation).
 - /remapper:rpc: the module port which receives the rpc module messages (e.g. quit).
    - [quit]: quit the module (exit).

Output ports names are also constructed from the "name" parameter, assuming this is set to "remapper", they are:
 - /remapper:out: stream of reconstructed images of size "width" and "height" (module parameters).

\section in_files_sec Input Data Files
No input data files.

\section out_data_sec Output Data Files
No output data files.
 
\section conf_file_sec Configuration Files
No configuration files. 

\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
logpolarRemapper --name remapper --remote /grabber --width 480

\author Giorgio Metta

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/logpolarRemapper/logpolarRemapper.cpp.

*/

/**
 * cond
 */

// default.
#include <stdio.h>
#include <memory.h>
#include <string>

// yarp.
#include <yarp/sig/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/dev/PolyDriver.h>

#include <iCub/drivers.h>
#include <iCub/LogpolarInterfaces.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

// the default app name (to be used in port names, etc.).
ConstString defaultname("remapper");
const int defaultSize = 480;

/*
 * my working thread.
 * this thread:
 * -starts the device drivers
 * -reads from the port, get images and remap them to cartesian (using the client grabber)
 * -writes to an output port the remapped images
 */
class Remapper : public Thread {
protected:
    PolyDriver poly;
    string remote;
    string local;
    string carrier;
    string outname;
    ILogpolarAPI *lpApi;
    ILogpolarFrameGrabberImage *lpImage;
    ImageOf<PixelRgb> lp;
    bool active;
    int width;
    int height;
    Port out;
    PortWriterBuffer<ImageOf<PixelRgb> > writer;

public:
    /*
     * Constructor.
     */
    Remapper() {
        active = false;
        width = height = defaultSize;
    }

    virtual ~Remapper() {}

    /*
     * configure the thread from the finder parameters.
     * @param rf is an instance of the ResourceFinder perhaps constructed from the command line.
     * @return true iff successful.
     */
    bool configure(const ResourceFinder& rf) {
        remote = ((ResourceFinder&)rf).find("remote").asString();

        // LATER: can use the rf.getName() here instead of searching again.
        ConstString name = ((ResourceFinder&)rf).find("name").asString();
        local = "/";
        local += ((name != "") ? name : defaultname);

        carrier = ((ResourceFinder&)rf).find("stream").asString();
        outname = "/";
        outname += ((name != "") ? name : defaultname);
        outname += ":out";

        width = ((ResourceFinder&)rf).find("width").asInt();
        height = ((ResourceFinder&)rf).find("height").asInt();

        if (width == 0 && height == 0) {
            width = height = defaultSize;
        }
        else
        if (width == 0 && height != 0) {
            width = height;
        }
        else 
        if (height == 0 && width != 0) {
            height = width;
        }

        writer.attach(out);
        return true;
    }

    /*
     * init the thread, instantiate the device driver and connet the ports appropriately.
     * @return true iff successful.
     */
    virtual bool threadInit() {
        Property p;
        p.put("device", "logpolarclient");
        p.put("local", local.c_str());
        p.put("remote", remote.c_str());
        poly.open(p);
        if (poly.isValid()) {
            poly.view(lpApi);
            poly.view(lpImage);
            active = false;

            if (lpApi != 0 && lpImage != 0) {
                const int nang = lpImage->nang();
                const int necc = lpImage->necc();
                const int fovea = lpImage->fovea();
                const double overlap = lpImage->overlap();

                if (necc != 0 && nang != 0) {
                    fprintf(stdout, "logpolar format with ang: %d ecc: %d fov: %d ovl: %f\n",
                        nang,
                        necc,
                        fovea,
                        overlap);

                    fprintf(stdout, "cartesian image of size %d %d\n", width, height);
                    lpApi->allocLookupTables(necc, nang, width, height, overlap);
                    active = true;

                    lp.resize(nang, necc);
                    
                    // open the out port.
                    out.open(outname.c_str());
                }
            }
        }
        return active;
    }

    /*
     * called when closing the thread to clean up resources.
     */
    virtual void threadRelease() {
        lpApi->freeLookupTables();
        poly.close();
        out.close();
        active = false;
    }

    /* 
     * main thread loop, sync'ed on the input port.
     */
    virtual void run() {
        while (!isStopping()) {
            if (active) {
                lpImage->getLogpolarImage(lp);
                ImageOf<PixelRgb>& datum = writer.get();
                datum.resize(width, height);

                // then remap
                lpApi->logpolarToCart(datum, lp);

                // then write to out port
                writer.write(true);
            }
        }
    }
};

/**
 * a logpolar remapper class/application implemented as a Yarp module.
 */
class logpolarRemapper : public RFModule, public Remapper
{
    Port handlerPort;  // a port to handle messages.

public:
    /**
     * Constructor.
     */
    logpolarRemapper() : Remapper(), RFModule() {
    }

    /**
     * Destructor.
     */
    virtual ~logpolarRemapper() {}

    
    /**
     * module periodicity (seconds), called implicitly by the module.
     * @return the periodicity in seconds.
     */
    double getPeriod() {
        return 1;
    }

    /**
     * This is our main function. Will be called periodically every getPeriod() seconds.
     * @return true if successful in the update.
     */
    bool updateModule() {
        return true;
    }
        
    /** 
     * Message handler. Just echo all received messages (this is the traditional respond).
     * @param command is the received command from the port (as a Bottle object).
     * @param reply is the reply to the command (another Bottle).
     * @return false causes the module to quit.
     */
    bool respond(const Bottle& command, Bottle& reply) {
        if (command.get(0).asString() == "quit")
            return false;     
        else
            reply=command; // here manages the messages (e.g. vocab style).
        return true;
    }

    /**
     * configure function. Wasn't this supposed to handle on the fly configuration?
     * @param rf is a reference to a ResourceFinder object that contains the init parameters.
     * @return true iff successful.
     */
    bool configure(const ResourceFinder& rf) {
        ConstString name = ((ResourceFinder&)rf).find("name").asString();
        if (name != "") {
            ConstString tmp = ConstString("/") + name  + ":rpc";
            handlerPort.open(tmp.c_str());
        }
        else {
            ConstString tmp = ConstString("/") + defaultname + ":rpc";
            handlerPort.open(tmp.c_str());
        }

        // LATER: call setName here (though not needed in this specific application).
        attach(handlerPort);

        // optional, attach to terminal if you want that text typed at the console
        // is redirected to the respond method
        attachTerminal();

        Remapper::configure(rf);
        Remapper::start();
        return true;
    }

    /** 
     * Interrupt function.
     * @return true always at the moment.
     */
    bool interruptModule() {
        return true;
    }

    /** 
     * Close function, to perform cleanup.
     * @return true is successful.
     */
    bool close() {
        Remapper::stop();
        handlerPort.close();
        return true;
    }

};


/*
 * A simple module that receives logpolar image and reconvert to cartesian
 * using the ClientLogpolarFrameGrabber driver. There's no robot name in 
 * this module.
 *
 * params:
 * --remote <string>
 * --name <string>
 * --width <int>
 * --height <int>
 *
 */
int main (int argc, char *argv[]) {
    // initialize yarp network.
    Network yarp;
    DriverCollection dev;
    fprintf(stdout, "%s\n", dev.status().c_str());

    // create your module.
    logpolarRemapper module; 

    // prepare and configure the resource finder.
    ResourceFinder rf;
    rf.setDefaultConfigFile("logpolarRemapper.ini");
    rf.setDefaultContext("logpolarRemapper/conf");
    rf.configure("ICUB_ROOT", argc, argv);
    rf.setVerbose(true);
 
    fprintf(stdout, "Configuring module\n");
    module.configure(rf);
    fprintf(stdout, "Starting module\n");
    module.runModule();

    fprintf(stdout, "Main returning\n");
    return 0;
}

/**
 * endcond
 */


