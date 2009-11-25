// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


/**
 * @ingroup icub_module
 *
 * \defgroup icub_template_tracker tracker
 *
 * A simple template-based visual tracker.
 * Run without options to see help on usage.
 *
 * The tracker module opens two ports, one for positions, one for images.
 * You need to send images to the image port input, or nothing will happen.
 * You can view the location the tracker is following by connecting
 * the image port out to a viewer.
 *
 *  \dot
 * digraph module_tracker_example {
 *     graph [ rankdir = "LR" ];
 *     edge [arrowhead="open", style="solid"];
 *     node [shape=ellipse];
 *     subgraph cluster_tracker {
 *      color = "black"; style = "solid";
 *      label = "tracker module";
 *       "/tracker/pos";
 *       "/tracker/img";
 *     }
 *     "/tracker/pos" -> "/motor/control"
 *     "/command" -> "/tracker/pos"
 *     "/camera" -> "/tracker/img"
 *     "/tracker/img" -> "/image/viewer"
 * \enddot
 *
 * You can connect from the position port to a reader in order to,
 * for example, control the motors based on the output.  The tracker
 * supports a number of output formats -- run the module without
 * arguments to see tips.
 *
 * You can send coordinates to the position port in order to override
 * where the tracker should be.
 * For testing, it can be useful to start a yarp viewer that sends
 * mouse clicks on the image to the tracker.  Start the viewer like 
 * this:
\verbatim
  yarpview --name /image/viewer --out /click
\endverbatim
 * The "/click" port can be connected to the tracker position input.
 * Mouse clicks should result in the tracker being reset to the
 * given location.
 *
 *  \dot
 * digraph module_tracker_example2 {
 *     graph [ rankdir = "LR" ];
 *     edge [arrowhead="open", style="solid"];
 *     node [shape=ellipse];
 *     subgraph cluster_tracker {
 *      color = "black"; style = "solid";
 *      label = "tracker module";
 *       "/tracker/pos";
 *       "/tracker/img";
 *     }
 *     subgraph cluster_viewer {
 *      color = "black"; style = "solid";
 *      label = "yarpview --out /click";
 *       "/image/viewer";
 *       "/click";
 *     }
 *     "/tracker/pos" -> "/motor/control"
 *     "/command" -> "/tracker/pos"
 *     "/camera" -> "/tracker/img"
 *     "/tracker/img" -> "/image/viewer"
 *     "/click" -> "/tracker/pos"
 * \enddot
 *
 *
 * For historic reasons, the tracker uses a hard-coded
 * image size (of 128x128).  It will work on arbitrary images however,
 * rescaling them for its internal use.  If you're optimizing, you
 * may want to modify the code to change the hard-coded size, but 
 * generally there's no need.
 * 
 * \see iCub::contrib::TemplateTracker
 *
 * \author Paul Fitzpatrick
 *
 */


#include <iostream>
#include <vector>
#include <math.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include "ImgTrack.h"
#include "TrackerMonitor.h"
#include "RetinalMap.h"

namespace iCub {
    namespace contrib {
        class TemplateTracker;
    }
}

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace iCub::contrib;


const char *DEFAULT_NAME = "/tracker";

#define VOCAB_GET VOCAB3('g','e','t')
#define VOCAB_SET VOCAB3('s','e','t')
#define VOCAB_POS VOCAB3('p','o','s')
#define VOCAB_IS VOCAB2('i','s')
#define VOCAB_W VOCAB1('w')
#define VOCAB_H VOCAB1('h')
#define VOCAB_HELP VOCAB4('h','e','l','p')


/**
 * A simple template-based visual tracker.
 *
 * \see icub_template_tracker
 */
class iCub::contrib::TemplateTracker : public Module,
            public TypedReaderCallback<Vector> {
private:
    RetinalMap retinalMap;
    Semaphore trackerAccess;
    BufferedPort<ColorImage> imgPort;
    BufferedPort<Vector> encPort;
    Port posPort;
    Terminee *pTerminee;
    bool verbose;
    YARPImageTrackTool tracker;
    TrackerMonitor monitor;
    //TrackerStat status;
    bool vvvFormat;
    bool calFormat;
    bool relFormat;
    bool pmONEFormat;
    bool icubFormat;
    bool shouldMonitor;
    bool isReset;
    PortWriterBuffer<Bottle> posPortOut;
    int w, h;
    double xCmd, yCmd;
    double cmdTime;
    int axes;
    int xlike;
    int ylike;
public:
    TemplateTracker() : trackerAccess(1) {
        pTerminee = NULL;
        verbose = false;
        vvvFormat = false;
        pmONEFormat = false;
        icubFormat = false;
        relFormat = false;
        calFormat = false;
        w = h = 0;
        shouldMonitor = false;
        isReset = true;
        cmdTime = -1000.0; // say command is old
        axes = -1;
        xlike = -1;
        ylike = -1;
    }

    virtual ~TemplateTracker() {
        if (pTerminee!=NULL) {
            delete pTerminee;
            pTerminee = NULL;
        }
    }

    /**
     * Execute the tracker, with the given configuration.
     * Call with an empty configuration to see what options
     * are available.
     */
    bool open(Searchable& config);

    bool close() {
        posPort.close();
        imgPort.close();
        encPort.close();
        return true;
    }

    bool interruptModule() {
        imgPort.interrupt();
        posPort.interrupt();
        encPort.interrupt();
        return true;
    }

    bool updateModule();

    bool respond(const Bottle& bot, Bottle& reply);

    /**
     * Called with encoder readings, if available.
     */
    void onRead(Vector& vect) {
        monitor.setMotorState(vect);
    }
};


bool TemplateTracker::open(Searchable& p) {
    posPortOut.attach(posPort);
    attach(posPort);
    //posPort.setReader(status);

    //BufferedPort<Vector> posPort;

    ConstString name = getName();
    vvvFormat = p.check("vvv");
    pmONEFormat = p.check("pmONE");
    relFormat = p.check("rel");
    calFormat = p.check("cal");
    icubFormat = p.check("icub");
    ConstString imageName = getName("img");
    ConstString posName = getName("pos");
    ConstString encName = getName("enc");
    imageName = p.check("image_name",
                        Value(imageName.c_str())).asString().c_str();
    posName = p.check("pos_name",
                      Value(posName.c_str())).asString().c_str();
    encName = p.check("enc_name",
                      Value(encName.c_str())).asString().c_str();

    if (p.check("camera")) {
        printf("Calibrated camera, normalizing output.\n");
        retinalMap.open(p.find("camera"));
    } else {
        printf("Calibrated camera not available.\n");
    }
    
    // send encoder readings, if available, to onRead
    encPort.useCallback(*this);

    imgPort.open(imageName.c_str());
    posPort.open(posName.c_str());
    encPort.open(encName.c_str());

	std::string quitName = name.c_str();
    quitName += "/quit";
    if (pTerminee!=NULL) {
        delete pTerminee;
        pTerminee = NULL;
    }
    pTerminee = new Terminee(quitName.c_str());
    if (pTerminee==NULL) {
        printf("Failed to allocate proper quit socket\n");
        return false;
    }
    if (!pTerminee->isOk()) {
        printf("Failed to create proper quit socket\n");
        return 1;
    }

	verbose = p.check("verbose");
    shouldMonitor = p.check("monitor");

    if (shouldMonitor) {
        axes = p.check("axes", Value(2), "number of axes").asInt();
    }

    return true;
}

bool TemplateTracker::updateModule() {
    if (pTerminee!=NULL) {
        if (pTerminee->mustQuit()) {
            return false;
        }
    }

    imgPort.read();
    if (imgPort.lastRead()==NULL) {
        return false;
    }

    if (shouldMonitor) {
        double xx, yy;
        if (monitor.getVisual(xx,yy)) {
            trackerAccess.wait();
            tracker.SetXYScaled((int)xx,(int)yy);
            isReset = true;
            trackerAccess.post();            
        }
    }

    ColorImage& in = *imgPort.lastRead();    
    w = in.width();
    h = in.height();
    trackerAccess.wait();
    tracker.Apply(in);
    retinalMap.setImage(w,h);
    // get the target.
    float x = tracker.GetX();
    float y = tracker.GetY();
    bool wasReset = isReset;
    isReset = false;
    trackerAccess.post();
    if (verbose) {
        printf("position %g %g\n", x, y);
    }
    cmdTime = Time::now();

    if (vvvFormat) {
        xCmd = x;
        yCmd = y;
        monitor.setZero(in.width()/2,in.height()/2);
    } else if (relFormat) {
        xCmd = x-in.width()/2;
        yCmd = y-in.height()/2;
        monitor.setZero(0,0);
    } else if (relFormat) {
        xCmd = x;
        yCmd = y;
    } else {
        xCmd = x-in.width()/2;
        yCmd = y-in.height()/2;
        monitor.setZero(0,0);
    }

    if (shouldMonitor) {
        monitor.setCommand(xCmd,yCmd,cmdTime);
        monitor.apply(in,(double)x,(double)y,
                      wasReset);
        xCmd = monitor.getMotorX();
        yCmd = monitor.getMotorY();
    }

    if (vvvFormat) {
        Bottle pos;
        pos.addDouble(1);
        pos.addDouble(xCmd);
        pos.addDouble(yCmd);
        pos.addDouble(in.width()/8);
        pos.addDouble(in.height()/8);
        pos.addDouble(in.width());
        pos.addDouble(in.height());
        posPortOut.prepare() = pos;
    } else if (relFormat) {
        Bottle pos;
        pos.addVocab(VOCAB3('s','e','t'));
        pos.addVocab(VOCAB4('r','e','l','s'));
        Bottle& coords = pos.addList();
        coords.addDouble(xCmd);
        coords.addDouble(yCmd);
        posPortOut.prepare() = pos;
        monitor.setZero(0,0);
    } else if (calFormat) {
        double xout, yout;
        Bottle pos;
        retinalMap.remap(x,y,xout,yout);
        pos.addDouble(xout);
        pos.addDouble(yout);
        pos.addDouble('r'); // this is very shady, talk to IST
        pos.addDouble('p'); // this is very shady, talk to IST
        pos.addDouble(0);   // control_gaze seems to look at this slot?
        pos.addDouble(0);   // control_gaze seems to look at this slot?
        posPortOut.prepare() = pos;
    } else if (icubFormat) {
        Bottle& output = posPortOut.prepare();
        double vx = 2*(((double)x)/((double)in.width()) - 0.5);
        double vy = 2*(((double)y)/((double)in.height()) - 0.5);
        vx *= 2;
        vy *= 2;
        output.clear();
        output.addVocab(Vocab::encode("set"));
        output.addVocab(Vocab::encode("rels"));
        Bottle poss;
        poss.addDouble(-vy);
        poss.addDouble(0);
        poss.addDouble(vx);
        poss.addDouble(-vy);
        poss.addDouble(vx);
        poss.addDouble(0);
        output.addList() = poss;
    } else if (pmONEFormat) { 
    //new format from july 2007 Mattia adds it
        Bottle pos;
        pos.addDouble(2*(((double)x)/((double)in.width()) - 0.5));
        pos.addDouble(2*(((double)y)/((double)in.height()) - 0.5));
        pos.addDouble(0.0);
        posPortOut.prepare() = pos;
	//printf("\n x=%lf",2*(((double)x)/((double)in.width()) - 0.5));
	//printf(" y=%lf",2*(((double)y)/((double)in.height()) - 0.5));
	}
    
    else {
    
        Bottle pos;
        pos.addDouble(x-in.width()/2);
        pos.addDouble(y-in.height()/2);
        posPortOut.prepare() = pos;
        monitor.setZero(0,0);
    }

    posPortOut.write();
    
    ColorImage& out = imgPort.prepare();
    out = in;
    addCircle(out,PixelRgb(255,0,0),(int)x,(int)y,10);
    for (int dx=-2; dx<=2; dx++) {
        for (int dy=-2; dy<=2; dy++) {
            addCrossHair(out,PixelRgb(255,255,255),dx+(int)x,dy+(int)y,20);
        }
    }
    imgPort.write();

    return true;
}


bool TemplateTracker::respond(const Bottle& bot, Bottle& reply) {
    if (!(bot.get(0).isVocab()||bot.get(0).isString())) {
        // direct command
        int x = 0;
        int y = 0;
        bool ok = false;
        if (bot.size()==2) {
            x = bot.get(0).asInt();
            y = bot.get(1).asInt();
            ok = true;
        } else if (bot.size()==7) {
            // summer school format
            double conf = bot.get(0).asDouble();
            if (conf>0.5) {
                double xx = bot.get(1).asDouble();
                double yy = bot.get(2).asDouble();
                double ww = bot.get(5).asDouble();
                double hh = bot.get(6).asDouble();
                if (ww>0.5) {
                    xx = w*xx/ww;
                }
                if (hh>0.5) {
                    yy = h*yy/hh;
                }
                x = (int)xx;
                y = (int)yy;
                ok = true;
            }
        }
        if (ok) {
            reply.addVocab(VOCAB_IS);
            reply.addVocab(VOCAB_POS);
            trackerAccess.wait();
            tracker.SetXYScaled(x,y);
            x = (int)tracker.GetX();
            y = (int)tracker.GetY();
            isReset = true;
            trackerAccess.post();
            reply.addInt(x);
            reply.addInt(y);
        } 
    } else {
        switch (bot.get(0).asVocab()) {
        case VOCAB_HELP:
            reply.addVocab(Vocab::encode("help"));
            reply.addString("[set] [pos]");
            reply.addString("[set] [pos] $x $y");
            reply.addString("$x $y");
            reply.addString("[get] [pos]");
            reply.addString("[get] [w]");
            reply.addString("[get] [h]");
            reply.addString("$confidence $x $y $targetWidth $targetHeight $imageWidth $imageHeight");
            break;
        case VOCAB_GET:
            {
                switch (bot.get(1).asVocab()) {
                case VOCAB_W:
                    reply.addVocab(VOCAB_IS);
                    reply.addVocab(VOCAB_W);
                    reply.addInt(w);
                    break;
                case VOCAB_H:
                    reply.addVocab(VOCAB_IS);
                    reply.addVocab(VOCAB_H);
                    reply.addInt(h);
                    break;
                case VOCAB_POS:
                    reply.addVocab(VOCAB_IS);
                    reply.addVocab(VOCAB_POS);
                    trackerAccess.wait();
                    int x = (int)tracker.GetX();
                    int y = (int)tracker.GetY();
                    trackerAccess.post();
                    reply.addInt(x);
                    reply.addInt(y);
                    break;
                }
            }
            break;
        case VOCAB_SET:
            {
                switch (bot.get(1).asVocab()) {
                case VOCAB_POS:
                    int x = bot.get(2).asInt();
                    int y = bot.get(3).asInt();
                    reply.addVocab(VOCAB_IS);
                    reply.addVocab(VOCAB_POS);
                    trackerAccess.wait();
                    if (!(bot.get(2).isInt()&&bot.get(2).isInt())) {
                        tracker.ResetXY();
                    } else {
                        tracker.SetXYScaled(x,y);
                    }
                    x = (int)tracker.GetX();
                    y = (int)tracker.GetY();
                    isReset = true;
                    trackerAccess.post();
                    reply.addInt(x);
                    reply.addInt(y);
                    break;
                }
            }
            break;
        }
    }
    if (reply.size()==0) {
        return false;
    }
    return true;
}


int main(int argc, char *argv[]) {
  
    if (argc<=1) {
        printf("default port names are:\n");
        printf("    /tracker/img (for incoming and outgoing images)\n");
        printf("    /tracker/pos (for streaming position vector output)\n");
        printf("can change \"/tracker\" prefix with --name option\n");
        printf("can change port names individually with --image_name, --pos_name, --enc_name\n");
        printf("use --vvv for \"summer school\" format\n");
        printf("use --rel for \"[set] [rels] ($x $y)\" format\n");
        printf("use --cal for \"calibrated\" format\n");
        printf("use --icub for \"direct-to-iCubInterface\" format\n");
        printf("use --pmONE for \"+1/-1\" format\n");
        return 1;
    }

    Network yarp;

    TemplateTracker tracker;
    tracker.setName(DEFAULT_NAME);
    return tracker.runModule(argc,argv);
}
