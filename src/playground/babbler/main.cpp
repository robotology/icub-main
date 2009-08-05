// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>

#include <stdio.h>
#include <stdlib.h>

#include <vector>
#include <string>

#include "shared.h"

#include "Range.h"
#include "Evolver.h"
#include "Pool.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

SearchState searchState;


#define GLOBAL_PERIOD 20

class Reporter {
public:
    virtual void reportStatus(Bottle& s) = 0;
};

class Driver : public Range {
public:
    int n;
    int lastWave;
    double first;
    double lastAct;
    double rewardTime;
    double stableRewardTime;
    //int group;
    string alt;
    Bottle override;
    Reporter *reporter;
    Property options;
    PolyDriver dev;
    IPositionControl *pos;
    IEncoders *enc;
    IControlLimits *limits;
    Pool pool;
    Vector mins, maxs, ranges;
    double waveTime;
    int exploreMode;

    Driver() {
        exploreMode = 1;
        alt = "left_arm";
        rewardTime = -1000;
        lastWave = 0;
        reporter = NULL;
        waveTime = 0;
    }

    void setWave(double waveTime) {
        this->waveTime = waveTime;
    }

    void setReporter(Reporter *reporter = NULL) {
        this->reporter = reporter;
    }

    virtual double getMin(int i) {
        return mins[i];
    }

    virtual double getMax(int i) {
        return maxs[i];
    }

    //void setGroup(int group) {
    //this->group = group;
    //}


    bool open(Searchable& ref) {
        printf("Status %s\n", ref.toString().c_str());
        if (ref.check("wave")) {
            setWave(ref.check("wave",Value(1)).asDouble());
            searchState.period = waveTime;
        }
        exploreMode = ref.check("explore",Value(1)).asInt();
        if (ref.check("override")) {
            override = ref.findGroup("override").tail();
        }


        first = Time::now();
        lastAct = -1000;
        options.put("device","remote_controlboard");
        string port = ref.check("port",Value("notset")).asString().c_str();
        alt = port;
        options.put("remote",port.c_str());
        options.put("local",(string("/handy") + alt.c_str()).c_str());
        options.put("carrier","tcp");

        dev.open(options);

        if (!dev.isValid()) {
            printf("Failed to connect to device\n");
            exit(1);
        }
  
        dev.view(pos);
        if (pos==NULL) {
            printf("Failed to get control interface\n");
            exit(1);
        }

        dev.view(enc);
        if (enc==NULL) {
            printf("Failed to get encoder interface\n");
            exit(1);
        }

        pos->getAxes(&n);
        maxs = Vector(n);
        mins = Vector(n);
        ranges = Vector(n);

        for (int i=0; i<n; i++) {
            pos->setRefSpeed(i,5);
        }

        dev.view(limits);
        if (limits==NULL) {
            printf("Failed to get limits interface, will just guess\n");
            for (int i=0; i<n; i++) {
                maxs[i] = 70;
                mins[i] = -70;
            }
        } else {
            for (int i=0; i<n; i++) {
                bool ok = limits->getLimits(i,&(mins[i]),&(maxs[i]));
                printf("%d joint got limit %g %g (%d)\n", i, mins[i], maxs[i], ok);
                if (!ok) {
                    maxs[i] = 70;
                    mins[i] = -70;
                }
            }
        }
        for (int i=0; i<n; i++) {
            ranges[i] = maxs[i]-mins[i];
        }
        for (int i=0; i<override.size(); i++) {
            double v = override.get(i).asDouble();
            maxs[i] = v+0.001;
            mins[i] = v;
        }

        pool.setRange(this);
        pool.hold(n);

        return (n>0);
    }


    void wander() {
        double now = Time::now();
        now -= first;
        searchState.currentTime = now;
        //if (currentTime-stableTime<1 && currentTime-stableTime2<1) {
        if (searchState.currentTime-searchState.stableSummaryTime<1) {
            if (searchState.currentTime-stableRewardTime>8) {
                if (waveTime<0.01) {
                    printf("Reward for stability %s\n", alt.c_str());
                    pool.reward(0.05);
                    stableRewardTime = searchState.currentTime;
                }
            }
        }
        if (searchState.currentTime-searchState.niceSummaryTime<1) {
            if (searchState.currentTime-rewardTime>8) {
                printf("Mover reward for %s\n", alt.c_str());
                if (waveTime>0.01) {
                    pool.reward(0.25);
                } else {
                    pool.reward(0.1);
                }
                rewardTime = searchState.currentTime;
            }
            printf("*** RECORDING possibly correlated situation %s\n", alt.c_str());
            Bottle bot;
            bot.clear();
            bot.addString(alt.c_str());
            bot.addDouble(searchState.niceSummaryTime);
            bot.addDouble(searchState.currentTime);
            Bottle& sub = bot.addList();
            for (int i=0; i<n; i++) {
                double v = 0;
                enc->getEncoder(i,&v);
                sub.addDouble(v);
            }
            if (reporter!=NULL) {
                reporter->reportStatus(bot);
            }
        }

        if (exploreMode) {
            if (now-lastAct>GLOBAL_PERIOD || 
                (now-lastAct>GLOBAL_PERIOD*0.75 && 
                 now-searchState.globalLastAct<GLOBAL_PERIOD*0.2)) {
                rewardTime = -1000;
                searchState.niceSummaryTime = -1000;
                searchState.globalLastAct = now;
                lastAct = now;
                printf("Go %s ", alt.c_str());
                pool.next();
                for (int i=0; i<n; i++) {
                    double v = pool.get(i);
                    //Random::uniform()*140-70;
                    pos->positionMove(i,v);
                    printf("%d:%g ", i, v);
                }
                printf("\n");
                lastWave = 0;
            }
            if (waveTime>0.01) {
                if (searchState.currentTime-lastAct>2) {
                    int wave = 2*(int(searchState.currentTime/waveTime)%2) - 1;
                    if (wave!=lastWave) {
                        printf("Wave %d\n", wave);
                        lastWave = wave;
                        for (int i=0; i<n; i++) {
                            double v = pool.get(i);
                            pos->positionMove(i,v+wave*(ranges[i]/24));
                        }
                    }
                }
            }
        }
    }

    void run() {
        while (true) {
            wander();
            Time::delay(0.25);
        }
    }

    void fini() {
        printf("Stopping device (%s)\n", alt.c_str());
        dev.close();
        printf("Device stopped\n");
    }
};


class Watcher : public Thread {
public:
    Property options;
    virtual bool interrupt() {
        watcher_interrupt();
        return true;
    }
    virtual void run() {
        watcher_main(options);
    }
} watcher;


class Aligner : public Thread {
public:
    Property options;
    virtual bool interrupt() {
        aligner_interrupt();
        return true;
    }
    virtual void run() {
        aligner_main(options);
    }
} aligner;


class BabblerModule : public Module, public Reporter {
private:
    BufferedPort<Bottle> status;
    vector<Driver> drivers;
    int ndrv;
public:
    virtual bool open(Searchable& config) {
        if (!config.check("file")) {
            printf("Please supply a \"--file config.ini\" argument\n");
            return false;
        }

        status.setStrict(true);
        status.open(getName("status"));

        watcher.options.fromString(config.toString());
        watcher.start();
        aligner.options.fromString(config.toString());
        aligner.start();

        Bottle names = config.findGroup("part").tail();
        printf("names are %s\n", names.toString().c_str());

        ndrv = names.size();
        drivers = vector<Driver>(ndrv);
        for (int i=0; i<ndrv; i++) {
            Driver& drv = drivers[i];
            drv.setReporter(this);
            //drv.setGroup((i==0)?GROUP_HEAD:GROUP_ARM);
            ConstString name = names.get(i).asString();
            Bottle ref = config.findGroup(name.c_str());
            printf("Setting for %s is %s\n", name.c_str(),
                   ref.toString().c_str());
            drv.open(ref);
        }

        return true;
    }

    virtual bool updateModule() {
        for (int i=0; i<ndrv; i++) {
            drivers[i].wander();
        }
        Time::delay(0.25);
        return true;
    }

    virtual bool interruptModule() {
        printf("interrupting\n");
        status.interrupt();
        watcher.interrupt();
        aligner.interrupt();
        for (int i=0; i<ndrv; i++) {
            drivers[i].fini();
        }
        watcher.stop();
        aligner.stop();
        printf("interrupted\n");
        return true;
    }

    virtual bool close() {
        printf("closing\n");
        status.close();
        printf("closed\n");
        return true;
    }


    virtual void reportStatus(Bottle& s) {
        status.prepare() = s;
        status.write(true);
    }

};


int main(int argc, char *argv[]) {
    Network yarp;

    BabblerModule module;
    module.setName("/babbler");
    return module.runModule(argc,argv);

    //Network::connect("/icubSim/cam/left","/sim/view");
}
