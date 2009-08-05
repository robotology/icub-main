// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <stdio.h>
#include <stdlib.h>

#include <vector>
#include <string>

#include "shared.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

double niceTime = -10000;
double niceTime2 = -10000;
double stableTime = -10000;
double stableTime2 = -10000;


#define LOG_SIZE 200
#define COMP_SIZE 5

class Hist {
private:
    ImageOf<PixelFloat> log[LOG_SIZE];
    double times[LOG_SIZE];
    int comps[COMP_SIZE];
    int index;
    string name;
    bool first;
    int nom;
public:
    Hist() {
        index = 0;
        first = true;
        nom = 0;
    }

    void reset() {
        index = 0;
        first = true;
    }

    void setNom(int nom) {
        this->nom = nom;
    }

    void setName(const char *name) {
        this->name = name;
    }

    void applyAll(ImageOf<PixelFloat>& img, double t) { 
        for (int i=0; i<LOG_SIZE; i++) {
            log[i] = img;
            times[i] = t;
        }
    }


    void apply(ImageOf<PixelFloat>& img, double t) {
        if (first) {
            applyAll(img,t);
            first = false;
        }
        log[index] = img;
        times[index] = t;
        comps[0] = index;
        double dt = searchState.period;
        //printf(">>> ");
        for (int i=1; i<COMP_SIZE; i++) {
            int at = index;
            for (int j=1; j<LOG_SIZE; j++) {	
                at = (index+LOG_SIZE-j)%LOG_SIZE;
                if (times[at]<t-dt*i) {
                    //printf("%g ", times[at]-t);
                    break;
                }
            }
            comps[i] = at;
        }
        //printf("\n");
        ImageOf<PixelFloat> sum;
        ImageOf<PixelFloat> mean;
        sum.resize(img);
        mean.resize(img);
        sum.zero();
        mean.zero();
        for (int i=0; (i+1)<COMP_SIZE; i++) {
            IMGFOR(sum,x,y) {
                float diff = log[comps[i]](x,y)-log[comps[i+1]](x,y);
                diff *= (i%2)?1:-1;
                sum(x,y) += diff;
                mean(x,y) += log[comps[i]](x,y);
            }
        }
        int total = 0;
        int ct = 0;
        double val = 0;
        double gx = 0;
        double gy = 0;
        double gx2 = 0;
        double gy2 = 0;
        IMGFOR(sum,x,y) {
            double v = sum(x,y);
            double m = mean(x,y);
            if (m<10) m = 10;
            v = fabs(v)/m;
            sum(x,y) = v;
            if (v>1.3) {
                val++;
                ct++;
                gx += x;
                gy += y;
                gx2 += x*x;
                gy2 += y*y;
            }
        }
        double xvar = 1e32;
        double yvar = 1e32;
        if (ct>0) {
            gx /= ct;
            gy /= ct;
            gx2 /= ct;
            gy2 /= ct;
            xvar = sqrt(fabs(gx*gx-gx2));
            yvar = sqrt(fabs(gy*gy-gy2));
        }
        if (val>50) {
            double var = xvar;
            if (yvar>var) { var=yvar; }
            bool ok = (var<50);
            printf("[%s %g %g %g %d]\n", name.c_str(), val, searchState.period,
                   var, ok);
            if (ok) {
                if (nom == 1) {
                    if (searchState.currentTime-niceTime>2) {
                        niceTime = searchState.currentTime;
                    }
                } else {
                    if (searchState.currentTime-niceTime2>2) {
                        niceTime2 = searchState.currentTime;
                    }
                }
            }
        }
        index = (index+1)%LOG_SIZE;
    }
};


class Watcher {
private:
    Hist logger;
    double lastDisturbance;
    double lastRipple;
    double lastPlacid;
    ImageOf<PixelFloat> prev, work, diff;
    int nom;
    int active;
    double lastReset;
public:
    BufferedPort<ImageOf<PixelRgb> > port;

    void down() {
        active = 0;
        port.interrupt();
        lastReset = -10000;
    }

    void init(yarp::os::Property& conf) {
        active = 1;

        printf("watcher\n");

        string local = conf.check("local",
                                  Value("/pf/watcher")).asString().c_str();
        string remote = conf.check("remote",
                                   Value("/icubSim/cam/left")).asString().c_str();

        nom = conf.check("nom",Value(0)).asInt();
        logger.setNom(nom);
        logger.setName(remote.c_str());

        port.open(local.c_str());
        Network::connect(remote.c_str(),port.getName().c_str());

        lastDisturbance = -1000;
        lastRipple = -1000;
        lastPlacid = -1000;
    }

    void process() {
        if (searchState.currentTime-searchState.globalLastAct<2 &&
            searchState.currentTime-lastReset>5) {
            logger.reset();
            lastReset = searchState.currentTime;
            printf("\n************ RESET WATCH *****************\n");
        }

        if (!active) return;
        ImageOf<PixelRgb> *img = port.read();
        if (img==NULL) return;
        if (!active) return;
        if (prev.width()==0) {
            prev.copy(*img);
        }
        work.copy(*img);
        logger.apply(work,Time::now());
        diff.resize(work);
        float total = 0;
        int ct = 0;
        IMGFOR(work,x,y) {
            float del = fabs(work(x,y)-prev(x,y));
            if (del>2) del = 255;
            total += del;
            ct++;
        }
        if (ct>0) total /= ct;
        bool stable = false;
        bool unstable = false;
        if (total>20) {
            unstable = true;
            lastDisturbance = searchState.currentTime;
        }
        if (total<7) {
            stable = true;
            //printf("STABLE total %04.5f\n", total);
        } else {
            lastRipple = searchState.currentTime;
        }
        if (searchState.currentTime-lastRipple>1.0) {
            lastPlacid = searchState.currentTime;
        }
        bool learnable = false;
        bool armMove = false;
        bool headMove = true;
        /*
        if (currentTime-movingHeadTime>3) {
            headMove = false;
            if (movingArmTime>movingHeadTime && currentTime-movingArmTime<3) {
                armMove = true;
                if (lastPlacid>movingHeadTime && lastPlacid>1) {
                    learnable = true;
                }
            }
        }
        bool placid = (lastPlacid>movingHeadTime);
        */
        if (searchState.currentTime-searchState.globalLastAct>3) {
            headMove = false;
            armMove = true;
            if (lastPlacid>searchState.globalLastAct && lastPlacid>1) {
                learnable = true;
            }
        }
        bool placid = (lastPlacid>searchState.globalLastAct);
        if (placid) {
            if (nom==1) {
                stableTime = searchState.currentTime;
            } else {
                stableTime2 = searchState.currentTime;
            }
            //printf("stable on %s\n", port.getName().c_str());
        }

        if (searchState.currentTime-stableTime<1 && 
            searchState.currentTime-stableTime2<1) {
            if (searchState.currentTime-searchState.stableSummaryTime>2) {
                searchState.stableSummaryTime = searchState.currentTime;
            }
        }
        if (searchState.currentTime-niceTime<3 && 
            searchState.currentTime-niceTime2<3) {
            if (searchState.currentTime-searchState.niceSummaryTime>4) {
                searchState.niceSummaryTime = searchState.currentTime;
            }
        }

        prev.copy(*img);
    }
};

static int watcher_active = 1;

static Watcher w1, w2;

bool watcher_interrupt() {
    watcher_active = 0;
    w1.down();
    w2.down();
    return true;
}

int watcher_main(yarp::os::Property& conf) {

    conf.put("local","/pf/watch/left");
    conf.put("remote","/icubSim/cam/left");
    conf.put("nom",1);
    w1.init(conf);

    conf.put("local","/pf/watch/right");
    conf.put("remote","/icubSim/cam/right");
    conf.put("nom",2);
    w2.init(conf);

    while (watcher_active) {
        w1.process();
        w2.process();
    }
    printf("WATCHER STOPPED\n");

    return 0;
}
