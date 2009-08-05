// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "TrackerMonitor.h"
#include <stdio.h>
#include <yarp/os/Time.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

static double mag(double x, double y) {
    return sqrt(x*x+y*y);
}

void TrackerMonitor::apply(ImageOf<PixelRgb>& view, 
                           double x, double y, bool reset) {
    double vx = view.width()/2;
    double vy = view.height()/2;
    x -= vx;
    y -= vy;

    accessMutex.wait();

    double ex = cx - ox;
    double ey = cy - oy;

    double exMax = view.width()/4;
    double eyMax = view.height()/4;
    double emagMax = sqrt(exMax*exMax + eyMax*eyMax);

    double exMin = view.width()/16;
    double eyMin = view.height()/16;
    double emagMin = sqrt(exMin*exMin + eyMin*eyMin);

#if 0
    printf("tracking (%g,%g) effort (%g,%g) scale (%g,%g) enc %s %s\n",
           x, y,
           ex, ey,
           sx, sy,
           state.toString().c_str(),
           reset?"reset":"");
#endif
    
    TrackerRecord rec;
    rec.x = x;
    rec.y = y;
    rec.ex = ex;
    rec.ey = ey;
    rec.emag = sqrt(ex*ex+ey*ey);
    rec.reset = reset;

    accessMutex.post();


    xSuggest = sx*ex+ox;
    ySuggest = sy*ey+oy;


    double now = Time::now();
    static int uncomfortable = 0;
    static double tx = 0, ty = 0;
    static bool running = false;
    static double prevTime = 0;
    static int goodCt = 0;
    static int sgnFlipper = 0;
    static int abortCt = 0;

    if (running) {
        double tmag = mag(tx,ty);
        double cmag = mag(x,y);
        if (cmag>tmag*1.1 || uncomfortable) {
            printf("ABORT experiment, diverged\n");
            xVisualSuggest = vx;
            yVisualSuggest = vy;
            visualSuggest = true;
            running = false;
            abortCt++;
            if (abortCt>=4) {
                sgnFlipper++;
                if (sgnFlipper%2==0) {
                    sy *= -1;
                } else {
                    sx *= -1;
                }
                printf("Flipped scales to %g %g\n",
                       sx, sy);
                abortCt = 0;
            }
        }

        if (cmag<tmag/2) {
            abortCt = 0;
            double dt = now-expTime;
            printf("Reached halfway in %g seconds\n", 
                   dt);
            if (dt>1.5) {
                goodCt = 0;
                if (prevTime/dt>0.8 && prevTime/dt<1.2) {
                    sx *= 1.1;
                    sy *= 1.1;
                    printf("Seems robust - scale %g %g\n",
                           sx, sy);
                }
            } else {
                goodCt++;
            }
            prevTime = dt;
            running = false;
        }

        if (now-expTime>4) {
            abortCt = 0;
            printf("Slow\n");
            printf("Went from (%g %g) to (%g %g)\n",
                   tx, ty, x, y);
            sx *= 1.1;
            sy *= 1.1;
            printf("scale %g %g\n",
                   sx, sy);
            running = false;
        }
    }



    if (now-expTime>5 && goodCt<5) {
        printf("Making suggestion\n");
        expTime = now;
        expState++;
        int sgn1 = (expState%2)?1:-1;
        int sgn2 = ((expState/2)%2)?1:-1;
        tx = sgn1*vx/2;
        ty = sgn2*vy/2;
        xVisualSuggest = vx+tx;
        yVisualSuggest = vy+ty;
        visualSuggest = true;
        running = true;
    }

    if (now-expTime>100) {
        goodCt = 0;
    }


    /*
     *
     * Apply some very basic tests to see if behavior of tracker is
     * plausible or crazy.
     *
     */
    uncomfortable = 0;
    history.push_back(rec);
    if (history.size()>10) {
        history.pop_front();

        int ct = (int)history.size();
        int ups = 0;
        int downs = 0;
        int wides = 0;
        double emag = 0;
        for (deque<TrackerRecord>::iterator it=history.begin(); 
             it!=history.end(); it++) {
            if (it->emag>emag+0.0001) {
                if (it->emag>emagMin) {
                    ups++;
                }
            }
            if (it->emag<emag-0.0001) {
                downs++;
            }
            if (it->emag>emagMax) {
                wides++;
            }
            emag = it->emag;
        }
        bool ok = true;
        if (ups>ct/2 && downs==0) { ok = false; }
        if (wides>ct*0.9) {
            if (downs<ct/2) {
                ok = false;
            }
        }
        if (!ok) {
            uncomfortable = 1;
            printf("*** uncomfortable: ups %d downs %d wides %d ***\n",
                   ups,
                   downs,
                   wides);
            xSuggest = ox;
            ySuggest = oy;
        }
    }
}

