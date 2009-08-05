// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUB_TRACKERMONITOR_INC
#define ICUB_TRACKERMONITOR_INC

#include <yarp/os/Semaphore.h>
#include <yarp/sig/all.h>
#include <deque>

/**
 *
 * Monitor the effects of tracker commands.
 * The purpose is to enable compensation without a perfect head model.
 *
 */
class TrackerMonitor {
public:
    TrackerMonitor() : accessMutex(1) {
        haveZero = false;
        xSuggest = 0;
        ySuggest = 0;
        ox = 0;
        oy = 0;
        sx = 0.1;
        sy = 0.1;
        expTime = -1000;
        expState = 0;
        visualSuggest = false;
    }

    /**
     * Set the command which will produce zero motion, if that
     * command is known.
     */
    void setZero(double ox, double oy) {
        this->ox = ox;
        this->oy = oy;
        haveZero = true;
        xSuggest = ox;
        ySuggest = oy;
    }

    /**
     * Should periodically provide the current command output.
     */
    void setCommand(double cx, double cy, double cmdTime) {
        this->cx = cx;
        this->cy = cy;
        this->cmdTime = cmdTime;
    }


    /**
     * Can periodically provide the current motor state.
     */
    void setMotorState(const yarp::sig::Vector& v) {
        accessMutex.wait();
        state = v;
        accessMutex.post();
    }

    /**
     * Should periodically provide the current view and tracker location.
     */
    void apply(yarp::sig::ImageOf<yarp::sig::PixelRgb>& view,
               double x, double y, bool reset = false);

    double getMotorX() {
        return xSuggest;
    }
   
    double getMotorY() {
        return ySuggest;
    }

    bool getVisual(double& x, double& y) {
        if (visualSuggest) {
            x = xVisualSuggest;
            y = yVisualSuggest;
        }
        bool result = visualSuggest;
        visualSuggest = false;
        return result;
    }

private:
    bool haveZero;
    double ox, oy;
    double cx, cy, cmdTime;
    double sx, sy;
    double xSuggest, ySuggest;
    double xVisualSuggest, yVisualSuggest;
    bool visualSuggest;
    yarp::sig::Vector state;
    yarp::os::Semaphore accessMutex;
    double expTime;
    int expState;

    class TrackerRecord {
    public:
        double x, y;
        double ex, ey, emag;
        bool reset;
    };

    std::deque<TrackerRecord> history;
};


#endif

