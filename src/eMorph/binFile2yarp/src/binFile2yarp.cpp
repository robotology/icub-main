// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * Data buffer implementation for the event-based camera.
 *
 * Author: Charles Clercq, Giorgio Metta
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include "binFile2yarp.h"

using namespace std;
using namespace yarp::os;

binfile2yarp::binfile2yarp(string i_fileName) //: RateThread(10)
{
    len=0;
    memset(buffer, 0, SIZE_OF_DATA);

    raw = fopen(i_fileName.c_str(), "rb");
    if (raw != 0) {
        // obtain file size.
        fseek(raw, 0, SEEK_END);
        lSize = ftell(raw);
        rewind(raw);
        printf("file size: %d\n", lSize);
    }
    else {
        printf("can't open binary data file %s\n", i_fileName.c_str());
    }

    port.open("/DV128/out");
}

binfile2yarp::~binfile2yarp()
{
    if (raw)
        fclose(raw);
    port.close();
}

void  binfile2yarp::run()
{
    while (!isStopping()) {
        if (raw != 0) {
            int sz = 0;
            if(len < lSize)
            {
                int result = fread (&sz, sizeof(int), 1, raw);
                result = fread (buffer, 1, sz, raw);

                eventBuffer& tmp = port.prepare();
                tmp.set_data (buffer, sz);
                port.write();
                len += sz;

                int duration = getDuration(sz);
                //printf("duration: %d\n", duration);
                //fflush(stdout);
                // LATER: the timescale has to be checked.
                yarp::os::Time::delay(duration * 1e-6);
            }
            else
            {
                len = 0;
                sz = 0;
                rewind(raw);
            }
        }
        else
            yarp::os::Time::delay(5.0);
    }
}

int binfile2yarp::getDuration(int sz) {
    int index;
    int starttime = 0, endtime = 0;
    int wrapAdd = 0;
    bool first = true;

    for (index = 0; index < sz; index += 4) {

        if((buffer[index+3]&0x80) == 0x80) {
            wrapAdd += 0x4000;
        }
        else if ((buffer[index+3]&0x40) == 0x40) {
            wrapAdd = 0;
        }
        else {
            if (first) {
                starttime = (buffer[index+3]<<8) | buffer[index+2];
                starttime += wrapAdd;
                first = false;
            }
            else {
                endtime = (buffer[index+3]<<8) | buffer[index+2];
                endtime += wrapAdd;
            }
        }
    }

    return endtime - starttime;
}


