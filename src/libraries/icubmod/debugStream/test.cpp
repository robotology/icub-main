/*
 * Copyright (C) 2014  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <vector>

#include <yarp/os/Log.h>

#define WITH_LOGSTREAM

#ifdef WITH_LOGSTREAM
#include <yarp/os/LogStream.h>
#endif



int main(int argc, char *argv[])
{
    int i = 13;
#ifdef WITH_LOGSTREAM
    std::vector<int> v(4);
    v[0] = 1;
    v[1] = 2;
    v[2] = 3;
    v[3] = 4;
#endif
    yTrace("This is a trace");
    yTrace("This is %s (%d)", "a trace", i);
#ifdef WITH_LOGSTREAM
    yTrace();
    yTrace() << "This is" << "another" << "trace" << i;
    yTrace() << v;
#endif

    yDebug("This is a debug");
    yDebug("This is %s (%d)", "a debug", i);
#ifdef WITH_LOGSTREAM
    yDebug();
    yDebug() << "This is" << "another" << "debug" << i;
    yDebug() << v;
#endif

    yInfo("This is info");
    yInfo("This is %s (%d)", "info", i);
#ifdef WITH_LOGSTREAM
    yInfo();
    yInfo() << "This is" << "more" << "info" << i;
    yInfo() << v;
#endif

    yWarning("This is a warning");
    yWarning("This is %s (%d)", "a warning", i);
#ifdef WITH_LOGSTREAM
    yWarning();
    yWarning() << "This is" << "another" << "warning" << i;
    yWarning() << v;
#endif
    yError("This is an error");
    yError("This is %s (%d)", "an error", i);
#ifdef WITH_LOGSTREAM
    yError();
    yError() << "This is" << "another" << "error" << i;
    yError() << v;
#endif


#ifdef WITH_LOGSTREAM
    yFatal() << "This is the end.";
#else
    yFatal("This is the end.");
#endif

}
