#include "stereoCalibModule.h"
#include <yarp/os/Stamp.h>
#include <yarp/os/Os.h>



bool stereoCalibModule::configure(yarp::os::ResourceFinder &rf)
{
    moduleName=rf.check("name",Value("stereoCalib"),"module name (string)").asString().c_str();
    setName(moduleName.c_str());

    handlerPortName="/";
    handlerPortName+=getName(rf.check("CommandPort",Value("/cmd"),"Output image port (string)").asString());

    char dirName[255];
    bool proceed=true;
    string dir=rf.getHomeContextPath().c_str();

    for (int i=1; proceed; i++)
    {
        sprintf(dirName,"%s/%s_%.5d",dir.c_str(),"calibImg",i);
        proceed=!yarp::os::stat(dirName);
        sprintf(dirName,"%s/%s_%.5d/",dir.c_str(),"calibImg",i);
    }    
    yarp::os::mkdir_p(dirName);

    if (!handlerPort.open(handlerPortName.c_str()))
    {
        cout << ": unable to open port " << handlerPortName << endl;
        return false;
    }
    attach(handlerPort);

    calibThread=new stereoCalibThread(rf,&handlerPort,dirName);
    calibThread->start();

    return true;
}


bool stereoCalibModule::interruptModule()
{
    calibThread->stopCalib();
    calibThread->stop();
    return true;
}


bool stereoCalibModule::close()
{
    calibThread->stop();
    delete calibThread;

    return true;
}


bool stereoCalibModule::respond(const Bottle& command, Bottle& reply) 
{
    reply.clear();

    if(command.size() == 0)
    {
        reply.addString("error");
        reply.addString("empty command");
        return true;
    }

    const std::string cmd = command.get(0).asString();

    if (cmd == "start") {
        calibThread->startCalib();
        reply.addString("ok");
        reply.addString("calibration collection started");
    }
    else if(cmd == "stop")
    {
        calibThread->stopCalib();
        reply.addString("ok");
        reply.addString("calibration collection stopped");
    }
    else if(cmd == "status")
    {
        StereoCalibStatus status = calibThread->getStatus();
        reply.addString("ok");
        reply.addString("state");
        reply.addString(status.state);
        reply.addString("pairedFrames");
        reply.addInt64(status.pairedFrames);
        reply.addString("droppedLeftFrames");
        reply.addInt64(status.droppedLeftFrames);
        reply.addString("droppedRightFrames");
        reply.addInt64(status.droppedRightFrames);
        reply.addString("meanTimestampDeltaMs");
        reply.addFloat64(status.meanTimestampDeltaMs);
        reply.addString("maxTimestampDeltaMs");
        reply.addFloat64(status.maxTimestampDeltaMs);

        // Do not report unavailable calibration values as zero.  These fields
        // appear only after the engine has completed successfully.
        if(status.calibrationAvailable)
        {
            reply.addString("calibrationMode");
            reply.addString(status.calibrationMode);
            if(status.leftMonocularRms >= 0.0)
            {
                reply.addString("leftMonocularRms");
                reply.addFloat64(status.leftMonocularRms);
            }
            if(status.rightMonocularRms >= 0.0)
            {
                reply.addString("rightMonocularRms");
                reply.addFloat64(status.rightMonocularRms);
            }
            if(status.stereoRms >= 0.0)
            {
                reply.addString("stereoRms");
                reply.addFloat64(status.stereoRms);
            }
            if(status.baselineNorm >= 0.0)
            {
                reply.addString("baselineNorm");
                reply.addFloat64(status.baselineNorm);
            }
            if(status.medianVerticalRectificationErrorPx >= 0.0)
            {
                reply.addString("medianVerticalRectificationErrorPx");
                reply.addFloat64(status.medianVerticalRectificationErrorPx);
            }
            if(status.p95VerticalRectificationErrorPx >= 0.0)
            {
                reply.addString("p95VerticalRectificationErrorPx");
                reply.addFloat64(status.p95VerticalRectificationErrorPx);
            }
            if(status.maxVerticalRectificationErrorPx >= 0.0)
            {
                reply.addString("maxVerticalRectificationErrorPx");
                reply.addFloat64(status.maxVerticalRectificationErrorPx);
            }
        }

        reply.addString("lastCalibrationError");
        reply.addString(status.lastCalibrationError.empty() ? "unavailable" : status.lastCalibrationError);
    }
    else if(cmd == "help")
    {
        reply.addString("available commands: start stop status help quit");
    }
    else if(cmd == "quit")
    {
        reply.addString("quitting");
        return false;
    }
    else
    {
        reply.addString("error");
        reply.addString("unknown command");
    }
    
    return true;
}


bool stereoCalibModule::updateModule()
{
    return true;
}


double stereoCalibModule::getPeriod()
{    
   return 0.1;
}
