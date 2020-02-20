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
    if (command.get(0).asString()=="start") {
        reply.addString("Starting Calibration...");
        calibThread->startCalib();
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


