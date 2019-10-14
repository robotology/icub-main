#include "stereoCalibModule.h"
#include <yarp/os/Stamp.h>
#include <yarp/os/Os.h>



bool stereoCalibModule::configure(yarp::os::ResourceFinder &rf)
{
    moduleName=rf.check("name",Value("stereoCalib"),"module name (string)").asString().c_str();
    setName(moduleName.c_str());

    handlerPortName="/";
    handlerPortName+=getName(rf.check("CommandPort",Value("/cmd"),"Output image port (string)").asString());

    dir=rf.getHomeContextPath().c_str();

    if (!handlerPort.open(handlerPortName.c_str()))
    {
        cout << ": unable to open port " << handlerPortName << endl;
        return false;
    }
    attach(handlerPort);

    calibThread=new stereoCalibThread(rf,&handlerPort);
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
    if (command.get(0).asString()=="start")
    {
        //create a new folder everytime the it receives the start commmand
        bool proceed=true;
        for (int i=1; proceed; i++)
        {
            snprintf(dirName,PATH_LEN-1,"%s/%s_%.5d", dir.c_str(), "calibImg",i);
            proceed=!yarp::os::stat(dirName);
            snprintf(dirName,PATH_LEN-1,"%s/%s_%.5d/", dir.c_str(),"calibImg",i);
        }    
        yarp::os::mkdir_p(dirName);
        
        reply.addString("Starting Calibration...");
        calibThread->startCalib(dirName);
    }
    else if (command.get(0).asString()=="cfi")
    {
        reply.addString("Calibration From Images (cfi) ...");
        calibThread->calibFromImages();
        reply.addString("... Done");
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


