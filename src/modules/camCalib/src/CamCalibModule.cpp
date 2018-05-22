// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/CamCalibModule.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

CamCalibPort::CamCalibPort()
{
    portImgOut=NULL;
    calibTool=NULL;

    verbose=false;
    t0=Time::now();
}

void CamCalibPort::setPointers(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *_portImgOut, ICalibTool *_calibTool)
{
    portImgOut=_portImgOut;
    calibTool=_calibTool;
}

void CamCalibPort::setSaturation(double satVal)
{
    currSat = satVal;
}

void CamCalibPort::onRead(ImageOf<PixelRgb> &yrpImgIn)
{
    double t=Time::now();
    // execute calibration
    if (portImgOut!=NULL)
    {        
        yarp::sig::ImageOf<PixelRgb> &yrpImgOut=portImgOut->prepare();

        if (verbose)
            yDebug("received input image after %g [s] ... ",t-t0);

        double t1=Time::now();

        if (calibTool!=NULL)
        {
            calibTool->apply(yrpImgIn,yrpImgOut);

            for (int r =0; r <yrpImgOut.height(); r++)
            {
                for (int c=0; c<yrpImgOut.width(); c++)
                {
                    unsigned char *pixel = yrpImgOut.getPixelAddress(c,r);
                    double mean = (1.0/3.0)*(pixel[0]+pixel[1]+pixel[2]);

                    for(int i=0; i<3; i++)
                    {
                        double s=pixel[i]-mean;
                        double sn=currSat*s;
                        sn+=mean;

                        if(sn<0.0)
                            sn=0.0;
                        else if(sn>255.0)
                            sn=255.0;

                        pixel[i]=(unsigned char)sn;
                    }
                }
            }

            if (verbose)
                yDebug("calibrated in %g [s]\n",Time::now()-t1);
        }
        else
        {
            yrpImgOut=yrpImgIn;

            if (verbose)
                yDebug("just copied in %g [s]\n",Time::now()-t1);
        }

        //timestamp propagation
        yarp::os::Stamp stamp;
        BufferedPort<ImageOf<PixelRgb> >::getEnvelope(stamp);
        portImgOut->setEnvelope(stamp);

        portImgOut->writeStrict();
    }

    t0=t;
}

CamCalibModule::CamCalibModule(){

    _calibTool = NULL;  
}

CamCalibModule::~CamCalibModule(){

}

bool CamCalibModule::configure(yarp::os::ResourceFinder &rf){

    string str = rf.check("name", Value("/camCalib"), "module name (string)").asString();
    setName(str.c_str()); // modulePortName

    // pass configuration over to bottle
    Bottle botConfig(rf.toString());
    botConfig.setMonitor(rf.getMonitor());      
    // Load from configuration group ([<group_name>]), if group option present
    Value *valGroup; // check assigns pointer to reference
    if(botConfig.check("group", valGroup, "Configuration group to load module options from (string)."))
    {
        string strGroup = valGroup->asString();        
        // is group a valid bottle?
        if (botConfig.check(strGroup)){            
            Bottle &group=botConfig.findGroup(strGroup,"Loading configuration from group " + strGroup);
            botConfig.fromString(group.toString());
        }
        else{
            yError() << "Group " << strGroup << " not found.";
            return false;
        }
    }
    else
    {
        yError ("There seem to be an error loading parameters (group section missing), stopping module");
        return false;
    }

    string calibToolName = botConfig.check("projection",
                                         Value("pinhole"),
                                         "Projection/mapping applied to calibrated image [pinhole|spherical] (string).").asString();

    _calibTool = CalibToolFactories::getPool().get(calibToolName.c_str());
    if (_calibTool!=NULL) {
        bool ok = _calibTool->open(botConfig);
        if (!ok) {
            delete _calibTool;
            _calibTool = NULL;
            return false;
        }
    }

    if (yarp::os::Network::exists(getName("/in")))
    {
        yWarning() << "port " << getName("/in") << " already in use";
    }
    if (yarp::os::Network::exists(getName("/out")))
    {
        yWarning() << "port " << getName("/out") << " already in use";
    }
    if (yarp::os::Network::exists(getName("/conf")))
    {
        yWarning() << "port " << getName("/conf") << " already in use";
    }
    _prtImgIn.setSaturation(rf.check("saturation",Value(1.0)).asDouble());
    _prtImgIn.open(getName("/in"));
    _prtImgIn.setPointers(&_prtImgOut,_calibTool);
    _prtImgIn.setVerbose(rf.check("verbose"));
    _prtImgIn.useCallback();
    _prtImgOut.open(getName("/out"));
    _configPort.open(getName("/conf"));

    attach(_configPort);
    fflush(stdout);

    return true;
}

bool CamCalibModule::close(){
    _prtImgIn.close();
    _prtImgOut.close();
    _configPort.close();
    if (_calibTool != NULL){
        _calibTool->close();
        delete _calibTool;
        _calibTool = NULL;
    }
    return true;
}

bool CamCalibModule::interruptModule(){
    _prtImgIn.interrupt();
    _prtImgOut.interrupt();
    _configPort.interrupt();
    return true;
}

bool CamCalibModule::updateModule(){
    return true;
}

double CamCalibModule::getPeriod() {
  return 1.0;
}

bool CamCalibModule::respond(const Bottle& command, Bottle& reply) 
{
    reply.clear(); 

    if (command.get(0).asString()=="quit") 
    {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="sat" || command.get(0).asString()=="saturation")
    {
        double satVal = command.get(1).asDouble();
        _prtImgIn.setSaturation(satVal);
        
        reply.addString("ok");
    }
    else
    {
        yError() << "command not known - type help for more info";
    }
    return true;
}


