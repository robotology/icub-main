// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/CamCalibModule.h>

CamCalibPort::CamCalibPort()
{
    portImgOut=NULL;
    calibTool=NULL;

    verbose=false;
    t0=Time::now();
}

void CamCalibPort::setPointers(yarp::os::Port *_portImgOut, ICalibTool *_calibTool)
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
    int temp = 0;
    // execute calibration
    if (portImgOut!=NULL)
    {        
        yarp::sig::ImageOf<PixelRgb> yrpImgOut;

        if (verbose)
            fprintf(stdout,"received input image after %g [s] ... ",t-t0);

        double t1=Time::now();

        if (calibTool!=NULL)
        {
            calibTool->apply(yrpImgIn,yrpImgOut);
           
            cv::Mat cv_img (yrpImgOut.height(), yrpImgOut.width(), CV_8UC3);

            vector<cv::Mat> planes;
            cv::cvtColor( cv::Mat((IplImage*)yrpImgOut.getIplImage()), cv_img, CV_RGB2BGR);
            cv::split(cv_img, planes);
            
            for (int c =0; c <cv_img.cols; c++)
            {
                for (int r=0; r<cv_img.rows; r++)
                {
                    double mean=(1.0/3.0)*(planes[0].at<uchar>(r,c)+planes[1].at<uchar>(r,c)+planes[2].at<uchar>(r,c));
                    for(int i=0; i<3; i++)
                    {
                        double s=planes[i].at<uchar>(r,c)-mean;
                        double sn=currSat*s;
                        sn+=mean;

                        if(sn<0.0)
                            sn=0.0;
                        if(sn>255.0)
                            sn=255.0;

                        planes[i].at<uchar>(r,c)=sn;
                    }
                }
            }
            cv::merge(planes,cv_img);
            cv::cvtColor( cv_img, cv_img, CV_BGR2RGB);

            IplImage test = cv_img;
            
            ImageOf<PixelRgb> yarpImg;
            yarpImg.resize(test.width,test.height);
            cvCopyImage(&test, (IplImage*)yarpImg.getIplImage());
            yrpImgOut.zero();
            yrpImgOut = yarpImg;

            if (verbose)
                fprintf(stdout,"calibrated in %g [s]\n",Time::now()-t1);
        }
        else
        {
            yrpImgOut=yrpImgIn;

            if (verbose)
                fprintf(stdout,"just copied in %g [s]\n",Time::now()-t1);
        }

        // timestamp propagation
        yarp::os::Stamp stamp;
        BufferedPort<ImageOf<PixelRgb> >::getEnvelope(stamp);
        portImgOut->setEnvelope(stamp);

        portImgOut->write(yrpImgOut);
    }

    t0=t;
}

CamCalibModule::CamCalibModule(){

    _calibTool = NULL;	
}

CamCalibModule::~CamCalibModule(){

}

bool CamCalibModule::configure(yarp::os::ResourceFinder &rf){

    ConstString str = rf.check("name", Value("/camCalib"), "module name (string)").asString();
    setName(str.c_str()); // modulePortName

    // pass configuration over to bottle
    Bottle botConfig(rf.toString().c_str());
    botConfig.setMonitor(rf.getMonitor());		
    // Load from configuration group ([<group_name>]), if group option present
    Value *valGroup; // check assigns pointer to reference
    if(botConfig.check("group", valGroup, "Configuration group to load module options from (string).")){
        string strGroup = valGroup->asString().c_str();        
        // is group a valid bottle?
        if (botConfig.check(strGroup.c_str())){            
            Bottle &group=botConfig.findGroup(strGroup.c_str(),string("Loading configuration from group " + strGroup).c_str());
            botConfig.fromString(group.toString());
        }
        else{
            cout << endl << "Group " << strGroup << " not found." << endl;
            return false;
        }
    }

    string calibToolName = botConfig.check("projection",
                                         Value("pinhole"),
                                         "Projection/mapping applied to calibrated image [projection|spherical] (string).").asString().c_str();

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
        cout << "====> warning: port " << getName("/in") << " already in use" << endl;
    }
    if (yarp::os::Network::exists(getName("/out")))
    {
        cout << "====> warning: port " << getName("/out") << " already in use" << endl;    
    }
    if (yarp::os::Network::exists(getName("/conf")))
    {
        cout << "====> warning: port " << getName("/conf") << " already in use" << endl;    
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
        cout << "command not known - type help for more info" << endl;
    }
    return true;
}


