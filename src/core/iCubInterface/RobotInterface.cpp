// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RobotInterface.h"

#include <yarp/os/Thread.h>

#include <yarp/os/Os.h>

using namespace yarp::dev;
using namespace yarp::os;

inline void printNoDeviceFound(const char *str)
{
    printf("==========\n");
    printf("Warning: skipping device %s, the automatic procedure failed to detect associated id on the CAN network.\n", str);
    printf("Possible causes could be:\n");
    printf("- The corresponding CAN device is not working\n");
    printf("- No valid descriptor was found on the firmware. Run the canLoader app and ");
    printf("set the 'additional info' string in the firmware so that it starts with %s (case sensitive).\n", str);
}

class ParallelCalibrator: public Thread
{
    IControlCalibration2 *iCalib;
public:
    ParallelCalibrator(IControlCalibration2 *ic)
    {
        iCalib=ic;
    }
    
    void run()
    {
        fprintf(stderr, "Starting calibrator...", iCalib);
        if (iCalib!=0)
        {
            iCalib->calibrate();
            fprintf(stderr, "done\n");
        }
        else
            fprintf(stderr, "skipping, calibrator was not set\n");
    }
};

class ParallelParker: public Thread
{
    IControlCalibration2 *iCalib;
public:
    ParallelParker(IControlCalibration2 *ic)
    {iCalib=ic;}

    void run()
    {
        fprintf(stderr, "Parking...");
        if (iCalib!=0)
        {
            iCalib->park();
            fprintf(stderr, "done\n");
        }
        else
            fprintf(stderr, "skipping, calibrator was not set\n");
    }
};

// implementation of the RobotInterface class

RobotInterface::RobotInterface() 
{
    head_ipc = 0;
    head_amp = 0;
    head_cal = 0;
	gyro_i = 0; 

	rarm_ipc = 0;
	rarm_amp = 0;
	rarm_cal = 0;

    larm_ipc = 0;
	larm_amp = 0;
	larm_cal = 0;


    legs_ipc = 0;
    legs_amp = 0;
    legs_cal = 0;

    initialized = false; 

    hasRightArm=false;
    hasLeftArm=false;
    hasLegs=false;
    hasHead=false;
    isParking=false;
    isCalibrating=false;
    abortF=false;
}  

void RobotInterface::park(bool wait)
{
    if (abortF)
        return;

    isParking=true;

    ParallelParker ph(head_cal);
    ParallelParker pla(larm_cal);
    ParallelParker pra(rarm_cal);
    ParallelParker plegs(legs_cal);

    if (hasHead)
        ph.start();
    if (hasLeftArm)
        pla.start();
    if (hasRightArm)
        pra.start();
    if (hasLegs)
        plegs.start();

    if (hasHead)
        ph.stop();
    if (hasLeftArm)
        pla.stop();
    if (hasRightArm)
        pra.stop();
    if (hasLegs)
        plegs.stop();
    
    isParking=false;
}    

bool RobotInterface::initialize(const std::string &file)
{
    fprintf(stderr, "Going to initialize the robot with a file\n");
#if 0
    // LATER: to be replaced with a yarpish function.
    const char *conf = ACE_OS::getenv("ICUB_ROOT");

    if (conf == NULL) {
		printf("This application requires the ICUB_ROOT environment variable\n");
		printf("defined to point at the configuration files under $ICUB_ROOT/conf\n");
		return false;
	}
#endif
  
    std::string portname;
 
    Property robotOptions;
 //   ACE_OS::sprintf (&filename[0], "%s/conf/%s", conf, inifile.asString().c_str());
	fprintf(stderr, "Read robot description from %s\n", file.c_str());
	robotOptions.fromConfigFile(file.c_str());

    if (robotOptions.findGroup("GENERAL").check("automaticIds"))
        {
            Value &v=robotOptions.findGroup("GENERAL").find("automaticIds");
            
            printf("I'm probing the icub can network on %s...\n", v.asString().c_str());
            can_ids=idDisc.discover(v.asString().c_str());
            idDisc.printList(can_ids);
        }
    else
        printf("Skipping automatic detection of CANbus network (trusting your ini files)\n");

    if (robotOptions.check("HEAD"))
    {
        hasHead=instantiateHead(robotOptions);
        if (!hasHead)
            fprintf(stderr, "RobotInterface::warning troubles instantiating head\n");
    }

    fprintf(stderr, "RobotInterface::now opening inertial\n");
    if (robotOptions.check("INERTIAL")) 
    {
        fprintf(stderr, "RobotInterface:: inertial sensor is in the conf file\n");
        if (!instantiateInertial(robotOptions))
            fprintf(stderr, "RobotInterface::warning troubles instantiating inertial sensor\n");
    }
    else
        fprintf(stderr, "RobotInterface::no inertial sensor defined in the config file\n");

    if (robotOptions.check("RIGHTARM"))
    {
        hasRightArm=instantiateRightArm(robotOptions);
        if (!hasRightArm)
            fprintf(stderr, "RobotInterface::warning troubles instantiating right arm\n");
    }
    
    if (robotOptions.check("LEFTARM"))
    {
        hasLeftArm=instantiateLeftArm(robotOptions);
        if (!hasLeftArm)
            fprintf(stderr, "RobotInterface::warning troubles instantiating left arm\n");
    }

    if (robotOptions.check("LEGS"))
        {
            hasLegs=instantiateLegs(robotOptions);
            if (!hasLegs)
                fprintf(stderr, "RobotInterface::warning troubles instantiating legs\n");
        }

    printf("Starting robot calibration\n");
    isCalibrating=true;
    initialized=true;
    if (abortF)
        return true;

    ParallelCalibrator hc(head_cal);
    ParallelCalibrator lac(larm_cal);
    ParallelCalibrator rac(rarm_cal);
    ParallelCalibrator legsc(legs_cal);

    if (hasHead)
        hc.start();
    if (hasLeftArm)
        lac.start();
    if (hasRightArm)
        rac.start();
    if (hasLegs)
        legsc.start();

    if (hasHead)
        hc.stop();
    if (hasLeftArm)
        lac.stop();
    if (hasRightArm)
        rac.stop();
    if (hasLegs)
        legsc.stop();

    printf("Finished robot calibration!\n");
    isCalibrating=false;
    return true;
}

bool RobotInterface::instantiateRightArm(Property &options)
{
    Property parm;

    fprintf(stderr, "Instantiating RIGHT ARM\n");

	const char *conf = yarp::os::getenv("ICUB_ROOT");

    Value &robotName=options.findGroup("GENERAL").find("name");
    
	//////////// arm
    Value &armInifile=options.findGroup("RIGHTARM").find("file");
    Value &device=options.findGroup("RIGHTARM").find("device");
	Value &subdevice=options.findGroup("RIGHTARM").find("subdevice");
    Value &candevice=options.findGroup("RIGHTARM").find("canbusdevice");

	std::string armFullFilename;
    std::string armPortName;
    armFullFilename+=conf;
    armFullFilename+="/conf/";
    armFullFilename+=armInifile.asString().c_str();
    fprintf(stderr, "Reading from %s\n",armFullFilename.c_str()); 

    parm.fromConfigFile(armFullFilename.c_str());

	parm.put("device", device);
	parm.put("subdevice", subdevice);
    parm.put("canbusdevice", candevice);

    if (options.findGroup("RIGHTARM").check("calibrator"))
    {
        Value &calibrator=options.findGroup("RIGHTARM").find("calibrator");
        Property pTemp;
        pTemp.fromString(parm.toString());
        pTemp.put("device",calibrator.toString());
        rarmCalib.open(pTemp);
    }

	if (options.check("verbose")) 
		parm.put("verbose", 1);
 
    armPortName+="/";
    armPortName+=robotName.asString().c_str();
    armPortName+="/right_arm";
    fprintf(stderr, "RightArm port: %s\n", armPortName.c_str());
    parm.put("name", armPortName.c_str());

    if (idDisc.wasInvoked())
        if (!forceNetworkId(parm, can_ids["RIGHTARM"]))
            {
                printNoDeviceFound("RIGHTARM");
                return false;
            }
    
    // create a device for the arm 
	rarm.open(parm);
    if (!rarm.isValid()) 
		return false; 
  
    bool ok=true;   
  
    ok &= rarm.view(rarm_ipc);
	ok &= rarm.view(rarm_amp);
	ok &= rarm.view(rarm_cal);

    if (rarmCalib.isValid())
    {
        ICalibrator *icalibrator;
        rarmCalib.view(icalibrator);
        rarm_cal->setCalibrator(icalibrator);
    }
    
    return ok; 
} 

bool RobotInterface::instantiateInertial(Property &options)
{
    fprintf(stderr, "Instantiating an INERTIAL device\n");

	const char *conf = yarp::os::getenv("ICUB_ROOT");
    Value &robotName=options.findGroup("GENERAL").find("name");
    
	//////////// inertial
    Value &device=options.findGroup("INERTIAL").find("device");
	Value &subdevice=options.findGroup("INERTIAL").find("subdevice");
    Value &inifile=options.findGroup("INERTIAL").find("file");
 
    std::string fullFilename;
    std::string portName;
    fullFilename+=conf;
    fullFilename+="/conf/";
    fullFilename+=inifile.asString().c_str();
    
    Property p;
    p.fromConfigFile(fullFilename.c_str());

    p.put("device", device);
	p.put("subdevice", subdevice);

    portName+="/";
    portName+=robotName.asString().c_str();
    portName+="/inertial";
    p.put("name", portName.c_str());
 
	// create a device for the arm 
	gyro.open(p);
    if (!gyro.isValid()) 
    {  
		return false;
    } 

    bool ok = gyro.view(gyro_i);
	
    return ok;
}

bool RobotInterface::finalize()
{ 
    if (initialized) 
    {
		if (head.isValid())
			head.close();
		if (rarm.isValid())
			rarm.close();
		if (gyro.isValid())
			gyro.close();
        if (larm.isValid())
            larm.close();
        if (legs.isValid())
            legs.close();

        if (headCalib.isValid())
            headCalib.close();
        if (rarmCalib.isValid())
            rarmCalib.close();
        if (larmCalib.isValid())
            larmCalib.close();
        if (legsCalib.isValid())
            legsCalib.close();
    }
    
    // release interfaces.
    head_ipc = 0;
    head_amp = 0;
    head_cal = 0;
	head_ivc = 0;
			
    rarm_ipc = 0;
    rarm_amp = 0;
    rarm_cal = 0;
 
    larm_ipc = 0;
    larm_amp = 0;
    larm_cal = 0;

    legs_ipc = 0;
    legs_amp = 0;
    legs_cal = 0;

	gyro_i = 0;
  
    initialized = false;
 
    return true;
}
 
bool RobotInterface::instantiateLegs(Property &options)
{
    Property plegs;

    fprintf(stderr, "Instantiating LEGS\n");

	const char *conf = yarp::os::getenv("ICUB_ROOT");
    Value &robotName=options.findGroup("GENERAL").find("name");
    
	//////////// legs
    Value &legsInifile=options.findGroup("LEGS").find("file");
    Value &device=options.findGroup("LEGS").find("device");
	Value &subdevice=options.findGroup("LEGS").find("subdevice");
    Value &candevice=options.findGroup("LEGS").find("canbusdevice");

    std::string legsFullFilename;
    std::string legsPortName;
    legsFullFilename+=conf;
    legsFullFilename+="/conf/";
    legsFullFilename+=legsInifile.asString().c_str();

    plegs.fromConfigFile(legsFullFilename.c_str());

    plegs.put("device", device);
	plegs.put("subdevice", subdevice);
    plegs.put("canbusdevice", candevice);

    if (options.findGroup("LEGS").check("calibrator"))
    {
        Value &calibrator=options.findGroup("LEGS").find("calibrator");
        Property pTemp;
        pTemp.fromString(plegs.toString());
        pTemp.put("device",calibrator.toString());
        legsCalib.open(pTemp);
     }

	if (options.check("verbose")) 
		plegs.put("verbose", 1);
 
    legsPortName+="/";
    legsPortName+=robotName.asString().c_str();
    legsPortName+="/legs";
    fprintf(stderr, "Legs port: %s\n", legsPortName.c_str());
    plegs.put("name", legsPortName.c_str());

    if(idDisc.wasInvoked())
        if (!forceNetworkId(plegs, can_ids["LEGS"]))
            {
                printNoDeviceFound("LEGS");
                return false;
            }

	// create a device for the arm 
	legs.open(plegs);
    if (!legs.isValid()) 
		return false; 
  
    bool ok=true;   
  
    ok &= legs.view(legs_ipc);
	ok &= legs.view(legs_amp);
	ok &= legs.view(legs_cal);

    if (legsCalib.isValid())
    {
        ICalibrator *icalibrator;
        legsCalib.view(icalibrator);
        legs_cal->setCalibrator(icalibrator);
    }

    return ok; 
} 

// check if automatically discovered network id matches the one 
// in the Property, substitute it if necessary.
bool RobotInterface::forceNetworkId(yarp::os::Property& op, int autoN)
{
    bool ret=true;
    if (autoN==-1)
        return false;

    Bottle& can = op.findGroup("CAN");
    int networkN=can.find("CanDeviceNum").asInt();
    Bottle &n=can.addList();
    char tmp[80];
    sprintf(tmp, "CanForcedDeviceNum %d", autoN);
    n.fromString(tmp);

    return ret;
}


void RobotInterface::abort()
{
    if (isParking)
    {
        if (head_cal!=0)
            head_cal->abortPark();
        if (larm_cal!=0)
            larm_cal->abortPark();
        if (rarm_cal!=0)
            rarm_cal->abortPark();
        if (legs_cal!=0)
            legs_cal->abortPark();
    }
    if (isCalibrating)
    {
        if (head_cal!=0)
            head_cal->abortCalibration();
        if (larm_cal!=0)
            larm_cal->abortCalibration();
        if (rarm_cal!=0)
            rarm_cal->abortCalibration();
        if (legs_cal!=0)
            legs_cal->abortCalibration();

    }
    abortF=true;
}


bool RobotInterface::instantiateLeftArm(Property &options)
{
    Property parm;

    fprintf(stderr, "Instantiating LEFT ARM\n");

	const char *conf = yarp::os::getenv("ICUB_ROOT");
    Value &robotName=options.findGroup("GENERAL").find("name");
    
	//////////// arm
    Value &armInifile=options.findGroup("LEFTARM").find("file");
    Value &device=options.findGroup("LEFTARM").find("device");
	Value &subdevice=options.findGroup("LEFTARM").find("subdevice");
    Value &candevice=options.findGroup("LEFTARM").find("canbusdevice");

    std::string armFullFilename;
    std::string armPortName;
    armFullFilename+=conf;
    armFullFilename+="/conf/";
    armFullFilename+=armInifile.asString().c_str();

    parm.fromConfigFile(armFullFilename.c_str());

	parm.put("device", device);
	parm.put("subdevice", subdevice);
    parm.put("canbusdevice",candevice);

   if (options.findGroup("LEFTARM").check("calibrator"))
    {
        Value &calibrator=options.findGroup("LEFTARM").find("calibrator");
        Property pTemp;
        pTemp.fromString(parm.toString());
        pTemp.put("device",calibrator.toString());
        larmCalib.open(pTemp);
    }

	if (options.check("verbose")) 
		parm.put("verbose", 1);
 
    armPortName+="/";
    armPortName+=robotName.asString().c_str();
    armPortName+="/left_arm";
    fprintf(stderr, "LeftArm port: %s\n", armPortName.c_str());
    parm.put("name", armPortName.c_str());
  
    if(idDisc.wasInvoked())
        if (!forceNetworkId(parm, can_ids["LEFTARM"]))
        {
            printNoDeviceFound("LEFTARM");
            return false;
        }

	// create a device for the arm 
	larm.open(parm);
    if (!larm.isValid()) 
		return false; 
  
    bool ok=true;   
  
    ok &= larm.view(larm_ipc);
	ok &= larm.view(larm_amp);
	ok &= larm.view(larm_cal);

    if (larmCalib.isValid())
    {
        ICalibrator *icalibrator;
        larmCalib.view(icalibrator);
        larm_cal->setCalibrator(icalibrator);
    }

    return ok; 
} 

bool RobotInterface::instantiateHead(Property &options)
{
    fprintf(stderr, "Instantiating a HEAD device\n");

	const char *conf = yarp::os::getenv("ICUB_ROOT");
    Value &robotName=options.findGroup("GENERAL").find("name");
      
	//////////// head
    Value &inifile=options.findGroup("HEAD").find("file");
    Value &device=options.findGroup("HEAD").find("device");
	Value &subdevice=options.findGroup("HEAD").find("subdevice");
    Value &candevice=options.findGroup("HEAD").find("canbusdevice");
              
    std::string fullFilename;  
    std::string portName;
    fullFilename+=conf;
    fullFilename+="/conf/";
    fullFilename+=inifile.asString().c_str();

    Property p;

    p.fromConfigFile(fullFilename.c_str());

	p.put("device", device);
    p.put("subdevice", subdevice);
    p.put("canbusdevice",candevice);

    if (options.findGroup("HEAD").check("calibrator"))
    {
        Value &calibrator=options.findGroup("HEAD").find("calibrator");
        Property pTemp;
        pTemp.fromString(p.toString());
        pTemp.put("device",calibrator.toString());
        headCalib.open(pTemp);
     }

    if (options.check("verbose"))
		p.put("verbose", 1);
 
    portName+="/";
    portName+=robotName.asString().c_str();
    portName+="/head";
    p.put("name", portName.c_str());

    if(idDisc.wasInvoked())
        if (!forceNetworkId(p, can_ids["HEAD"]))
            {
                printNoDeviceFound("HEAD");
                return false;
            }

	// create a device for the arm 
	head.open(p);
    if (!head.isValid()) 
        return false;

    bool ok=true;

    ok &= head.view(head_ipc);
	ok &= head.view(head_amp);
  	ok &= head.view(head_cal);

    if (headCalib.isValid())
    {
        fprintf(stderr, "headClib valid\n");
        ICalibrator *icalibrator;
        headCalib.view(icalibrator);
        head_cal->setCalibrator(icalibrator);
     }

    return ok;
}
 
