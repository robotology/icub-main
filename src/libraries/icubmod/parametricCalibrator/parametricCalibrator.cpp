// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006 Giorgio Metta, Lorenzo Natale
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>

#include "parametricCalibrator.h"
#include <math.h>

#include "Debug.h"

using namespace yarp::os;
using namespace yarp::dev;

// calibrator for the arm of the Arm iCub

const int 		PARK_TIMEOUT=30;
const double 	GO_TO_ZERO_TIMEOUT		= 10; //seconds how many? // was 10
const int 		CALIBRATE_JOINT_TIMEOUT	= 25;
const double 	POSITION_THRESHOLD		= 2.0;

int numberOfJoints =0;

parametricCalibrator::parametricCalibrator() :
    logfile(stderr),
    type(NULL),
    param1(NULL),
    param2(NULL),
    param3(NULL),
    original_pid(NULL),
    limited_pid(NULL),
    maxPWM(NULL),
    currPos(NULL),
    currVel(NULL),
    zeroPos(NULL),
    zeroVel(NULL),
    homeVel(0),
    homePos(0),
    abortCalib(false)
{
}

parametricCalibrator::~parametricCalibrator()
{
    close();
}

bool parametricCalibrator::open(yarp::os::Searchable& config)
{
	Property p;
	p.fromString(config.toString());

	if (!p.check("GENERAL")) {
		fprintf(logfile, "CALIB::Cannot understand configuration parameters\n");
		return false;
	}

	yDebug() << "eo_iCubArmCalibratorJ8 parameters:\n" << p.toString().c_str();


#warning " era GENERAL"
	int nj = p.findGroup("CALIBRATION").find("Joints").asInt();
	if (nj == 0)
	{
		fprintf(logfile, "CALIB[part?]: Calibrator is for %d joints but device has %d\n", numberOfJoints, nj);
		return false;
	}

	type = new unsigned char[nj];
	param1 = new double[nj];
	param2 = new double[nj];
	param3 = new double[nj];
	maxPWM = new int[nj];

	zeroPos = new double[nj];
	zeroVel = new double[nj];
	currPos = new double[nj];
	currVel = new double[nj];
	homePos = new double[nj];
	homeVel = new double[nj];

	logfile_name = p.findGroup("CALIBRATION").find("Logfile").asString();
	if (logfile_name != "")
	{
		fprintf(stdout, "CALIB::calibrator: opening logfile %s\n", logfile_name.c_str());
		logfile = fopen (logfile_name.c_str(), "w");
	}
	else
	{
		fprintf(stdout, "CALIB: no logfile specified, errors displayed on standard stderr\n");
	}

	Bottle& xtmp = p.findGroup("CALIBRATION").findGroup("Calibration1");

	int i;
	for (i = 1; i < xtmp.size(); i++)
		param1[i-1] = xtmp.get(i).asDouble();
	xtmp = p.findGroup("CALIBRATION").findGroup("Calibration2");

	for (i = 1; i < xtmp.size(); i++)
		param2[i-1] = xtmp.get(i).asDouble();
	xtmp = p.findGroup("CALIBRATION").findGroup("Calibration3");

	for (i = 1; i < xtmp.size(); i++)
		param3[i-1] = xtmp.get(i).asDouble();
	xtmp = p.findGroup("CALIBRATION").findGroup("CalibrationType");

	for (i = 1; i < xtmp.size(); i++)
		type[i-1] = (unsigned char) xtmp.get(i).asDouble();


	xtmp = p.findGroup("CALIBRATION").findGroup("PositionZero");

	for (i = 1; i < xtmp.size(); i++)
		zeroPos[i-1] = xtmp.get(i).asDouble();

	xtmp = p.findGroup("CALIBRATION").findGroup("VelocityZero");

	for (i = 1; i < xtmp.size(); i++)
		zeroVel[i-1] = xtmp.get(i).asDouble();

	xtmp = p.findGroup("HOME").findGroup("PositionHome");

	for (i = 1; i < xtmp.size(); i++)
		homePos[i-1] = xtmp.get(i).asDouble();

	xtmp = p.findGroup("HOME").findGroup("VelocityHome");

	for (i = 1; i < xtmp.size(); i++)
		homeVel[i-1] = xtmp.get(i).asDouble();

	if (p.findGroup("CALIBRATION").check("MaxPWM"))
	{
		xtmp = p.findGroup("CALIBRATION").findGroup("MaxPWM");
		for (i = 1; i < xtmp.size(); i++) maxPWM[i-1] =  xtmp.get(i).asInt();
	}
	else
	{
		fprintf(logfile, "CALIB[part?] :MaxPWM parameter not found, assuming 60\n");
		for (i = 1; i < nj+1; i++) maxPWM[i-1] = 60;
	}


	xtmp = p.findGroup("CALIB_ORDER");

	yDebug() << "Group size " << xtmp.size() << "\nValues: " << xtmp.toString().c_str();

	std::list<int>  tmp;

	for(int i=1; i<xtmp.size(); i++)
	{
		tmp.clear();
		Bottle *set;
		set= xtmp.get(i).asList();

		for(int j=0; j<set->size(); j++)
		{
			tmp.push_back(set->get(j).asInt() );
		}
		joints.push_back(tmp);
	}
	return true;
}

bool parametricCalibrator::close ()
{
    if (type != NULL) {
        delete[] type;
        type = NULL;
    }
    if (param1 != NULL) {
        delete[] param1;
        param1 = NULL;
    }
    if (param2 != NULL) {
        delete[] param2;
        param2 = NULL;
    }
    if (param3 != NULL) {
        delete[] param3;
        param3 = NULL;
    }

    if (maxPWM != NULL) {
        delete[] maxPWM;
        maxPWM = NULL;
    }
    if (original_pid != NULL) {
        delete[] original_pid;
        original_pid = NULL;
    }
    if (limited_pid != NULL) {
        delete[] limited_pid;
        limited_pid = NULL;
    }

    if (currPos != NULL) {
        delete[] currPos;
        currPos = NULL;
    }
    if (currVel != NULL) {
        delete[] currVel;
        currVel = NULL;
    }

    if (zeroPos != NULL) {
        delete[] zeroPos;
        zeroPos = NULL;
    }
    if (zeroVel != NULL) {
        delete[] zeroVel;
        zeroVel = NULL;
    }

    if (homePos != NULL) {
        delete[] homePos;
        homePos = NULL;
    }
    if (homeVel != NULL) {
        delete[] homeVel;
        homeVel = NULL;
    }

    if (logfile_name != "") {
        fclose(logfile);
        logfile_name.clear();
        logfile = stderr;
    }

    return true;
}

bool parametricCalibrator::calibrate(DeviceDriver *dd)  // dd dovrebbe essere il wrapper, non mc
{
    yTrace();
    abortCalib=false;

	bool calibration_ok = true;
	bool goHome_ok = true;
	int  setOfJoint_idx = 0;

	int nj=0;
	int totJointsToCalibrate = 0;

    yarp::dev::PolyDriver *p = dynamic_cast<yarp::dev::PolyDriver *>(dd);
    p->view(iCalibrate);
    p->view(iAmps);
    p->view(iEncoders);
    p->view(iPosition);
    p->view(iPids);
    p->view(iControlMode);

    if (!(iCalibrate && iAmps && iPosition && iPids && iControlMode)) {
        yError() << "CALIB: interface not found" << iCalibrate << iAmps << iPosition << iPids << iControlMode;
        return false;
    }

	if ( !iEncoders->getAxes(&nj))
	{
		yError() << "CALIB: error getting number of encoders" ;
		return false;
	}

	// ok we have all interfaces


	int a = joints.size();
	printf("List of list size %d\n", a);

	std::list<int>  tmp;

	std::list<std::list<int> >::iterator Bit=joints.begin();
	std::list<std::list<int> >::iterator Bend=joints.end();

	std::list<int>::iterator lit;
	std::list<int>::iterator lend;


	while(Bit != Bend)
	{
		tmp.clear();
		tmp = (*Bit);
		lit  = tmp.begin();
		lend = tmp.end();
		printf("Joints calibration order :\n");
		totJointsToCalibrate += tmp.size();

		while(lit != lend)
		{
			printf("%d,", (*lit));
			lit++;
		}
		printf("\n");
		Bit++;
	}

	if (totJointsToCalibrate > nj)
	{
		yError() << "CALIB: too much axis to calibrate for this part..." << totJointsToCalibrate << " bigger than "<< nj;
		return false;
	}


	original_pid=new Pid[nj];
	limited_pid =new Pid[nj];

	Bit=joints.begin();

	printf(" before sleep\n");
	usleep(4 * 1000 * 1000);
	printf("After sleep\n");
	while( (Bit != Bend) && (!abortCalib) )			// per ogni set di giunti
	{
		setOfJoint_idx++;
		tmp.clear();
		tmp = (*Bit);

		lit  = tmp.begin();
		lend = tmp.end();
		while( (lit != lend) && (!abortCalib) )		// per ogni giunto del set
		{
			if ((*lit) >= nj)		// check the axes actually exists
			{
				yError() << "Asked to calibrate joint" << (*lit) << ", which is bigger than the number of axes for this part ("<< nj << ")";
				return false;
			}
			if(!iPids->getPid((*lit),&original_pid[(*lit)]) )
			{
				yError() << "getPid joint " << (*lit) << "failed... aborting calibration";
				return false;
			}
			limited_pid[(*lit)]=original_pid[(*lit)];
			limited_pid[(*lit)].max_int=maxPWM[(*lit)];
			limited_pid[(*lit)].max_output=maxPWM[(*lit)];
			iPids->setPid((*lit),limited_pid[(*lit)]);			// per i giunti delle 4dc, il valore da usare è quello normale
			lit++;
		}

		//
		// Calibrazione
		//

		lit  = tmp.begin();
		while(lit != lend)		// per ogni giunto del set
		{
			iEncoders->getEncoders(currPos);

			// Enable amp moved to MotionControl class;
			// Here we just call the calibration procedure
			yDebug() <<  "CALIB["  << setOfJoint_idx << ":" << (*lit) << "]: Calibrating... current enc values: " << currPos[(*lit)];
			calibrateJoint((*lit));
			lit++;
		}
		Time::delay(1.0);	// needed?


		if(checkCalibrateJointEnded((*Bit)) )
		{
			yDebug() <<  "CALIB["  << setOfJoint_idx << ":" << (*lit) << "]: Calibration done!\n";
			lit  = tmp.begin();
			lend = tmp.end();
			while( (lit != lend) && (!abortCalib) )		// per ogni giunto del set
			{
				iPids->setPid((*lit),original_pid[(*lit)]);
				lit++;
			}
		}
	    else			// keep pid safe  and go on
	    {
			yError() <<  "CALIB["  << setOfJoint_idx << ":" << (*lit) << "]: Calibration went wrong! Disabling axes and keeping safe pid limit\n";
			while( (lit != lend) && (!abortCalib) )		// per ogni giunto del set
			{
	            iAmps->disableAmp((*lit));
				lit++;
			}
	    }


		lit  = tmp.begin();
		while(lit != lend)		// per ogni giunto del set
		{
			// Manda in Home
			goToZero((*lit));
			lit++;
		}
		Time::delay(1.0);	// needed?

		bool goneToZero = true;
		lit  = tmp.begin();
		while(lit != lend)		// per ogni giunto del set
		{
			// abs sensors is BLL style
			goneToZero &= checkGoneToZero(*lit);
			lit++;
		}

		if(goneToZero)
		{
			yDebug() <<  "CALIB["  << setOfJoint_idx << ":" << (*lit) << "]: Reached zero position!\n";
	    }
	    else			// keep pid safe and go on
	    {
			yError() <<  "CALIB["  << setOfJoint_idx << ":" << (*lit) << "]: some axis got timeout while reaching zero position... disabling this set of axes\n";
			while( (lit != lend) && (!abortCalib) )		// per ogni giunto del set
			{
	            iAmps->disableAmp((*lit));
				lit++;
			}
	    }

		// Go to the next set of joints to calibrate... if any
		Bit++;
	}
	return calibration_ok;
}

void parametricCalibrator::calibrateJoint(int joint)
{
	yDebug() << "CALIB[part?]: Calling calibrateJoint on joint "<< joint << " with params: " << type[joint] << param1[joint] << param2[joint] << param3[joint];
	iCalibrate->calibrate2(joint, type[joint], param1[joint], param2[joint], param3[joint]);
}

bool parametricCalibrator::checkCalibrateJointEnded(std::list<int> set)
{
	int timeout = 0;
	bool calibration_ok = false;

	std::list<int>::iterator lit;
	std::list<int>::iterator lend;

	lend = set.end();
	while(!calibration_ok && (timeout <= CALIBRATE_JOINT_TIMEOUT))
	{
		calibration_ok = true;
		lit  = set.begin();
		while(lit != lend)		// per ogni giunto del set
		{
			printf("check calib joint ended (%d)\n", (*lit));
			if (abortCalib)
			{
				yDebug() << "CALIB: aborted\n";
			}

			// Joint with absolute sensor doesn't need to move, so they are ok with just the calibration message,
			// but I'll check anyway, in order to have everything the same
			if( !(calibration_ok &=  iCalibrate->done((*lit))) )		// the assignement inside the if is INTENTIONAL
				break;
			lit++;
		}
		Time::delay(1.0);
		timeout++;
	}

	if(timeout > CALIBRATE_JOINT_TIMEOUT)
		yError() << "CALIB Timeout while calibrating!\n";

	return calibration_ok;
}


void parametricCalibrator::goToZero(int j)
{
	if (abortCalib)
		return;
	iControlMode->setPositionMode(j);
	iPosition->setRefSpeed(j, zeroVel[j]);
	iPosition->positionMove(j, zeroPos[j]);
}

bool parametricCalibrator::checkGoneToZero(int j)
{
	// wait.
	bool ok = false;
	double start_time = yarp::os::Time::now();

	while ( (!ok) && (!abortCalib))
	{
		iPosition->checkMotionDone(j, &ok);

		if (yarp::os::Time::now() - start_time > GO_TO_ZERO_TIMEOUT)
		{
			yError() << "CALIB[" << j << "]: Timeout while going to zero!\n";
			ok = false;
			break;
		}
	}
	if (abortCalib)
		yWarning() << "CALIB[" << j << "]: abort wait for joint %d going to zero!\n";   // quale parte del corpo?

	return ok;
}

// Not used anymore... EMS knows wath to do. Just ask if motion is done!! ^_^
bool parametricCalibrator::checkGoneToZeroThreshold(int j)
{
	// wait.
	bool finished = false;
	double ang[4];
	double angj = 0;
	double pwm[4];
	double delta=0;

	double start_time = yarp::os::Time::now();
	while ( (!finished) && (!abortCalib))
	{
		iEncoders->getEncoder(j, &angj);
		for (int l=0; l<4; l++)
		{
			iEncoders->getEncoder(l, &ang[l]);
			iPids->getOutput(l,&pwm[l]);
		}

		delta = fabs(angj-zeroPos[j]);
		fprintf(logfile, "CALIB[part] (joint %d) curr:%+7.2f des:%+7.2f -> delta:%+7.2f \
                          *** encs: %+7.2f %+7.2f %+7.2f %+7.2f pwms: %+7.2f %+7.2f %+7.2f %+7.2f\n",
                          j, angj, zeroPos[j], delta,
                          ang[0], ang[1], ang[2], ang[3],
                          pwm[0], pwm[1], pwm[2], pwm[3]);
		if (delta<POSITION_THRESHOLD)
		{
			fprintf(logfile, "CALIB[part?] (joint %d) completed! delta:%f\n", j,delta);
			finished=true;
		}

		if (logfile_name != "")
			Time::delay (0.1);
		else
			Time::delay (0.5);

		if (yarp::os::Time::now() - start_time > GO_TO_ZERO_TIMEOUT)
		{
			fprintf(logfile, "CALIB[part?]: Timeout on joint %d while going to zero!\n", j);
			return false;
		}
	}
	if (abortCalib)
		fprintf(logfile, "CALIB[part?]: Abort wait for joint %d going to zero!\n", j);

	return finished;
}

bool parametricCalibrator::park(DeviceDriver *dd, bool wait)
{
	yTrace();
	int nj=0;
	bool ret=false;
	abortParking=false;

	if ( !iEncoders->getAxes(&nj))
	{
		yError() << "CALIB[part?]: error getting number of encoders" ;
		return false;
	}

	if (nj!=numberOfJoints)
	{
		yError() << "CALIB[part?]: calibrator is for %d joints but device has %d\n", numberOfJoints, nj;
		return false;
	}

	int timeout = 0;

	iPosition->setPositionMode();
	iPosition->setRefSpeeds(homeVel);
	iPosition->positionMove(homePos);			// all joints together????

	if (wait)
	{
		fprintf(logfile, "CALIB[part?]: Moving to park positions \n");
		bool done=false;
		while((!done) && (timeout<PARK_TIMEOUT) && (!abortParking))
		{
			iPosition->checkMotionDone(&done);
			fprintf(logfile, ".");
			Time::delay(1);
			timeout++;
		}
		if(!done)
		{	// In case of error do another loop trying to detect the error!!
			for(int j=0; j < nj; j++)
			{
				if (iPosition->checkMotionDone(j, &done))
				{
					if (!done)
						yError() << "CALIB[part?]: joint " << j << " not in position after timeout";
					// else means that asxe get to the position right after the timeout.... do nothing here
				}
				else	// if the CALL to checkMotionDone fails for timeout
					yError() << "CALIB[part?]: joint " << j << "did not answer during park";
			}
		}
	}

	if (abortParking)
		fprintf(logfile, "CALIB[part?]: Park was aborted!\n");
	else
		fprintf(logfile, "CALIB[part?]: Park was done!\n");

	// iCubInterface is already shutting down here... so even if errors occour, what else can I do?

	return true;
}

bool parametricCalibrator::quitCalibrate()
{
	fprintf(logfile, "CALIB[part?]: Quitting calibrate\n");
	abortCalib=true;
	return true;
}

bool parametricCalibrator::quitPark()
{
	fprintf(logfile, "CALIB[part?]: Quitting parking\n");
	abortParking=true;
	return true;
}
