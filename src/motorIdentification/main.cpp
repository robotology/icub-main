/**
 * 
 * @ingroup icub_module
 * 
 * \defgroup motorIdentification motorIdentification
 * 
 * Controls velocities of a list of joints reading inputs to send from a file.
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * Author: Andrea Tacchetti
 * 
 * Date: first release 18/09/2009
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * \section intro_sec Destription
 * 
 * This is a basic module for cotrolling joints with velocities read from
 * a file.
 * It is possible to control multiple joints, define a starting
 * position and multiply input signals by a scale factor.
 * This module is intended to be used in order to grab measures from motors as
 * they are moving (measures are to be taken by other modules such as
 * \ref controlBoardDumper and \ref dataDumper) without having to sweep
 * positions or velocities manually during long data collecting sessions.
 * It is possible (and recommended) to keep the motors from drifting
 * from their point of labour (that is close to position limits) by
 * adding a proportional position controller. Desired position is calculated
 * automatically by integrating the velocity signal and the proportional
 * gain of each joint's controller can be defined by the user.
 * 
 * \section lib_sec Libraries
 * 
 * YARP libraries
 * 
 * \section parameters_sec Parameters
 *
 * The module uses the ResourceFinder class as a way to retrieve its
 * own configuration. In particular, the default config file is
 * motorIdentificaton.ini and it is assumed to be in
 * src/motorIdentification directory.
 * The File structure is the following:
 *
 * \code
 *
 * InputSignalFile	OneHourChirpTWOJOINTS.dat		//file where
 * signal is read from 
 *
 * rate			5			   	//sampling time (in [ms]) of the input signal
 *
 * lenght			3600			   	//time (in [sec]) of the whole session
 *
 * device			remote_controlboard        	//device
 *
 * local			/test/local			//local port
 *
 * remote			/icub/left_leg			//robot and part to move
 *
 * printStep		500				//program will print joints' positions every printStep*period ms, just to have a feedback
 *
 * keepLinear		1				//toggle the proportional controller
 *
 * nJoints		2				//number of moved joints
 *
 * k			(0.5 0.5)			//gains of proportional controllers (one for each joint)
 *
 * joints			(0 3)				//indexes of moved joints
 *
 * scaleFactors		(0.5 0.5)			//scale factors for each joint's signal
 *
 * startPosition		(21.0 -56.0)			//start position for each joint
 * 
 * \endcode
 *
 * Input file is to be organized with all the signal data for the first joint
 * (of joints list) first, then those of the second and so on.
 * Input file is required to be long enough to meet period, lenght
 * and nJoints specifications. If scaleFactor is not found it is
 * assumed to be 1.0 for each and every joint. Parameters k, joints,
 * scaleFactort and startPosition are required to be of nJoints
 * lenght.
 *
 * \section portsa_sec Ports Accessed
 *
 * The port specified in remote (e.g. /icub/left_leg):
 * 
 * <ul>
 * <li> /icub/left_leg/rpc:i
 * <li> /icub/left_leg/command:i
 * <li> /icub/left_leg/state:i
 * </ul>
 * 
 * \section portsc_sec Ports Created
 *
 * The port specified in local.
 * 
 * \author Andrea Tacchetti
 *
 * Copyright (C) 2009 RobotCub Consortium
 *
 * This file can be edited at src/motorIdentification/main.cpp.
 **/  
 
#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>

#include <string>
#include <iostream>
#include <fstream>
#include <cmath>


using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

using namespace std;

class myThread;

class vControlFile: public RFModule
{
	private:
		Port portHandler;
		PolyDriver pd;
		Property options;
		ifstream inFile;
		int period;
		int timeLenght;
		int nBuf;
		double** buffer;
		double** IntegralOfBuffer;
		int printStep;
		int joints;
		int nUsedJoints;
		int* usedJoints;
		bool keepLinear;
		double* k;
		double* signalScaleFactor;
		double* startPosition;
		myThread *mt;

		bool parseConfFile(yarp::os::ResourceFinder &rf);
		bool getInputSignalFile(yarp::os::ResourceFinder &rf);
		bool getRate(yarp::os::ResourceFinder &rf);
		bool getLenght(yarp::os::ResourceFinder &rf);
		bool getDevice(yarp::os::ResourceFinder &rf);
		bool getLocal(yarp::os::ResourceFinder &rf);
		bool getK(yarp::os::ResourceFinder &rf);
		bool getRemote(yarp::os::ResourceFinder &rf);
		bool getPrintStep(yarp::os::ResourceFinder &rf);
		bool getKeepLinear(yarp::os::ResourceFinder &rf);
		bool getNJoints(yarp::os::ResourceFinder &rf);
		bool getJoints(yarp::os::ResourceFinder &rf);
		bool getScaleFactors(yarp::os::ResourceFinder &rf);
		bool getStartPosition(yarp::os::ResourceFinder &rf);

	public:
		double getPeriod(){return 1.0;}
		bool updateModule();
		bool respond(const Bottle& command, Bottle& reply);
		bool configure(yarp::os::ResourceFinder &rf);
		bool interruptModule();
		virtual bool close();
};

class myThread: public RateThread
{
	private:
		int count;
		IVelocityControl *ivel;
		IEncoders        *iencs;
		IControlLimits	 *ilims;
		IPositionControl *ipos;
		Vector encoders;
		Vector commands;
		Vector supLim;
		Vector infLim;
		Vector errs;
		int printStep;
		int nBuf;
		int nUsedJoints;
		int* usedJoints;

		bool keepLinear;
		double** buffer;
		double** IntegralOfBuffer;
		double* k;
		double* signalScaleFactor;
		double* startPosition;
		PolyDriver *pd;

	public:
		myThread(int period);
		bool threadConfigure(PolyDriver* _pd, double** _buffer, double** _IntegralOfBuffer, double* _k, int _printStep, int _nBuf, bool _keepLinear, double* _signalScaleFactor, int _nUsedJoints, int* _usedJoints, double* _startPosition);
		void run();
		bool threadInit();
		virtual void threadRelease();
};

bool vControlFile::updateModule()
{
}

bool vControlFile::parseConfFile(yarp::os::ResourceFinder &rf)
{
	bool ok;
	ok = getInputSignalFile(rf);
	if(!ok) return false;
	ok = getRate(rf);
	if(!ok) return false;
	ok = getLenght(rf);
	if(!ok) return false;
	ok = getNJoints(rf);
	if(!ok) return false;

	usedJoints = new int[nUsedJoints];
	k = new double[nUsedJoints];
	signalScaleFactor = new double[nUsedJoints];
	startPosition = new double[nUsedJoints];
	if(!rf.check("joints"))
	{
		cout << "Error: could not find joints" << endl;
		return false;
	}
	
	ok = getJoints(rf);
	if(!ok) return false;
	ok = getDevice(rf);
	if(!ok) return false;
	ok = getLocal(rf);
	if(!ok) return false;
	ok = getRemote(rf);
	if(!ok) return false;
	ok = getPrintStep(rf);
	if(!ok) return false;
	ok = getKeepLinear(rf);
	if(!ok) return false;
	ok = getK(rf);
	if(!ok) return false;
	ok = getScaleFactors(rf);
	if(!ok) return false;
	ok = getStartPosition(rf);
	if(!ok) return false;

	return true;
}

bool vControlFile::respond(const Bottle& command, Bottle& reply)
{
	cout<<"Got something, echo is on"<<endl;
        if (command.get(0).asString()=="quit")
            return false;     
        else
            reply=command;
        return true;
}

bool vControlFile::configure(yarp::os::ResourceFinder &rf)
{
	int i, j;
	bool ok;

	/*
	 * INIT
	 */

	buffer = NULL;
	IntegralOfBuffer = NULL;
	usedJoints = NULL;
	k = NULL;
	signalScaleFactor = NULL;
	startPosition = NULL;
	mt = NULL;

	/*
	 * END INIT
	 */

	ok = this->parseConfFile(rf);

	if(!ok)
		return false;

	double dPeriod = (double) 0.001 * period;
	nBuf = (int) (timeLenght / dPeriod);

	/*
	 * SOMETHING WRONG WITH THAT CAST
	 */

	nBuf++;


	buffer = new double*[nUsedJoints];
	for(i=0;i<nUsedJoints;i++)
		buffer[i] = new double[nBuf];
	for(i=0; i<nUsedJoints; i++)
	{
		for(j=0; j<nBuf && inFile >> buffer[i][j]; j++)
		{
			buffer[i][j] = signalScaleFactor[i] * buffer[i][j];
		}
		if(j != nBuf)
		{
			cout << "Error: not enough data in file to fill buffer for joint: "<< i << endl;
			return false;
		}
	}

	IntegralOfBuffer = new double*[nUsedJoints];
	for(i=0;i<nUsedJoints;i++)
		IntegralOfBuffer[i] = new double[nBuf];

	pd.open(options);

	if(!pd.isValid())
	{
		cout << "Could not open device" << endl;
		return false;
	}

	
	portHandler.open("/vControlFile");
	attach(portHandler);

	mt = new myThread(period);
	mt->threadConfigure(&pd,buffer,IntegralOfBuffer,k,printStep,nBuf,keepLinear,signalScaleFactor,nUsedJoints,usedJoints,startPosition);
	mt->start();
	return true;
}

bool vControlFile::interruptModule()
{
	cout << "Interrupting your module for port cleanup" << endl;
	return true;
}

bool vControlFile::close()
{
	cout << "Shutting down module..." << endl;
	cout << "Cleaning up memory..." << endl;

	if(mt != NULL)
	{
		mt->stop();
		delete mt;
	}

	for(int i = 0; i< nUsedJoints; i++)
	{
		if(buffer != NULL && buffer[i] != NULL)
			delete [] buffer[i];
	}
	if(buffer != NULL)
		delete [] buffer;

	for(int i = 0; i< nUsedJoints; i++)
	{
		if(IntegralOfBuffer != NULL && IntegralOfBuffer[i] != NULL)
			delete [] IntegralOfBuffer[i];
	}
	if(IntegralOfBuffer != NULL)
		delete [] IntegralOfBuffer;
	if(usedJoints != NULL)
		delete [] usedJoints;
	if(k != NULL)
		delete [] k;
	if(signalScaleFactor != NULL)
		delete [] signalScaleFactor;
	if(startPosition != NULL)
		delete [] startPosition;
	
	cout << "Done cleaning up memory" << endl;
	portHandler.close();
	inFile.close();
	return true;
}


bool vControlFile::getInputSignalFile(yarp::os::ResourceFinder &rf)
{
	if(!rf.check("InputSignalFile"))
	{
		cout << "Error: could not find InputSignalFile" << endl;
		return false;
	}
	ConstString fileName = rf.find("InputSignalFile").asString();
	inFile.open(fileName.c_str());
	if(!inFile)
	{
		cout << "Could not find data file" << endl;
		return false;
	}

	return true;
}
bool vControlFile::getRate(yarp::os::ResourceFinder &rf)
{
	if(!rf.check("rate"))
	{
		cout << "Error: could not find period" << endl;
		return false;
	}
	period = rf.find("rate").asInt();
	return true;
}
bool vControlFile::getLenght(yarp::os::ResourceFinder &rf)
{
	if(!rf.check("lenght"))
	{
		cout << "Error: could not find lenght" << endl;
		return false;
	}
	timeLenght = rf.find("lenght").asInt();
	return true;
}
bool vControlFile::getDevice(yarp::os::ResourceFinder &rf)
{
	ConstString opt;
	if(!rf.check("device"))
	{
		cout << "Error: could not find device" << endl;
		return false;
	}
	opt = rf.find("device").asString();
	options.put("device",opt.c_str());
	return true;
}
bool vControlFile::getLocal(yarp::os::ResourceFinder &rf)
{
	ConstString opt;
	if(!rf.check("local"))
	{
		cout << "Error: could not find local" << endl;
		return false;
	}
	opt = rf.find("local").asString();
	options.put("local",opt.c_str());
	return true;
}
bool vControlFile::getK(yarp::os::ResourceFinder &rf)
{
	if(keepLinear)
	{
		if(!rf.check("k"))
		{
			cout << "Error: could not find k values" << endl;
			return false;
		}
		Value& vK = rf.find("k");
		Bottle* bK = vK.asList();
		if(bK->size() != nUsedJoints)
		{
			cout << "Error: nJoints and joints parameters are incompatible" <<endl;
			return false;
		}
		for(int i = 0; i<nUsedJoints; i++)
		{
			k[i] = bK->get(i).asDouble();
		}
	}
	return true;
}
bool vControlFile::getRemote(yarp::os::ResourceFinder &rf)
{
	ConstString opt;
	if(!rf.check("remote"))
	{
		cout << "Error: could not find remote" << endl;
		return false;
	}
	opt = rf.find("remote").asString();
	options.put("remote",opt.c_str());
	return true;
}
bool vControlFile::getPrintStep(yarp::os::ResourceFinder &rf)
{
	if(!rf.check("printStep"))
	{
		cout << "Error: could not find printStep" << endl;
		return false;
	}
	printStep = rf.find("printStep").asInt();
	return true;
}
bool vControlFile::getKeepLinear(yarp::os::ResourceFinder &rf)
{
	int kl = rf.find("keepLinear").asInt();
	if(kl == 0 || kl > 1)
	{
		cout << "Warning: not keeping your axes away from position limits!" << endl;
		keepLinear = false;
	}
	else 
		keepLinear = true;
	return true;
}
bool vControlFile::getNJoints(yarp::os::ResourceFinder &rf)
{
	if(!rf.check("nJoints"))
	{
		cout << "Error: could not find nJoints" << endl;
		return false;
	}
	nUsedJoints = rf.find("nJoints").asInt();
	return true;
}
bool vControlFile::getJoints(yarp::os::ResourceFinder &rf)
{
	if(!rf.check("joints"))
	{
		cout << "Error: could not find joints" << endl;
		return false;
	}
	Value& vJoints = rf.find("joints");
	Bottle* pJoints = vJoints.asList();
	if(pJoints->size() != nUsedJoints)
	{
		cout << "Error: nJoints and joints parameters are incompatible" << endl;
		return false;
	}
	for(int i = 0; i< nUsedJoints; i++)
	{
		usedJoints[i] = pJoints->get(i).asInt();
	}
	return true;
}
bool vControlFile::getScaleFactors(yarp::os::ResourceFinder &rf)
{
	if(!rf.check("scaleFactors"))
	{
		for(int i = 0; i < nUsedJoints; i++)
		{
			signalScaleFactor[i] = 1.0;
		}
	}
	else
	{
		Value& vscaleFactors = rf.find("scaleFactors");
		Bottle* bscaleFactors = vscaleFactors.asList();
		if(bscaleFactors->size() != nUsedJoints)
		{
			cout << "Error: nJoints and joints parameters are incompatible" << endl;
			return false;
		}
		for(int i = 0; i< nUsedJoints; i++)
		{
			signalScaleFactor[i] = bscaleFactors->get(i).asDouble();
		}
	}
	return true;
}
bool vControlFile::getStartPosition(yarp::os::ResourceFinder &rf)
{
	if(!rf.check("startPosition"))
	{
		cout << "Error: could not find startPosition" << endl;
		return false;
	}
	else
	{
		Value& vstartPosition = rf.find("startPosition");
		Bottle* bstartPosition = vstartPosition.asList();
		if(bstartPosition->size() != nUsedJoints)
		{
			cout << "Error: startPosition and joints parameters are incompatible" << endl;
			return false;
		}
		for(int i = 0; i< nUsedJoints; i++)
		{
			startPosition[i] = bstartPosition->get(i).asDouble();
		}
	}
	return true;
}

void myThread::run()
{
	for(int i=0; i<nUsedJoints; i++)
	{
		iencs->getEncoder(usedJoints[i],&encoders.data()[i]);
	}

	if(count < nBuf)
	{
		count++;
		for(int i = 0; i< nUsedJoints; i++)
			errs[i] = encoders[i] - IntegralOfBuffer[i][count];

		if(!keepLinear)
		{
			for(int i = 0; i< nUsedJoints; i++)
				commands[i] = buffer[i][count];
		}
		else
		{
			for(int i = 0; i< nUsedJoints; i++)
				commands[i] = buffer[i][count] - k[i]*errs[i];
		}
		for(int i = 0; i< nUsedJoints; i++)
			ivel->velocityMove(usedJoints[i],commands[i]);

		for(int i = 0; i< nUsedJoints; i++)
		{
			if(count % printStep == 0)
				cout << "Position of joint: " << usedJoints[i] << " : " <<encoders[i] << endl;
		}
		for(int i = 0; i<nUsedJoints; i++)
		{	
			if(fabs(encoders[i] - infLim[i]) < .5)
				cout << "Warning : inf limit of joint: "<< usedJoints[i] << " almost reached at step: "<< count << endl;
			if(fabs(encoders.data()[i] - supLim.data()[i]) < .5)
				cout << "Warning : inf limit of joint: "<< usedJoints[i] << " almost reached at step: "<< count << endl;
		}
	}
	else if(count == nBuf)
	{
		cout << "Done! Type CTRL-C" <<endl;
		ivel->stop();
		count++;
	}
	else
		count++;
}

bool myThread::threadInit()
{
	count = 0;
	bool ok;
	ok = pd->view(iencs);
	ok = ok && pd->view(ivel);
	ok = ok && pd->view(ilims);
	ok = ok && pd->view(ipos);

	if(!ok)
	{
		cout<<"Problems acquiring interfaces"<< endl;
		return false;
	}

	if( (!iencs) || (!ivel))
		return false;

	for(int i = 0; i<nUsedJoints; i++)
	{
		cout << "Putting joint : " << usedJoints[i] << " at start position" << endl;
		ipos->positionMove(usedJoints[i],startPosition[i]);
		bool done = false;
		while(!done)
		{
			ipos->checkMotionDone(usedJoints[i],&done);
			Time::delay(0.1);
		}
	}

	double dPeriod = 0.001 * getRate();

	for(int i = 0; i < nUsedJoints; i++)
	{
		while(!iencs->getEncoder(usedJoints[i],&IntegralOfBuffer[i][0]));
		cout << "Start position of joint " << usedJoints[i] <<" : " << IntegralOfBuffer[i][0] << endl;
	}

	for(int i = 0; i< nUsedJoints; i++)
	{
		for(int j = 1; j < nBuf; j++)
		{
			IntegralOfBuffer[i][j] = IntegralOfBuffer[i][j-1] + dPeriod*buffer[i][j];
		}
	}

	encoders.resize(nUsedJoints);
	commands.resize(nUsedJoints);
	supLim.resize(nUsedJoints);
	infLim.resize(nUsedJoints);
	errs.resize(nUsedJoints);

	commands = 1000000;
	
	for(int i = 0; i < nUsedJoints; i++)
		ivel->setRefAcceleration(usedJoints[i],commands[i]);

	for(int i = 0; i < nUsedJoints; i++)
	{
		ilims->getLimits(usedJoints[i],&infLim.data()[i],&supLim.data()[i]);

		cout << "Inferior position limit of joint "<< usedJoints[i] << " : " << infLim.data()[i] <<endl;
		cout << "Superior position limit of joint "<< usedJoints[i] << " : " << supLim.data()[i] <<endl;
	}

	return true;

}

myThread::myThread(int period):RateThread(period){}

bool myThread::threadConfigure(PolyDriver* _pd, double** _buffer, double** _IntegralOfBuffer, double* _k, int _printStep, int _nBuf, bool _keepLinear, double* _signalScaleFactor, int _nUsedJoints, int* _usedJoints, double* _startPosition)
{
	pd = _pd;
	buffer = _buffer;
	IntegralOfBuffer = _IntegralOfBuffer;
	k = _k;
	printStep = _printStep;
	nBuf = _nBuf;
	keepLinear = _keepLinear;
	signalScaleFactor = _signalScaleFactor;
	nUsedJoints = _nUsedJoints;
	usedJoints = _usedJoints;
	startPosition = _startPosition;
}

void myThread::threadRelease()
{
	ivel->stop();
}

int main(int argc, char **argv)
{
	Network yarp;
	vControlFile myVControl;
	ResourceFinder rf;
	
	rf.setDefaultContext("../src/motorIdentification");
	rf.setDefaultConfigFile("motorIdentification.ini");
	rf.configure("ICUB_ROOT",argc,argv);
	rf.setVerbose(true);

	myVControl.configure(rf);

	myVControl.runModule();

	return 0;
}
