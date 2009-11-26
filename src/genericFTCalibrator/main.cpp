/**
@ingroup icub_module

\defgroup genericFTCalibrator genericFTCalibrator
 
Converts Voltage row values from FT sensors and converts their measurements into
equivalent Force/Torque measurements. 

Copyright (C) 2008 RobotCub Consortium
 
Author: Matteo Fumagalli
 
Date: first release 26/11/2009 

CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/


#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iostream>
#include <iomanip>
#include <string>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
//using namespace ctrl;
using namespace std;

// class dataCollector: class for reading from Vrow and providing for FT values on an output port

class dataCollector : public BufferedPort<Bottle>
{
private:
	Matrix C;
	BufferedPort<Vector> &port_FT;
	
	virtual void onRead(Bottle &b)
	{
		Stamp info;
		BufferedPort<Bottle>::getEnvelope(info);

		size_t sz = b.size();
		Vector x(sz);
		for(unsigned int i=0;i<sz;i++)
			x[i]=b.get(i).asDouble();
		
		Vector FT=C*x;
		
		port_FT.prepare() = FT;
		port_FT.setEnvelope(info);
		port_FT.write();
	}
public:
	dataCollector(BufferedPort<Vector> &_port_FT, ResourceFinder &rf) :
	  port_FT(_port_FT)
	  {
		  if(rf.check("rows") && rf.check("cols"))
			  C.resize(rf.find("rows").asInt(),rf.find("cols").asInt());
		  else
		  {
			  fprintf(stderr,"missing information on matrix caibration!!! Default matrix = I(6,6)\n");
			  C=eye(6,6);
		  }

		  string row;
		  char buf[2];

		  //reading calibration matrix
		  fprintf(stderr,"C:\n");
		  for(int i = 0; i<C.rows(); i++)
		  {
			  row = string("row") + _itoa(i,buf,10);
			  fprintf(stderr,"%s\n", row.c_str());
			  
			  if(rf.check(row.c_str()))
			  {
				  for(int j=0;j<C.cols();j++)
				  {
					  C(i,j)=rf.findGroup(row.c_str()).get(j+1).asDouble();
					  fprintf(stderr,"%.2lf\t",C(i,j));
				  }
				  fprintf(stderr,"\n");
			  }
		  }
	  }
};

// class FT_Calibrator: class which listen to a port sending Volts, and converts the row into the equivalent FT values

class FT_Calibrator: public RFModule
{
private:
	dataCollector *port_V;
	BufferedPort<Vector> port_FT;
public:
	FT_Calibrator(){ }

	virtual bool configure(ResourceFinder &rf)
	{
		string PortName;

		ConstString robotName=rf.find("robot").asString();
		if (rf.check("robot"))
            PortName=rf.find("robot").asString();
        else
		{
			fprintf(stderr,"Device not found\n");
            PortName="/iCub";
		}
		
		ConstString partName=rf.find("part").asString();
		if (rf.check("part"))
			PortName=(PortName+"/")+rf.find("part").asString().c_str();
        else
		{
			fprintf(stderr,"Device not found\n");
            PortName=PortName+"/LeftArm";
		}

		int rows;
		int cols;
		if(rf.check("rows"))	
		{
			rows = rf.find("rows").asInt();
			fprintf(stderr,"number of rows: %d\n", rows);
		}
		else 
		{
			rows=0;
			fprintf(stderr,"error! row string not found.\nreturning...");
			return true;
		}
		if(rf.check("cols"))	
		{
			rows = rf.find("cols").asInt();
			fprintf(stderr,"number of cols: %d\n", rows);
		}
		else 
		{
			rows=0;
			fprintf(stderr,"error! cols string not found.\nreturning...");
			return true;
		}

		port_FT.open(("/" + PortName+"/FT:o").c_str());//PortNameFT.c_str());
		port_V = new dataCollector(port_FT, rf);
		port_V->useCallback();
		port_V->open(("/" + PortName+"/V:i").c_str());
		return true;
	}

	
	virtual double getPeriod()	{ return 1.0; }
	virtual bool updateModule() { return true; }
	
	
	virtual bool close()
	{
		port_FT.interrupt();
		port_FT.close();

		port_V->interrupt();
		port_V->close();

		delete port_V;
		return true;
	}
};



int main(int argc, char * argv[])
{
    //initialize yarp network
    Network yarp;

    // prepare and configure the resource finder
    ResourceFinder rf;
    rf.setVerbose();
	rf.setDefaultContext("FTSensorCalibration/conf");
	rf.setDefaultConfigFile("LeftArmFTCal.ini");

    rf.configure("ICUB_ROOT", argc, argv);

    //create your module
    FT_Calibrator ft_sender;

    cout<<"Configure module..."<<endl;
    ft_sender.configure(rf);
    cout<<"Start module..."<<endl;
    ft_sender.runModule();

    cout<<"Main returning..."<<endl;
		  // Connect the ports so that anything written from /out arrives to /in
		  

    return 0;
}


