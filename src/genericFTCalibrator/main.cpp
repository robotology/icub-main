/**
* @ingroup icub_module
*
* \defgroup genericFTCalibrator genericFTCalibrator
* 
* Converts Voltage row values from FT sensors into
* equivalent Force/Torque measurements. 
* 
* Copyright (C) 2008 RobotCub Consortium
*  
* Author: Matteo Fumagalli
*  
* Date: first release 26/11/2009 
* 
* CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description

This module converts the Forces and Torques of an FT sensor,
given the input vector of the measured Voltages. Voltages are 
passed to the module through a YARP port and the outputs are
sent to another YARP port. The convertion is provided by the 
Calibration matrix, which is read by the module using a 
configuration file. 

\section lib_sec Libraries 
- YARP libraries. 

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section conf_file_sec Configuration Files
- FTCalibration.ini. (Default: LeftArmFT.ini)

\section parameters_sec Parameters
--context \e context 
- The parameter \e context identifies the context to add where
  the resource finder will look for the configuration file. 
  Note that the context path is relative to $ICUB_ROOT/app.
 
--from \e from
- The parameter \e from identifies the configuration file name
  to be read by the module in order to define the ports to open
  and the calibration matrix. (Default: LeftArmFT.ini)

\section portsc_sec Ports Created
 
- \e <robot>/<part>/V:i (e.g. /iCub/left_arm/V:i) receives the input data 
  vector.
 
- \e <robot>/<part>/FT:o (e.g. /iCub/left_arm/FT:i) send the output Force/Torque vector
 
\section tested_os_sec Tested OS
Windows.


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
		  Bottle tmp;
		  if(rf.check("rows"))
		  {
			  if(rf.check("row0"))
			  {
				  tmp = rf.findGroup("row0");
				  C.resize(rf.find("rows").asInt(),tmp.size()-1);
			  }
			  else 
			  {
				  fprintf(stderr,"error: first row should be labeled as 'row0'\n Setting number of colums = 1");
				  C.resize(rf.find("rows").asInt(),1);
			  }
		  }
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
		    sprintf(buf, "%d", i);
			  row = string("row") + buf;
			  fprintf(stderr,"%s\n", row.c_str());
			  
			  if(rf.check(row.c_str()))
			  {
				  if(tmp.size()<rf.findGroup(row.c_str()).size())
					  fprintf(stderr,"Warning: too many elements!!!\n One or more datas will not be used...");

				  for(int j=0;j<tmp.size()-1;j++)
				  {
					  if(!rf.findGroup(row.c_str()).get(j+1).isNull())
					  {
							C(i,j)=rf.findGroup(row.c_str()).get(j+1).asDouble();
							fprintf(stderr,"%.2lf\t",C(i,j));
					  }
					  else
					  {
						  C(i,j)=0.0;
						  fprintf(stderr,"%.2lf\t",C(i,j));
						  fprintf(stderr,"Warning:missing element, zero added");
					  }
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
            PortName=PortName+"/left_arm";
		}

		int rows;
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
	rf.setDefaultContext("ftSensorCalibration/conf");
	rf.setDefaultConfigFile("leftArmFTCal.ini");

    rf.configure("ICUB_ROOT", argc, argv);

	if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
		cout << "\t--context   context: where to find the called resource (referred to $ICUB_ROOT\\app: default FTSensorCalibration\\conf)"                << endl;
        cout << "\t--from      from: The name of the file.ini to be used for calibration"          << endl;
        return 0;
    }

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


