
#include <iomanip>

#include <iostream>
#include <iomanip>
#include <string>

#include "PredSmoothP.h"
using namespace std ;
#define FPS 15
class ControlThread: public RateThread
{
    PolyDriver dd;
    IVelocityControl *ivel;
    IEncoders        *iencs;
    BufferedPort<Bottle>  velPort;
	Vector encoders;
    Vector commands;
	Vector val;
	int div;
    int count;
	double output;
	double prevOutput;


	

public:
    ControlThread(int period):RateThread(period){}

    bool threadInit()
    {
        //initialize here variables
        printf("EyeVelocityThread:starting\n");
        
        Property options;
        options.put("device", "remote_controlboard");
        options.put("local", "/local/head");
        
        //substitute icubSim with icub for use with the real robot
        options.put("remote", "/icub/head");
        
        dd.open(options);
		output=0;
		prevOutput=0;
		val.resize(2);
		val[0]=0.0;
		val[1]=0;
		div=0;
        dd.view(iencs);
        dd.view(ivel);
		//dd.view(ipos);

        if ( (!iencs) || (!ivel) )
            return false;
		
		//Network::init();

		
		velPort.open("/EyeControlVel");
		Network::connect("/OutputRLS","/EyeControlVel");
		
		int joints;
   
        iencs->getAxes(&joints);
    
        encoders.resize(joints);
        commands.resize(joints);

        commands=10000;
        ivel->setRefAccelerations(commands.data());

        count=0;
        return true;
    }

    void threadRelease()
    {
        printf("ControlThread:stopping the robot\n");
        
        ivel->stop();

        dd.close();

        printf("Done, \n");
    }

    void run()
    {
		
		if(val[1]==0){
		prevOutput=output;
		
		Bottle *velBott=velPort.read();       
		
		output=velBott->get(0).asDouble();
		cout<< "output"<< output<<"\n"<<endl;
		div=(100 / FPS)-2;
		val[0]=abs(output-prevOutput)/div;

		}else{
			
			if(val[1]==div-1)
				val[1]=-1;
			}
		
		if(output<0)
			prevOutput=prevOutput- val[0];
		else
			prevOutput=prevOutput+ val[0];
		val[1]++;
		ivel->velocityMove(4,prevOutput);
	}
		
		
			  
    
};

/************************************************************************/
PredSmoothP::PredSmoothP(PolyDriver *_drvHead,
                             const string &_localName, 
                             unsigned int _period,unsigned int _joint) :
                             RateThread(_period),          drvHead(_drvHead),
                             localName(_localName),     joint(_joint),             
                             period(_period), Ts(_period/1000.0)

	{  
        // create interfaces
        bool ok;
        
        ok&=drvHead->view(encHead);

        if (!ok)
            cout << "Problems acquiring interfaces!" << endl;

        // read number of joints
        
        encHead->getAxes(&nJointsHead);


    }
   



/************************************************************************/
bool PredSmoothP::threadInit()
{
   // port_pixel=new BufferedPort<Bottle>();
	//string n=localName+"/pixel:i";
	port_stateTarget=new BufferedPort<Bottle>();
	port_velocityJoint=new BufferedPort<Bottle>();
	
	string velString=localName+"/velJoint:i";
	string slipString=localName+"/stateTarget:i";
    //port_pixel->open(n.c_str());
	port_stateTarget->open(slipString.c_str());
	port_velocityJoint->open(velString.c_str());
    cout << "Starting Predictive Smooth Pursuit at " << period << " ms" << endl;

    return true;
}


/************************************************************************/
void PredSmoothP::afterStart(bool s)
{
    if (s)
        cout << "Predictive Smooth Pursuit started successfully" << endl;
    else
        cout << "Predictive Smooth Pursuit did not start" << endl;
}


/************************************************************************/
void PredSmoothP::run()
{
  

		Vector v(3);
		
		
		

		
	




}


/************************************************************************/
void PredSmoothP::threadRelease()
{
	/*port_pixel->interrupt();
    port_pixel->close();
    delete port_pixel;*/
  
}





