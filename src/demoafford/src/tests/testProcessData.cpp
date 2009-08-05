#include <yarp/os/all.h>

#include <stdio.h>
#include <conio.h>
#include <iostream>

#include "processobjdata.h"

using namespace std;
using namespace yarp::os;


enum processdatastates {
   WAITFORACTIONINIT,
   ACTIONRUNNING,
   ACTIONENDED
} state;

int main() 
{
double data[256];
char colors[16][16]={"orange","orange","yellow","yellow","green","green","green","blue","blue","blue","indigo","indigo","violet","violet","red","red"};
char shapes[2][16]={"ball","box"};
int shape = 0;
int color = 0;

state = WAITFORACTIONINIT;

processobjdata processdata;
int retaddpoint;

Network::init();

	BufferedPort<Bottle> port_obj;
	port_obj.open("/processdata");
	//Network::connect("/camshiftplus/all/o","/processdata");

	BufferedPort<Bottle> port_sync;
	port_sync.open("/processdatasyncvision");
	//Network::connect("/processdatasyncvision","/camshiftplus/roi/i");

	//open port
	while(1)
	{
	//read from port
		
	yarp::os::Time::delay(0.01);
	Bottle *input_obj=port_obj.read(false);
	if (input_obj!=NULL)
	{
		printf(".\n");
		//printf("size %d --- %s\n",input_obj->size(),input_obj->toString().c_str());

		for(int cnt = 0;cnt<input_obj->size();cnt++)
			data[cnt] = input_obj->get(cnt).asDouble();
	}
	else
	{
		yarp::os::Time::delay(0.005);
		continue;
	}
	
	
	double *hist;
	hist = processdata.getcolorhist((double*)data);
	color = processdata.classifycolor( hist );
	//processdata.printhist( hist );

	hist = processdata.getshapehist(data);
	shape = processdata.classifyshape( hist );
	processdata.printhist( hist );

	retaddpoint = processdata.addDataPoint((double*)data);

	switch (state)
	{
	case WAITFORACTIONINIT:
		if (processdata.detectActionInit()==1)
		{
			printf("action init\n");
			processdata.restartDataAcquisition();
			state = ACTIONRUNNING;
		}
		break;

	case   ACTIONRUNNING:
		if(processdata.bufferfull())
			state = ACTIONENDED;
		break;

	case	ACTIONENDED:
		double effects[32];
		unsigned char effectclassification[32];

		processdata.computeEffects((double*)effects);
		processdata.classifyeffects((double*)effects, (unsigned char*)effectclassification);

		printf("Effects %g %d %g %d \n",effects[0],effectclassification[0],
										effects[1],effectclassification[1]);

		processdata.restartDataAcquisition();
		state = WAITFORACTIONINIT;
		getchar();
		{
			Bottle& output = port_sync.prepare();
			output.clear();
			output.addInt(155);
			output.addInt(115);
			output.addInt(165);
			output.addInt(125);
    		cout << "writing " << output.toString().c_str() << " " << output.size() << endl;
			port_sync.write();
		}
		break;

	default:
		printf("error state\n");
		getchar();
	}
	

	

	printf("I'm looking at a %s %s\n",colors[color],shapes[shape]);


	}
	
	port_obj.close();
	port_sync.close();

	return 0;
}