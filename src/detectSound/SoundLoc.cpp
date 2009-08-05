//Author: Julio Gomes 19/02/2007
//jgomes@isr.ist.utl.pt

#include <yarp/os/all.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Bottle.h>

#include <iostream>
#include <stdio.h>
//#include <conio.h>
#include <simio.h>


#include "SoundDetector.h"
#include "SoundSourceEstimation.h"



using namespace std;
using namespace yarp::os;


SoundDetector* soundsensor;
SoundSourceEstimation* soundPosEstimation;


#undef main


int main(void) {


	Port port;
	yarp::os::PortWriterBuffer<yarp::os::Bottle> writer;


	writer.attach(port);
    port.open("/soundloc:o");


	soundsensor=new SoundDetector();
	soundPosEstimation=new SoundSourceEstimation();


	double ITD, notch;
	double pan,tilt,horzvel,vertvel,dispvel,disparity;
	double pan_bef,tilt_bef;

	int saliency, last_saliency, countDetections;

	saliency=last_saliency=countDetections=0;


	pan=tilt=horzvel=vertvel=dispvel=saliency=disparity=0;
	pan_bef=tilt_bef=0.0;

	

	
	while (!kbhit())
	{
		Bottle& bot = writer.get();
	
		if(soundsensor->detect_features())
		{
			//printf("\nHeard sound----------------\n");
			

			ITD=soundsensor->get_feature(0);
			notch=soundsensor->get_feature(1);

			soundPosEstimation->control(ITD, notch);
			pan=soundPosEstimation->get_pan();
			tilt=soundPosEstimation->get_tilt();

			printf("\npan->%2.2f tilt->%2.2f\n", pan, tilt);
			//getchar();


			//saliency calculation--------------------------
			countDetections++;

			/*if(countDetections>50)
					countDetections=50;
			if(countDetections<0)
					countDetections=-1;

			saliency=(double)countDetections/50;*/

			if (countDetections>20)
				countDetections=20;
			if(countDetections<0)
					countDetections=-1;

			if (countDetections>0) 
			{
				if (last_saliency==-2)
					saliency=2;
				else
					saliency=1;
			}
			else 
				if (countDetections<0)
					saliency=-2;



			printf("\nsaliency->%d<- - last_saliency->%d<-\n", saliency, last_saliency);

			last_saliency=saliency;	
			//----------------------------------------------


			// ambiguous for some gccs, hope I resolved
			// the ambiguity correctly...
			horzvel=(int)abs((int)(pan_bef-pan));
			vertvel=(int)abs((int)(tilt_bef-tilt));
			pan_bef=pan;
			tilt_bef=tilt;



			// filling the bottle ---------------------------
			bot.clear();		
			bot.addDouble(saliency);
			bot.addDouble(pan);
			bot.addDouble(tilt);
			bot.addDouble(disparity);
			bot.addDouble(horzvel);
			bot.addDouble(vertvel);
			bot.addDouble(dispvel);
			writer.write();	//sending it-------

		}
		else
		{
			pan=0.0;
			tilt=0.0;
			countDetections--;

			if (countDetections>20)
				countDetections=20;
			if(countDetections<0)
					countDetections=-1;

			if (countDetections>0) 
			{
				if (last_saliency==-2)
					saliency=2;
				else
					saliency=1;
			}
			else 
				if (countDetections<0)
					saliency=-2;


			printf("\nsaliency->%d<- - last_saliency->%d<-\n", saliency, last_saliency);

			last_saliency=saliency;	
			// filling the bottle ---------------------------
			bot.clear();		
			bot.addDouble(saliency);
			bot.addDouble(pan);
			bot.addDouble(tilt);
			bot.addDouble(disparity);
			bot.addDouble(horzvel);
			bot.addDouble(vertvel);
			bot.addDouble(dispvel);
			writer.write();	//sending it-------
			printf("no sound\n");
		}
			
	}

	printf("\nexiting press key\n");

	getchar();
	return 1;
}
