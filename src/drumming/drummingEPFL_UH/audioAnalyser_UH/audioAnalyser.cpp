/* 
 * Copyright (C) <2008> RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Hatice Kose-Bagci (University of Hertfordshire)
 * email:   h.kose-bagci@herts.ac.uk
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

 /**
 * @ingroup icub_module
 *
 * \defgroup AudioAnalyser
 *
 *  Records the sound taken by microphones and records it in sound.dat file
 *  Analyses the sound and finds the beats and the durations between the beats
 *  then send these patterns to the drumming module via messaging bottles
 *
 *  Options:
 *  --file <file> Configuration file listing connection ports
 *
 *  \author Hatice Kose-Bagci
 *  \see AudioAnalyser
 */

 
/* 
 *  Records the sound taken by microphones and records it in sound.dat file
 *  Analyses the sound and finds the beats and the durations between the beats
 *  then send these patterns to the drumming module via messaging bottles
 *
 */
#include <stdio.h>
#include <stdlib.h>

#include <time.h>

#include <yarp/sig/Image.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/AudioGrabberInterfaces.h>
#include <yarp/os/Property.h>
#include <yarp/os/all.h>
#include <yarp/sig/Sound.h>
#include <yarp/os/BufferedPort.h>

#include <iostream>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

Port dataPortIn;
int KasparPlaying=1;
FILE *id;
#define AUDIO_WINDOW_SIZE 4
#define AUDIO_MAX_VAL 1000
#define TWOPI 2*3.14

/**
 * Main function which decides whether human or robot is playing, grab the sound, analyse it
 * and find the patterns (number of beats and the durations between them) and send these to
 * the drumming module via messaging bottles
 */
int main(int argc, char * argv[])
{
	Property p;
	p.fromConfigFile("audioAnalyserMicrophone.ini");
	PolyDriver dev1;
	dev1.open(p);
	if (!dev1.isValid())
	{	printf("Audio device not available\n");
		exit(1);
	}
	int i,j,l;
	int k1=1, k2=1;
	int hmask[]={-1, -1, 1, 1};
	int lmask[]={1, 1, -1, -1};
	double window[AUDIO_WINDOW_SIZE],f=0;
	int z=0;
	int count=0, count1=0, count2=0, count3=0,step=0;
	double val;
	double high=-AUDIO_MAX_VAL, low=AUDIO_MAX_VAL, avg=0;
	double vhigh=0, vlow=0;
	int H=0,L=0;
	double max1=-AUDIO_MAX_VAL,max2=-AUDIO_MAX_VAL,max3=-AUDIO_MAX_VAL;
	int maxid1=0,maxid2=0,maxid3=0;
	int th1=10000;
	int th=3000;
	int beatNo=0;
	FILE *outFile;
	int top = 20;
	time_t stime, ftime;
	time_t stime1, ftime1;
	clock_t st, ft, st1;
	int total_time=180;//default value
	int duration=0;
	double dur=0;
	double durArr[100]={0};
	int durArray[20]={0};
	int mode=0;//1 play 0 listen
	int stop=0;
    Port dataPortOut;
	Port dataPortOut1;
	double Threshold1=0.4;
	int Threshold2=2;

		Property cmdLine;
		cmdLine.fromCommand(argc,argv);

		if (!cmdLine.check("file")) {
			printf("Please call with: --file audioAnalysisConfig.txt\n");
			exit(1);
		}
		ConstString fname = cmdLine.find("file").asString();
		Property config;
		config.fromConfigFile(fname.c_str());

		if (!config.check("PORTS")) {
			printf("Config file needs section [PORTS] with port names defined\n");
			exit(1);
		}

		// input port of
		ConstString ackInputPortName = config.findGroup("PORTS").check("ack_in", Value("/uh/Audio/ack:i")).asString();
		dataPortIn.open(ackInputPortName);

		//output port for
		ConstString beatOutputPortName = config.findGroup("PORTS").check("beat_out",Value("/uh/Audio/beat:o")).asString();
		dataPortOut.open(beatOutputPortName);

		//output port for
		ConstString durationOutputPortName = config.findGroup("PORTS").check("duration_out",Value("/uh/Audio/duration:o")).asString();
		dataPortOut1.open(durationOutputPortName);


		//input port of 
		ConstString ackOutputPortName = config.findGroup("PORTS").check("ack_out", Value("/uh/Drum/ack:o")).asString();

		//input port of
		ConstString beatInputPortName = config.findGroup("PORTS").check("beat_in",Value("/right_arm/score/in")).asString();

		//input port of
		ConstString durationInputPortName = config.findGroup("PORTS").check("duration_in",Value("/interactive/in")).asString();


		if (!config.findGroup("GENERAL").check("game_time")) {
			fprintf(stderr,"Configuration file expects game_time\n");
			exit(-1);
		}
		total_time = config.findGroup("GENERAL").find("game_time").asInt();


	outFile = fopen("sound.dat","w");
	while( !Network::connect(beatOutputPortName.c_str(),beatInputPortName.c_str(), "tcp", false)){
		 printf("audioAnalyser connecting to %s ...\n",beatInputPortName.c_str());
		 Time::delay(0.5);
	}
	while( !Network::connect(durationOutputPortName.c_str(),durationInputPortName.c_str(), "tcp", false)){
		 printf("audioAnalyser connecting to %s ...\n",durationInputPortName.c_str());
		 Time::delay(0.5);
	}

	IAudioGrabberSound	*grabber;
	dev1.view(grabber);
	printf("Audio device is OK\n");
	Sound sound;

	if (grabber!=NULL)
	{
		printf("grabber not null\n");
		time(&stime);
		time(&ftime);
		ftime1=ftime;
		st1=clock();

		while(difftime(ftime, stime)<total_time)
		{
			ftime1=ftime;
			printf("START PLAYING...\n");
			while((stop==0)&&(difftime(ftime, stime)<total_time))
			{
				
				if (grabber->getSound(sound))
				{
					for (i=0;i<sound.getSamples();i++)
					{
						for(j = 0; j < sound.getChannels(); j++)
						{
						   val=(double)sound.get(i,j)/32768;
						   fprintf(outFile, "%1.6f ", val);
						   window[z]=val;
						   // a filter based algorithm is used to find the peaks in the sound wave
						   // online, and filters the noise to find the beats.,
						   if (z>=AUDIO_WINDOW_SIZE-1)
						   {
								f=0;
								for (l=0;l<AUDIO_WINDOW_SIZE;l++)
									f= f + window[l]*hmask[l];
								if (f>Threshold1)
								{
									count++;
									if ((high<=f) && (H==0) && (count1-count3>th1))
									{
										high=f;
										avg=window[1]+window[2];
										low=AUDIO_MAX_VAL;
										H=1;
										L=0;
				    					vhigh=window[1];
										count2=2;
										count3=count1;
										time(&stime1);
										ft=clock();
										if (beatNo!=0)
										{
											dur = (double)(ft - st) / CLOCKS_PER_SEC;
											durArr[beatNo]=dur;
										}
									}
								}
								if (H==1)
								{
									avg+=val;
									count2++;
								}
								f=0;
								for (l=0;l<AUDIO_WINDOW_SIZE;l++)
									f= f + window[l]*lmask[l];
								 if (f>Threshold1)
								 {
									count++;
									if (low>=f && L==0 && H==1)
									{
										low=f;
										avg=avg-window[2]-window[3];
										count2=count2-2;//window[2], window[3]
										avg=avg/count2;
										vlow=window[1];
										time(&ftime1);
										st=clock();
										beatNo++;
										high=-AUDIO_MAX_VAL;
										low=AUDIO_MAX_VAL;
										avg=0;
										H=0;
										L=0;

									}
								 }
								 window[0]= window[1];
						 		 window[1]= window[2];
								 window[2]= window[3];

						   }
						   else
								z++;
						   count1++;
						}
					}

				}
				else
				{
					printf("*** Failed \n");
					break;
				}
				time(&ftime);
				duration=difftime(ftime, ftime1);
				if (duration>Threshold2)
				{
					stop=1;
					printf("STOP PLAYING...\n");
				}
			}
			if (difftime(ftime, stime)>total_time)
				stop=1;
		/*	beatNo=3;
			durArr[1]=1.0;
			durArr[2]=1.0;
			*/

			if (stop==1) 
			{
				if (beatNo!=0){
				Bottle bot;
				//beatNo=3;
				//if beatNo>16 
				bot.addInt(0);//duration
				for (i=0; i<beatNo; i++)
				{
					bot.addInt(1);//duration
				}
				for (i=beatNo; i<15; i++)
				{
					bot.addInt(0);//duration
				}
				dataPortOut.write(bot);

				printf("send data %d beats...to iCub \n", beatNo);
				//send number of beats
				Bottle bot1;
				for (i=0; i<16; i++)
				{
					bot1.addDouble(0.5);//duration
				}
				/*
				for (i=1; i<beatNo; i++)
				{
					bot1.addDouble(1/(TWOPI*durArr[i]));//duration
				}
				*/
				for (i=beatNo; i<=16; i++)
				{
					bot1.addInt(0);//duration
				}
				dataPortOut1.write(bot1);
			//	return 0;//****************************************************/
				}
				else
				{printf("no music...\n");
				}
			/*	Bottle bot;
				bot.addInt(beatNo);
				for (i=1; i<beatNo; i++)
				{
					duration = (int)(durArr[i]*AUDIO_MAX_VAL);
					bot.addInt(duration);//duration
				}
				*/
				// send the message
				//dataPortOut.write(bot);
				beatNo=0;
				stop=0;
				
			}
			
		}
	} else
		 printf("grabber null\n");
		Bottle bot;
				for (i=0; i<16; i++)
				{
					bot.addInt(0);//duration
				}
				dataPortOut.write(bot);

       
		printf("AudioAnalyser application terminates...\n");
 		dataPortOut.close();
		fclose(outFile);
		return 0;
}



