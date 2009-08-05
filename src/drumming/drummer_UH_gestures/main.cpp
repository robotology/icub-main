/*
 *  Records the sound taken by microphones and records it in sound.dat file
 *  Analyses the sound and finds the beats and the durations between the beats
 *  then send these patterns to the drumming module via messaging bottles
 *
 */

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
 *
 *  $Id: main.cpp,v 1.1 2008/07/29 15:10:13 hatice Exp $
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

/**
 * Thread for reading from a port
 */
class DataThread : public yarp::os::Thread
{
	public:
		DataThread() {
		}

		~DataThread() {
		}

		void run() {
			while (!isStopping()) {
				if (dataPortIn.read(b)){
					KasparPlaying = b.get(0).asVocab();
					printf("iCUB DONE...\n");
				}
			}
		}

		void onStop() {}
		void beforeStart() {}
		void afterStart() {}
		bool threadInit() {
			return true;
		}
		void threadRelease() {}

	private:
		Bottle b;
};
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
		ConstString patternOutputPortName = config.findGroup("PORTS").check("pattern_out",Value("/uh/Audio/pattern:o")).asString();
		dataPortOut.open(patternOutputPortName);

		// input port of
		ConstString ackOutputPortName = config.findGroup("PORTS").check("ack_out", Value("/uh/Drum/ack:o")).asString();

		//output port for
		ConstString patternInputPortName = config.findGroup("PORTS").check("pattern_in",Value("/uh/Drum/pattern:i")).asString();



		if (!config.findGroup("GENERAL").check("game_time")) {
			fprintf(stderr,"Configuration file expects game_time\n");
			exit(-1);
		}
		total_time = config.findGroup("GENERAL").find("game_time").asInt();


	outFile = fopen("sound.dat","w");
	while( !Network::connect(patternOutputPortName.c_str(),patternInputPortName.c_str(), "tcp", false)){
		 printf("audioAnalyser connecting to %s ...\n",patternInputPortName.c_str());
		 Time::delay(0.5);
	}
	DataThread *dataThread = new DataThread();
	dataThread->start();
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
			if (mode)
			printf("ROBOT PLAYING...\n");
		else
			printf("HUMAN PLAYING...\n");
			ftime1=ftime;
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
				if (mode==0 && duration>Threshold2)
				{
					stop=1;
				}
				if(mode==1 && KasparPlaying==0){
					stop=1;
				}
			}
			if (difftime(ftime, stime)>total_time)
				stop=1;
			if (stop==1 && mode==0)
			{
				Bottle bot;
				bot.addInt(beatNo);
				for (i=1; i<beatNo; i++)
				{
					duration = (int)(durArr[i]*AUDIO_MAX_VAL);
					bot.addInt(duration);//duration
				}
				// send the message
				printf("send beats... %d ...\n", beatNo);
				dataPortOut.write(bot);
				beatNo=0;
				mode=!mode;
				stop=0;
				step++;
			}
			else if(mode==1 && stop==1)
			{
				stop=0;
				mode=!mode;
				KasparPlaying=1;
				beatNo=0;
				printf("Robot finished playing...\n");
			}
		}
	} else
        printf("grabber null\n");
		printf("AudioAnalyser application terminates...\n");
 		dataThread->stop();
  	    dataPortIn.close();
		dataPortOut.close();
		fclose(outFile);
		return 0;
}



