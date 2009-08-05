/*
 *  Takes the patterns via messaging bottles from the audioAnalyser and set the joints of
 *  the right arm ready, then move the right forearm as much as the pattern to demonstrate
 *  the drumming action, then send an acknowledgement to the audioAnalyser module to say that
 *  the robot is done with the drumming and it is human's turn. 
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
 *  $Id: drummer.cpp,v 1.1 2008/07/29 15:10:13 hatice Exp $
 */

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
/*
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
*/
#include <yarp/os/all.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp;

const int COMMANDED_JOINTS=4;
double ARM_VELOCITY[4]={30.0, 30.0, 30.0, 60.0};
double ARM_POSITION1[4]={5.0, 11.0, -7.0, 27.0};
double ARM_POSITION2[4]={5.0, 11.0, -7.0, 70.0};//110
double ARM_START_POSITION[4]={5.0, 11.0, -7.0, 50.0};

const int COMMANDED_JOINTS_LEFT=7;
double ARM_VELOCITY_LEFT[COMMANDED_JOINTS_LEFT]={15.0, 25.0, 25.0, 25.0,25.0, 25.0, 25.0};
double ARM_POSITION1_LEFT[COMMANDED_JOINTS_LEFT]={-20.0, 80.0, -40.0, 101.0, 0.0, 0.0, -5.0};
double ARM_POSITION2_LEFT[COMMANDED_JOINTS_LEFT]={-20.0, 80.0, -40.0, 70.0, 0.0, 0.0, 5.0};//110
double ARM_START_POSITION_LEFT[COMMANDED_JOINTS_LEFT]={-20.0, 80.0, -40.0, 80.0, 0.0, 0.0, 5.0};
double ARM_STOP_POSITION_LEFT[COMMANDED_JOINTS_LEFT]={-20.0, 80.0, -40.0, 90.0, 0.0, 0.0, 0.0};

int ini=1;
time_t stime, ftime;
clock_t st, ft, st1;
int step=0;
Port emotion;
void happy_face(){
	Bottle bot;
	//happy face expression
	bot.clear();
    bot.addVocab(Vocab::encode("set"));
    bot.addVocab(Vocab::encode("all"));
	bot.addVocab(Vocab::encode("hap"));
	printf("Sent message: %s\n", bot.toString().c_str());
	
	emotion.write(bot);
    Time::delay(0.3);
	//blink
	bot.clear();
    bot.addVocab(Vocab::encode("set"));
    bot.addVocab(Vocab::encode("raw"));
	bot.addVocab(Vocab::encode("S10"));
	printf("Sent message: %s\n", bot.toString().c_str());
	emotion.write(bot);
	Time::delay(0.7);
	bot.clear();
    bot.addVocab(Vocab::encode("set"));
    bot.addVocab(Vocab::encode("raw"));
	bot.addVocab(Vocab::encode("S50"));
	printf("Sent message: %s\n", bot.toString().c_str());
	emotion.write(bot);
	Time::delay(0.3);
	}
void sad_face(){
	Bottle bot;
	//sad face expression
	bot.clear();
    bot.addVocab(Vocab::encode("set"));
    bot.addVocab(Vocab::encode("all"));
	bot.addVocab(Vocab::encode("sad"));
	printf("Sent message: %s\n", bot.toString().c_str());
	emotion.write(bot);
	Time::delay(0.3);
	}
void wave(){
	//left arm waves
	Property armOptions_LEFT;
    armOptions_LEFT.put("device", "remote_controlboard");
    armOptions_LEFT.put("local", "/icub/left_arm/waiver_UH/");   //local port names
 	armOptions_LEFT.put("remote", "/icub/left_arm");         //where we connect to
	int beat=0;
    // create a device
    PolyDriver armdd_LEFT(armOptions_LEFT);
    if (!armdd_LEFT.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        Network::fini();
    }
    IPositionControl *pos_LEFT;
    IEncoders *encs_LEFT;

    bool ok;
    ok = armdd_LEFT.view(pos_LEFT);
    ok = ok && armdd_LEFT.view(encs_LEFT);

    if (!ok) {
        printf("Problems acquiring interfaces\n");
        Network::fini();
    }
    int nj=0;
    pos_LEFT->getAxes(&nj);
    double *encoders_LEFT=new double [nj];
	// moving to start position
    fprintf(stderr, "Going to move to start position");
    int k=0;
    bool done=false;
    double timeout=0.0;
    for(k=0; k<COMMANDED_JOINTS_LEFT;k++)
    {
        pos_LEFT->setRefSpeed(k,ARM_VELOCITY_LEFT[k]);
        pos_LEFT->positionMove(k,ARM_START_POSITION_LEFT[k]);
        done=false;
        timeout=0;
		 while(!done)
        {
			encs_LEFT->getEncoders(encoders_LEFT);
			pos_LEFT->checkMotionDone(&done);
			if (timeout>0.5)//originally 3
			   done=true;
			Time::delay(0.2);
			timeout+=0.2;
        }
    }
 	/*******************WAVE*************************/
	beat=6;
			int times=1;
			while (times<=(2*beat))
			{
				//for waving action every  two
				//move should be repeated for every wave, 1 time left,
				//and 1 time right, which makes 2xmoves
				times++;
				if (times%2){
					//k=2;//just the arm yaw move
					k=3;
					pos_LEFT->positionMove(k, ARM_POSITION1_LEFT[k]);
				}
				else{
					k=3;//k=2;
					pos_LEFT->positionMove(k, ARM_POSITION2_LEFT[k]);
				}
				// wait until motion is completed
				done=false;
				timeout=0;
				while(!done)
				{
					encs_LEFT->getEncoders(encoders_LEFT);
					pos_LEFT->checkMotionDone(&done);
					if (timeout>0.3)//originally 3
						done=true;
					Time::delay(0.1);
					timeout+=0.1;
				}
				fprintf(stderr, "done\n");
			  }

			for(k=0; k<COMMANDED_JOINTS_LEFT;k++)
			{
				pos_LEFT->positionMove(k,ARM_STOP_POSITION_LEFT[k]);
			}
  armdd_LEFT.close();
  delete [] encoders_LEFT;
}
/**
 * Takes the patterns via messaging bottles from the audioAnalyser and set the joints of
 * the right arm ready, then move the right forearm as much as the pattern to demonstrate
 * the drumming action, then send an acknowledgement to the audioAnalyser module to say that
 * the robot is done with the drummning and it is human's turn.
 */
int main(int argc, char *argv[]) {

	int beat=0, duration;
	int durArray[100];
	int count = 0;
	int total_time=100;//60;//default play time
	int i,top=1;
	int mode=1;//1 play 0 listen
	double dtime, timeDiff;
	int finito=0;
	clock_t st_kaspar, ft_kaspar;
	//double total_drumming_time=0;

	Network::init();
	Port input;
	input.open("/uh/Drum/pattern:i");
	BufferedPort<Bottle> output1;
	output1.open("/uh/Drum/ack:o");
	emotion.open("/rpc/test");
    Network::connect("/rpc/test", "/icub/face/emotions/in");

	Property armOptions;
    armOptions.put("device", "remote_controlboard");
    armOptions.put("local", "/icub/right_arm/UHdrummer/");   //local port names
    //armOptions.put("remote", "/icubSim/right_arm");         //where we connect to
	armOptions.put("remote", "/icub/right_arm");         //where we connect to

    // create a device
    PolyDriver armdd(armOptions);
    if (!armdd.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        Network::fini();
        return 0;
    }

    IPositionControl *pos;
    IEncoders *encs;

    bool ok;
    ok = armdd.view(pos);
    ok = ok && armdd.view(encs);

    if (!ok) {
        printf("Problems acquiring interfaces\n");
        Network::fini();
        return 0;
    }

    int nj=0;
    pos->getAxes(&nj);
    double *encoders=new double [nj];

	// moving to start position
    fprintf(stderr, "Going to move to start position");
    int k=0;
    bool done=false;
    double timeout=0.0;
    for(k=0; k<COMMANDED_JOINTS;k++)
    {
        pos->setRefSpeed(k,ARM_VELOCITY[k]);
        pos->positionMove(k,ARM_START_POSITION[k]);

        done=false;
        timeout=0;
		 while(!done)
        {
            encs->getEncoders(encoders);
            fprintf(stderr, "Encoders: %.2f %.2f %.2f %.2f\n",
                    encoders[0], encoders[1], encoders[2], encoders[3]);
            pos->checkMotionDone(&done);

            if (timeout>0.5)//originally 3
                done=true;
			
            Time::delay(0.2);
            timeout+=0.2;
        }
    }
    fprintf(stderr, "done\n");

		/********************************/
	time(&stime);
	time(&ftime);
	//st=clock();
    //drum_ready1();
	//while (1)
	while((difftime(ftime, stime)<total_time)&&(finito==0))
	{

		Bottle bot;
		input.read(bot);
		beat = bot.get(0).asVocab();
		for (i=1; i<beat; i++){
			duration = bot.get(i).asVocab();
			durArray[i]=duration;
		}
		printf("get beat:%d ...\n", beat);
		if (beat==-1)
			finito=1;
		else{
	//	beat=2;
			if (beat!=0){
			happy_face();
			int times=1;
			while (times<=(2*beat))
			{
				//for drumming action every  two
				//move should be repeated for every beat, beat times
				//up and beat times down, which makes 2xbeat moves
				fprintf(stderr, "Moving%d\n", times);
				times++;
				if (times%2){
					//k=2;//just the arm yaw move
					k=3;
					pos->positionMove(k, ARM_POSITION1[k]);
				}
				else{
					k=3;//k=2;
					pos->positionMove(k, ARM_POSITION2[k]);
				}
				// wait until motion is completed
				done=false;
				timeout=0;
				while(!done)
				{
					encs->getEncoders(encoders);
					fprintf(stderr, "Encoders: %.2f %.2f %.2f %.2f\n",
							encoders[0], encoders[1], encoders[2], encoders[3]);
					pos->checkMotionDone(&done);
					if (timeout>0.3)//originally 3
						done=true;
			
					Time::delay(0.1);
					timeout+=0.1;
				}
				fprintf(stderr, "done\n");
			  }
/*
			for(k=0; k<COMMANDED_JOINTS;k++)
			{
				pos->positionMove(k,ARM_START_POSITION[k]);
			}
			*/
				printf("beat %d times...\n", beat);
		}//if beatno!=0
			else{
				printf("zero BEATS...\n");
				sad_face();

			}
		
				Bottle &bot1 = output1.prepare();
				bot1.clear();
				bot1.addInt(0);
		    	output1.writeStrict();
				//printf("sending...%s...\n", bot1.);
				// wait a while
		count++;
		printf("COUNT = %d  *****************************",count);
		time(&ftime);
		}
    }//ACCCCCCCCCCCCCC
  printf("OLEEEEYYYYYYYYYYYYY\n");
  sad_face();
  wave();

  armdd.close();
  delete [] encoders;
 input.close();
 output1.close();
// output.close();
 Network::fini();
 return 0;

}





