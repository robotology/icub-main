//Author: Julio Gomes 14/02/2006
//jgomes@isr.ist.utl.pt

#include <yarp/os/all.h>
#include <yarp/os/BufferedPort.h>

#include <iostream>
#include <stdio.h>
//#include <conio.h>  // not portable!
#include <simio.h>

//#include <windows.h>  // not portable!


using namespace std;
using namespace yarp::os;


#define Pi 3.1415927

#undef main



int main(void) {

	float panerr,tilterr,last_pan,last_tilt;

/*
	IN ports:	/attcontrol/facetracker/in
				/attcontrol/soundloc/in
				/attcontrol/camshift/in

				/attcontrol/aux

	OUT ports:	/attcontrol/stimpos/out
				/attcontrol/stimvel/out

*/

	//out ports---------------------------------------------
	Port port_out;
	yarp::os::PortWriterBuffer<yarp::os::Bottle> writerPos;
	Port port_out1;
	yarp::os::PortWriterBuffer<yarp::os::Bottle> writerVel;
	//------------------------------------------------------


	//in ports-------------------------------------------
	BufferedPort<Bottle> port;
	BufferedPort<Bottle> port1;
	BufferedPort<Bottle> port2;

	BufferedPort<Bottle> portAux;
	//--------------------------------------------------------

	Network::init();


	writerPos.attach(port_out);
    port_out.open("/attcontrol/stimpos/out");
	writerVel.attach(port_out1);
    port_out1.open("/attcontrol/stimvel/out");


	port.setStrict(false);
	port1.setStrict(false);
	port2.setStrict(false);
	portAux.setStrict(false);

    port.open("/attcontrol/facetracker/in");
	port1.open("/attcontrol/soundloc/in");
	port2.open("/attcontrol/camshift/in");
	portAux.open("/attcontrol/aux");
   
	yarp::os::Network::connect("/facetrack:o","/attcontrol/facetracker/in","tcp");
	yarp::os::Network::connect("/soundloc:o","/attcontrol/soundloc/in","tcp");
	yarp::os::Network::connect("/controlboard/state:o","/attcontrol/aux","tcp");
	printf("connected port\n\n");

	double x, y, saliencyFace, saliencySound, panSound, tiltSound;

	int SoundStim, FaceStim, keepMemory;

	SoundStim=FaceStim=1;
	keepMemory=7;

	printf("\n\n");
	while(!kbhit()) 
	{

		//--------read from detector: "facetracker" -----------
		Bottle *input=port.read();

		saliencyFace=input->get(0).asDouble();
		x=input->get(1).asDouble();
		y=input->get(2).asDouble();


		//printf("\n x:->%f2.2<-   y:->%f2.2<-\n",x,y);
		//printf("\nsaliencyFace->%2.2f<-", saliencyFace);

		
		panerr = (float)(-0.1 * x/160);
		tilterr = (float)(-0.1 * (float)y/120);
		//----------------------------------------------------------


		//--------read from detector: "sound" -----------------
		Bottle *inputSound=port1.read();

		saliencySound=inputSound->get(0).asDouble();
		panSound=inputSound->get(1).asDouble();
		tiltSound=inputSound->get(2).asDouble();


		//printf("\n Soundpan:->%f2.2<-   Soundtilt:->%f2.2<-\n",panSound,tiltSound);
		//printf("\nsaliencySound->%2.2f<-", saliencySound);		
		//-------------------------------------------------------------
		
		
		printf("saliencyFace->%2.2f<- saliencySound->%2.2f<- faceStim->%d\r", saliencyFace, saliencySound, FaceStim);






//		if (saliencySound>0 || saliencyFace >0)
//		{

			if (saliencySound==2 && FaceStim!=0) {

				printf("\nIniciou Som\n");

				FaceStim=0;

				Bottle *encStateAux=portAux.read();

				last_tilt=encStateAux->get(0).asDouble()*Pi/180;
				last_pan=encStateAux->get(2).asDouble()*Pi/180;

				printf("\nread encoders\n");
					
				printf("tilt->%2.2f pan->%2.2f (degrees)\n", last_tilt*180/Pi, last_pan*180/Pi);
				
				//getchar();

				printf("\n sending actuation \n");


			
				Bottle& botPos = writerPos.get();
				// filling the bottle ---------------------------
				botPos.clear();		
				botPos.addDouble((double)panSound);
				botPos.addDouble((double)tiltSound);					
				writerPos.write();	//sending it---------------		
				
				

				//Sleep(4000);
				Time::delay(4);
				printf("\nsai da sacada\n");


			}
			else {

				if (!FaceStim)
				{

					printf("\nentra na recuperação\n");
					FaceStim=1;

					printf("\n para manda posição anterior\n");
		
					printf("tilt->%2.2f pan->%2.2f (degrees)\n", last_tilt*180/Pi, last_pan*180/Pi);

					Bottle& botPos = writerPos.get();
					// filling the bottle ---------------------------
					botPos.clear();		
					botPos.addDouble((double)last_pan);
					botPos.addDouble((double)last_tilt);		
					writerPos.write();	//sending it---------------


					//Sleep(4000);
					Time::delay(4);


				}
				else 
				{

					printf("\nfaz em velocidade\n");

					Bottle& botVel = writerVel.get();
					// filling the bottle ---------------------------
					botVel.clear();		
					botVel.addDouble((double)panerr);
					botVel.addDouble((double)tilterr);		
					writerVel.write();	//sending it---------------
				
				
				}

			}

	//	}

		


	}





	yarp::os::Network::disconnect("/controlboard/state:o","/attcontrol/aux","tcp");
	yarp::os::Network::disconnect("/facetrack:o","/attcontrol/facetracker/in","tcp");
	yarp::os::Network::disconnect("/soundloc:o","/attcontrol/soundloc/in","tcp");
	printf("\n disconnected port\n");
    Network::fini();

	printf("\nquiting - press key\n");
	getchar();
	
	return 1;
}


