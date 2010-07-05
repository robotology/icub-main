/**
* @ingroup icub_module
*
* \defgroup tactileGrasp
*
*
* Grasp objects with the help of the touch sensors in the fingertips.
*
* \author Alexander Schmitz
*
* Copyleft (C) 2010 RobotCub Consortium
*
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
**/

#include <yarp/os/BufferedPort.h>
// #include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
// #include <yarp/os/RateThread.h>
#include <yarp/sig/Vector.h>
// #include <yarp/sig/Matrix.h>
// #include <yarp/math/Math.h>
// #include <yarp/os/Stamp.h>
#include <yarp/dev/PolyDriver.h> 
#include <yarp/dev/ControlBoardInterfaces.h> 
#include <yarp/os/Bottle.h>

#include <string>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;


const double M_PI=3.145;

const double TOUCH_TRESHHOLD=7; //2
const double speedv = 26; 
const double speedo	= -26;
const int STORE_AVG = 10;
const int REF_SPEED_ACC = 100;
const int CAL_TIME = 500;
const int SKIN_DIM = 192;
const int MAX_SKIN = 255;

int main(int argc, char *argv[]) 
{
    Network yarp;

    Property params;
	params.put("robot", "icub");
	params.put("part", "left_arm");
    params.put("device", "remote_controlboard");
    params.put("local", "/icub/armcontrol/client");   //local port names
    params.put("remote", "/icub/left_arm");

	//Network::init();
	Time::turboBoost();

    // create a device
    PolyDriver robotDevice(params);
    if (!robotDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

	IPositionControl *pos;
	IVelocityControl *vel;
    IEncoders *encs;

    bool ok;
    ok = robotDevice.view(pos);
	ok = ok && robotDevice.view(vel);
    ok = ok && robotDevice.view(encs);

	if (!ok) {
        printf("Problems acquiring interfaces\n");
        return 0;
    }

    int nj=0;
    pos->getAxes(&nj); //the number of axes
    Vector encoders;
    Vector command;
    Vector tmp;
    encoders.resize(nj);
    command.resize(nj);
	tmp.resize(nj);
    
    
    int i;
    for (i = 0; i < nj; i++) {
         tmp[i] = REF_SPEED_ACC;
    }
    pos->setRefAccelerations(tmp.data());

    for (i = 0; i < nj; i++) {
        tmp[i] = REF_SPEED_ACC;
        pos->setRefSpeed(i, tmp[i]);
		vel->setRefAcceleration(i, tmp[i]);
    }

	pos->positionMove(11,0);
	pos->positionMove(12,0);
	pos->positionMove(13,0);
	pos->positionMove(14,0);
	pos->positionMove(15,0);
	Time::delay(2);
	
	BufferedPort<Bottle> skin_port;							//BufferedPort<Bottle> port;
    skin_port.open("/receiver");							//port.open("/summer");
	Network::connect("/icub/skin/lefthand","/receiver");
	
	//Bottle bot;
	//input.read(bot);
	//cout << "got " << bot.toString().c_str()) << endl;
	   
	Bottle *input = skin_port.read();

	//collect skin data for some time, and compute the 95% percentile
	int skin_empty[48][MAX_SKIN+1];
	for (int i=0; i<48; i++) {
		for (int j=0; j<=MAX_SKIN; j++) {
			skin_empty[i][j] = 0 ;
		}
	}
	Vector skin_start;
	skin_start.resize(input->size());
	//collect data
	for (int i=0; i<CAL_TIME; i++) {
		input = skin_port.read();
		if (input!=NULL) {
			Vector skin_values;
			skin_values.resize(48);
			for (int j=0; j<48; j++) {
                skin_values[j] = input->get(j).asDouble();
				skin_empty[j][int(skin_values[j])]++;
			}
		}
		Time::delay(0.01);
	}
	//get percentile
	for (int i=0; i<48; i++) {
		//cumulative values
		for (int j=(MAX_SKIN-1); j>=0; j--) {
			skin_empty[i][j] += skin_empty[i][j+1] ;
		}
		//when do we cross the treshhold?
		for (int j=MAX_SKIN; j>=0; j--) {
			if (skin_empty[i][j] > (CAL_TIME*0.95)) {
				skin_start[i] = j;
				j = 0;
			}
		}
	}
	//printf
	for (int i=0; i<48; i++) {
		fprintf(stderr,"%d ", int(skin_start[i]));
	}
	fprintf(stderr,"\n");

	//old version of the calibration
	//Vector skin_start;
	//skin_start.resize(input->size());
	//	for (int i=0; i<input->size(); i++) {
	//		skin_start[i] = input->get(i).asDouble();
	//	}
	
	//I'll use these variables to do the moving average
	int skin_storage[SKIN_DIM][STORE_AVG];
	for (int i=0; i<input->size(); i++) {
		for (int j=0; j<STORE_AVG; j++) {
			skin_storage[i][j] = MAX_SKIN;
		}
	}
	int skin_storage_count = 0;
	    
    while (true) {
        input = skin_port.read();
		encs->getEncoders(encoders.data());

		bool detect_touch_index = false;
		bool detect_touch_middle = false;
		bool detect_touch_ring = false;
		bool detect_touch_pinky = false;

		float skin_values[SKIN_DIM];

        if (input!=NULL) {
			//fprintf(stderr,"Got message: %s\n", input->toString().c_str());
			///Vector skin_values;
			///skin_values.resize(input->size());
			//for (int i=0; i<input->size(); i++) {
			for (int i=0; i<SKIN_DIM; i++) {
			    ///skin_values[i] = input->get(i).asDouble();
				skin_storage[i][skin_storage_count] = input->get(i).asInt();
				skin_values[i] = 0;
				for (int j=0; j < STORE_AVG; j++) {
					skin_values[i] += skin_storage[i][j];
				}
				skin_values[i] = skin_values[i] / STORE_AVG;
			}
			skin_storage_count++;
			skin_storage_count = skin_storage_count % STORE_AVG;
			



			
			//index-finger
			for (int i=0; i<12; i++) {
				//fprintf(stderr,"%d ", int(skin_start[i] - skin_values[i]));
				if ((skin_start[i] - skin_values[i]) > TOUCH_TRESHHOLD) {
					detect_touch_index = true;
				}
			}
			//middle-finger
			for (int i=12; i<24; i++) {
				//fprintf(stderr,"%d ", int(skin_start[i] - skin_values[i]));
				if ((skin_start[i] - skin_values[i]) > TOUCH_TRESHHOLD) {
					detect_touch_middle = true;
				}
			}
			//ring-finger
			for (int i=24; i<36; i++) {
				//fprintf(stderr,"%d ", int(skin_start[i] - skin_values[i]));
				if ((skin_start[i] - skin_values[i]) > TOUCH_TRESHHOLD) {
					detect_touch_ring = true;
				}
			}
			//pinky-finger
			for (int i=36; i<48; i++) {
				//fprintf(stderr,"%d ", int(skin_start[i] - skin_values[i]));
				if ((skin_start[i] - skin_values[i]) > TOUCH_TRESHHOLD) {
					detect_touch_pinky = true;
				}
			}

			//index-finger
			if (detect_touch_index) {
				fprintf(stderr,"Index detected ");
				//fprintf(stderr,"%+2.1lf %+2.1lf ", encoders[11], encoders[12]);
				//fprintf(stderr,"\r");
				if (encoders[11] > 0) 
					vel->velocityMove(11,speedo);	
				else
					vel->velocityMove(11,0);
				if (encoders[12] > 0) 
					vel->velocityMove(12,speedo);	
				else
					vel->velocityMove(12,0);
			}
			else {
				fprintf(stderr,"Index emptyyyy ");
				//fprintf(stderr,"%+2.1lf %+2.1lf ", encoders[11], encoders[12]);
				//fprintf(stderr,"\r");
				if (encoders[11] < 80) 
					vel->velocityMove(11,speedv);
					//pos->positionMove(11,encoders[11]+5); // relativeMove(11,1);
				else
					vel->velocityMove(11,0);
				
				if (encoders[12] < 80) 
					vel->velocityMove(12,speedv);
					//pos->positionMove(12,encoders[12]+5); // pos->relativeMove(12,1);
				else
					vel->velocityMove(12,0);
				
			}

			//middle-finger
			if (detect_touch_middle) {
				fprintf(stderr,"Middle detected ");
				//fprintf(stderr,"%+2.1lf %+2.1lf ", encoders[13], encoders[14]);
				//fprintf(stderr,"\r");
				if (encoders[13] > 0) 
					vel->velocityMove(13,speedo);	
				else
					vel->velocityMove(13,0);
				if (encoders[14] > 0) 
					vel->velocityMove(14,speedo);	
				else
					vel->velocityMove(14,0);
			}
			else {
				fprintf(stderr,"Middle emptyyyy ");
				//fprintf(stderr,"%+2.1lf %+2.1lf ", encoders[13], encoders[14]);
				//fprintf(stderr,"\r");
				if (encoders[13] < 80) 
					vel->velocityMove(13,speedv);
					//pos->positionMove(11,encoders[11]+5); // relativeMove(11,1);
				else
					vel->velocityMove(13,0);
				
				if (encoders[14] < 80) 
					vel->velocityMove(14,speedv);
					//pos->positionMove(12,encoders[12]+5); // pos->relativeMove(12,1);
				else
					vel->velocityMove(14,0);
				
			}

			//ring+pinky
			if (detect_touch_ring || detect_touch_pinky) {
				fprintf(stderr,"RingPinky detected ");
				//fprintf(stderr,"%+2.1lf ", encoders[15]);
				fprintf(stderr,"\r");
				if (encoders[15] > 0) 
					vel->velocityMove(15,speedo*4);	
				else
					vel->velocityMove(15,0);
			}
			else {
				fprintf(stderr,"RingPinky emptyyyy ");
				//fprintf(stderr,"%+2.1lf ", encoders[15]);
				fprintf(stderr,"\r");
				if (encoders[15] < 150) 
					vel->velocityMove(15,speedv*4);
					//pos->positionMove(11,encoders[11]+5); // relativeMove(11,1);
				else
					vel->velocityMove(15,0);
												
			}

        }
		Time::delay(0.01);
    }
    



	robotDevice.close();
	//input.close();
    
    return 0;
	
}