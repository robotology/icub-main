#include <iCub/TrackerThread.h>



/**
* Constructor of the thread
* @param Property &op is a parameter passed as reference of the thread property
*/
TrackerThread::TrackerThread(Property &op):RateThread(THREAD_RATE)
    {
        options=op;
        joints=0;
		v_l=NULL;
		v_r=NULL;

		//targetLeft=0; 
		//targetRight=0;
		//trackingPort=0;
		//inertial=0;

        enableVOR=true;
        enableTracking=true;
        
    }

TrackerThread::~TrackerThread(){}

bool TrackerThread::threadInit()
{
    just_eyes=false;
	fprintf(stderr, "Starting thread\n");
    dd.open(options);
    if (!dd.isValid()) 
        {
            printf("Device not available.  Here are the known devices:\n");
            printf("%s", Drivers::factory().toString().c_str());
			getch();
            return false;
        }

    bool ok;
    ok = dd.view(ivel);
    ok = ok&&dd.view(ipos);        
    ok = ok&&dd.view(iencs);        

    if (!ok)
        {
            fprintf(stderr, "Problems acquiring interfaces\n");
            return false;
        }

    ivel->getAxes(&joints);
    printf("Working with %d axes\n", joints);
    
    encoders = new double [joints];
    command = new double [joints];
    accelerations = new double [joints];
    prev = new double [joints];
    
    int k=0;
    for (k=0; k<joints; k++)
        {
            accelerations[k]=100;
            command[k]=0;
            prev[k]=command[k];
        }

    ivel->setRefAccelerations(accelerations);
    Time::delay(0.3);
    ivel->velocityMove(command);
    Time::delay(1);

    ipos->setRefSpeeds(START_SPEED);
    ipos->positionMove(START_POSITION);

    bool done=false;
    fprintf(stderr, "Going to start position\n");
    while(!done)
        {
            Time::delay(0.3);
            ipos->checkMotionDone(&done);
            fprintf(stderr, ".");
        }
    fprintf(stderr, "done!\n");

    targetR.resize(3);
    targetR=0;
    targetL.resize(3);
    targetL=0;

    timeStampL=0;
    timeStampR=0;
    timeStampLprev=timeStampL;
    timeStampRprev=timeStampR;
    return true;
}

void TrackerThread::run()
{
    iencs->getEncoders(encoders);
    //Vector *vr=targetRight->read(0);
	Vector *vr=v_r;
	Vector *vl=v_l;
    //Vector *vl=targetLeft->read(0);
    Vector *gyro=inertial->read(0);
    double timeNow=Time::now();
    double delayR=0.0;
    double delayL=0.0;

    for(int k=0;k<joints;k++)
        command[k]=0;

    if (vr!=0)
        {
            targetR=*vr;
            timeStampRprev=timeStampR;
            timeStampR=Time::now();
        }

    if (vl!=0)
        {
            targetL=*vl;
            timeStampLprev=timeStampL;
            timeStampL=Time::now();
        }

    delayR=timeNow-timeStampR;
    delayL=timeNow-timeStampL;

    if ( (delayR<TIMEOUT)&&(delayL<TIMEOUT))
        {                           
            //binocular
            double d=targetR(0)-targetL(0);
			command[4]=Kpan*(0.7*targetR(0)+0.3*targetL(0));
			command[5]=Kvergence*d;
			command[3]=Ktilt*0.5*(targetL(1)+targetR(1));
			if(!just_eyes){
				command[0]=Kn_tilt*(encoders[3]);
				command[2]=Kn_pan*(encoders[4]);
			}
            timeStampTO=timeNow;
            enableVOR=true;
        }
    else if (delayR<TIMEOUT)
        {
            //monocular
			command[4]=Kpan*targetR(0); //targetR(0) goal velocity for pan axis
			command[5]=0.1*Kvergence*(encoders[5]-10);  //encoders[5] eyes vergence
			command[3]=Ktilt*targetR(1); //targetR(1) goal velocity for tilt axis
			if(!just_eyes){
				command[0]=Kn_tilt*(encoders[3]); //encoders[3] eyes tilt
				command[2]=Kn_pan*(encoders[4]); //encoders[4] eyes pan
			}
            timeStampTO=timeNow;
            enableVOR=true;
        }
    else if (delayL<TIMEOUT)
        {
			command[4]=Kpan*targetL(0);
			command[5]=0.1*Kvergence*(encoders[5]-10);
			command[3]=Ktilt*targetL(1);
			if(!just_eyes){
				command[0]=Kn_tilt*(encoders[3]);
				command[2]=Kn_pan*(encoders[4]);
			}
            timeStampTO=timeNow;
            enableVOR=true;
        }
    else
        {
            if ((timeNow-timeStampTO)>STOP_TIME)
                {
                    enableVOR=false;
                    command[4]=0;
                    command[5]=0;
                    command[3]=0;
                    command[0]=0;
                    command[2]=0;
                }
            else
                {
                    for(int k=0;k<joints;k++)
                        command[k]=2.2*prev[k]/STOP_TIME;
                }
        }

    if ( (gyro != 0) && (enableVOR))
        {
            command[1] = vorx*(*gyro)[0]; //roll
            command[3] -= ((*gyro)[6] * 180.0 / 3.141528 * vory); //tilt
            command[4] -= ((*gyro)[7] * 180.0 / 3.141528 * vorz); //pan
        }
    else
        {
            command[1]=0.0; //roll
        }
        
    bool ok=ivel->velocityMove(command);

    for(int k=0;k<joints;k++)
        prev[k]=command[k];
}

void TrackerThread::threadRelease()
{
    fprintf(stderr, "Releasing thread\n");
    int k=0;
    for(k=0;k<joints;k++)
        command[k]=0;
    ivel->velocityMove(command);
    Time::delay(0.5);

    bool done=false;
    fprintf(stderr, "Going back to start position\n");
    ipos->setRefSpeeds(START_SPEED);
    ipos->positionMove(START_POSITION);
    while(!done)
        {
            Time::delay(0.3);
            ipos->checkMotionDone(&done);
            fprintf(stderr, ".");
        }
    fprintf(stderr, "done!\n");

    delete [] command;
    delete [] encoders;
    delete [] accelerations;
}

void TrackerThread::setLeftVector(double a, double b, double c){
	double *p=new double;
	p[0]=a;
	p[1]=b;
	p[2]=c;
	v_l=new Vector(3,p);
}

void TrackerThread::setRightVector(double a, double b, double c){
	double *p=new double;
	p[0]=a;
	p[1]=b;
	p[2]=c;
	v_r=new Vector(3,p);
}

void TrackerThread::resetVector(){
	enableVOR=false;
    command[4]=0;
    command[5]=0;
    command[3]=0;
    command[0]=0;
    command[2]=0;
	v_r=0;
	v_l=0;
}