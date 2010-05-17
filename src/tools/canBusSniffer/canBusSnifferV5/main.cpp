// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
*
Connect to the canbus, read messages and dump to file. Based on 
code by Lorenzo Natale adn Alberto Parmiggiani.

\author Unknown

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/canBusSniffer/main.cpp.
**/

#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/RateThread.h>
#include "fft.h"
#include "hist.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <bitset>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

using namespace std;

const int SNIFFER_THREAD_RATE=50;
const int CAN_DRIVER_BUFFER_SIZE=2047;
const int localBufferSize=2048;
bool done=false;



/******************************************************************************/
/*void test01 ( void )
{
#define F_SAMPLE 1000
  int i;
  fftw_complex *in;
  fftw_complex *in2;
  int n = 1000;
  fftw_complex *out;
  fftw_plan plan_forward;
  double *fft;
  double *freq;

  //Create the input array
  in = (fftw_complex*)fftw_malloc ( sizeof ( fftw_complex ) * n );

#define CONV_SIN 6.28/360*0.36

  for ( i = 0; i < n; i++ )
  {
    in[i][0] = 1*sin(5*i*CONV_SIN)+3*sin(22*i*CONV_SIN);
    in[i][1] = 0;
  }

  printf ( "\n" );
  printf ( "  Input Data:\n" );
  printf ( "\n" );

  for ( i = 0; i < n; i++ )
  {
    printf ( "  %3d  %12f  %12f\n", i, in[i][0], in[i][1] );
  }

  //Create the output array.
  out = (fftw_complex*)fftw_malloc ( sizeof ( fftw_complex ) * n );
  fft = new double[n];
  
  plan_forward = fftw_plan_dft_1d ( n, in, out, FFTW_FORWARD, FFTW_ESTIMATE );

  fftw_execute ( plan_forward );

  printf ( "\n" );
  printf ( "  Output FFT Coefficients:\n" );
  printf ( "\n" );

  for ( i = 0; i < n; i++ )
  {
    fft[i] = 2*norm_abs(out[i][0],out[i][1])/n ;
  }

  int NNFT = n; //400
  freq = new double[NNFT/2];
  for ( i = 0; i < NNFT/2; i++ )
  {
	  freq[i] = float(i)/(float(NNFT)/2);
	  
	  freq[i] = F_SAMPLE/2*freq[i];
  }

  for ( i = 0; i < 80; i++ )
  {
	 	printf ( "  %12f %12f \n",  freq[i], fft[i]);
  }


  //Free up the allocated memory.
  fftw_destroy_plan ( plan_forward );

  fftw_free ( in );
  fftw_free ( out );

  return;
}
*/
 bool log_start=false;
class SnifferThread: public RateThread
{
    PolyDriver driver;
    ICanBus *iCanBus;
    ICanBufferFactory *iBufferFactory;
    CanBuffer messageBuffer;
	unsigned long int cnt;    
	FILE *fp;

	/* dimension of local buffer, number of recieved messages */
    unsigned int messages, readMessages;

	/* variables to be sniffed from the Can Bus */
	unsigned int position[2];
	signed short pwm[2];
	signed short pid[2];
	signed short kp[2];
	signed short dutyCycle[2];
	signed short torque[2];
	signed short commut[2];
	unsigned short unsigned_gaugeData[6];
	signed short signed_gaugeData[6];
	fft_samples   samples;
	fft_cyclic_sample_buffer csamples;
	fft_performer fft;
	hist_performer hist;

	unsigned char cycleIndex;
	signed short filterCoefficients[6];
	signed short dataBuffer[6];

	signed short filterData(signed short datum);
	




public:
    SnifferThread(int r=SNIFFER_THREAD_RATE): RateThread(r)
    {
		messages = localBufferSize;
		cycleIndex = 0;
		filterCoefficients[0] = 1;
		filterCoefficients[1] = 1;
		filterCoefficients[2] = 1;
		filterCoefficients[3] = 1;
		filterCoefficients[4] = 1;
		filterCoefficients[5] = 1;
		for(int i=0; i<6; i++)
			dataBuffer[i] = 0;

		for(int i=0; i<2; i++)
		{
			position[i] = 0;
			pwm[i] = 0;
			dutyCycle[i] = 0;
		}
		
	}

    bool threadInit()
    {
		/* load configuration parameters into the options property collector */
        Property prop;

		/* set driver properties */
		prop.put("device", "ecan");

        prop.put("CanTxTimeout", 500);
        prop.put("CanRxTimeout", 500);
        prop.put("CanDeviceNum", 2);

		prop.put("CanTxQueue", CAN_DRIVER_BUFFER_SIZE);
        prop.put("CanRxQueue", CAN_DRIVER_BUFFER_SIZE);

		/* open driver */
        driver.open(prop);

        if (!driver.isValid())
        {
            fprintf(stderr, "Error opening PolyDriver check parameters\n");
            return false;
        }

        driver.view(iCanBus);
        driver.view(iBufferFactory);

		/* set the baud rate (0 is defaul for 1Mb/s) */
		if (!iCanBus->canSetBaudRate(0))
			fprintf(stderr, "Error setting baud rate\n");

		/* add the id of can messages to be read */
		iCanBus->canIdAdd(0x35a);
	    iCanBus->canIdAdd(0x35b);
//		iCanBus->canIdAdd(0x171);
//		iCanBus->canIdAdd(0x172);
		iCanBus->canIdAdd(0x177);

        messageBuffer=iBufferFactory->createBuffer(localBufferSize);

		/* turn force-torque sensor on
		messageBuffer[0].setId(0x205);
		messageBuffer[0].setLen(2);
		messageBuffer[0].getData()[0]=7;
		messageBuffer[0].getData()[1]=0;
		bool res=iCanBus->canWrite(messageBuffer,1,&readMessages);
		
		if (res)
			return true;
		else
		{
			fprintf(stderr,"Error starting the force/torque sensor\n");
			return false;
		}
		*/

		cnt = 0;
		fp = fopen("output.dat","w");
    }

    void run()
    {
		readMessages = 0; 
		/* read from the Can Bus messages with the id that has been 
		   specified */
        bool res=iCanBus->canRead(messageBuffer, messages, &readMessages);
		
		/* cycle from zero to the number of read messages */   
		for(int i=0; i<readMessages; i++)
		{
			if (messageBuffer[i].getId() == 0x35a)
			{
				unsigned_gaugeData[0] = (messageBuffer[i].getData()[1]<<8)|messageBuffer[i].getData()[0];
				unsigned_gaugeData[1] = (messageBuffer[i].getData()[3]<<8)|messageBuffer[i].getData()[2];
				unsigned_gaugeData[2] = (messageBuffer[i].getData()[5]<<8)|messageBuffer[i].getData()[4];
				signed_gaugeData[0] = unsigned_gaugeData[0]-0x7fff;
				signed_gaugeData[1] = unsigned_gaugeData[1]-0x7fff;
				signed_gaugeData[2] = unsigned_gaugeData[2]-0x7fff;
				//if (csamples.add_sample(signed_gaugeData[0])) fft.do_fft(csamples);
				//if (hist.add_sample(unsigned_gaugeData[0])) hist.do_hist();
			}
			if (messageBuffer[i].getId() == 0x35b)
			{
				unsigned_gaugeData[3] = (messageBuffer[i].getData()[1]<<8)|messageBuffer[i].getData()[0];
				unsigned_gaugeData[4] = (messageBuffer[i].getData()[3]<<8)|messageBuffer[i].getData()[2];
				unsigned_gaugeData[5] = (messageBuffer[i].getData()[5]<<8)|messageBuffer[i].getData()[4];
				signed_gaugeData[3] = unsigned_gaugeData[3]-0x7fff;
				signed_gaugeData[4] = unsigned_gaugeData[4]-0x7fff;
				signed_gaugeData[5] = unsigned_gaugeData[5]-0x7fff;
			}
			if (messageBuffer[i].getId() == 0x171)
				/* recompose scalar value from four bytes */
				position[0] = (messageBuffer[i].getData()[3]<<24)|(messageBuffer[i].getData()[2]<<16)|(messageBuffer[i].getData()[1]<<8)|messageBuffer[i].getData()[0];

			if (messageBuffer[i].getId() == 0x177) //COMMUT
			{
				//commut[0] = (messageBuffer[i].getData()[5]<<8)|messageBuffer[i].getData()[4];
				torque[0] = (messageBuffer[i].getData()[1]<<8)|messageBuffer[i].getData()[0];
				dutyCycle[0] = (messageBuffer[i].getData()[3]<<8)|messageBuffer[i].getData()[2];
				pid[0] = (messageBuffer[i].getData()[5]<<8)|messageBuffer[i].getData()[4];
				kp[0] = (messageBuffer[i].getData()[7]<<8)|messageBuffer[i].getData()[6];
			
			/*	if (kp[0]==1000)
				{
					this->stop();
					fclose(fp);
					exit(0);
				};*/
				
				//FFT
				//if (samples.add_sample(torque[0])) fft.do_fft(samples);
				//if (samples.add_sample(dutyCycle[0])) fft.do_fft(samples);
				if (csamples.add_sample(torque[0])) fft.do_fft(csamples);

				if (kp[0]==30 && log_start==false) log_start=true;

				log_start=true;
				
				if(log_start) fprintf(fp,"%d %d %d %d %d\n",cnt,kp[0],pid[0],dutyCycle[0],torque[0]);
				cnt++;
			}
		}
			

/*
			if (messageBuffer[i].getId() == 0x172)
			{
				pwm[0] = (messageBuffer[i].getData()[1]<<8)|messageBuffer[i].getData()[0];
				dutyCycle[0] = (messageBuffer[i].getData()[5]<<8)|messageBuffer[i].getData()[4];
			}
*/

//			fprintf(fp,"%d %d %d %d %d %d %d\n",cnt,gaugeData[0],gaugeData[1],gaugeData[2],gaugeData[3],gaugeData[4],gaugeData[5]);
			//fprintf(fp,"%d %d %d %d %d %d\n",cnt,signed_gaugeData[5],torque[0],dutyCycle[0],pid[0],kp[0]);
/*
			if (cnt==50000) 
			{
				this->stop();
				done =true;
			}
*/
		
	/*	
		cout<<setiosflags(ios::fixed)
			<<setw(10)<<"commut:"
			<<setw(8)<<setprecision(3)<<commut[0]
			<<" duty cycle:"
			<<setw(8)<<setprecision(3)<<dutyCycle[0]
			<<" gauge 5:"
			<<setw(8)<<setprecision(3)<<signed_gaugeData[5]
			<<" dsp torque:"
			<<setw(8)<<setprecision(3)<<torque[0]
			<<" pid:"
			<<setw(8)<<setprecision(3)<<pid[0]
			<<" kp:"
			<<setw(8)<<setprecision(3)<<kp[0]
			<<"\r";
		*/	
		/*
		cout<<setiosflags(ios::fixed)
			<<" "
			<<setw(8)<<setprecision(3)<<cnt
			<<" "
			<<setw(8)<<setprecision(3)<<gaugeData[0]
			<<" "
			<<setw(8)<<setprecision(3)<<gaugeData[1]
			<<" "
			<<setw(8)<<setprecision(3)<<gaugeData[2]
			<<" "
			<<setw(8)<<setprecision(3)<<gaugeData[3]
			<<" "
			<<setw(8)<<setprecision(3)<<gaugeData[4]
			<<" "
			<<setw(8)<<setprecision(3)<<gaugeData[5]<<"\r";
		*/
    }

    void threadRelease()
    {
		/* turn force-torque sensor off
		messageBuffer[0].setId(0x205);
		messageBuffer[0].setLen(2);
		messageBuffer[0].getData()[0]=7;
		messageBuffer[0].getData()[1]=1;
		bool res=iCanBus->canWrite(messageBuffer,1,&readMessages);
		*/

        iBufferFactory->destroyBuffer(messageBuffer);
        driver.close();
		fclose(fp);
    }
};

signed short SnifferThread::filterData(signed short datum)
{
	int accumulator = 0;
	dataBuffer[cycleIndex] = datum;
	cycleIndex = (cycleIndex==5 ? 0: cycleIndex++);
	for(int i=0; i<6; i++)
		accumulator = accumulator + (dataBuffer[i]*filterCoefficients[i]);
	return accumulator/6;
}


int main(int argc, char *argv[]) 
{
    SnifferThread thread;

    thread.start();

    std::string input;
    while(!done)
    {
/*        std::cin>>input;
        if (input=="quit")
            done=true;*/
    }

    thread.stop();
}