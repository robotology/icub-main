// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
*
@ingroup icub_module
\defgroup icub_canbussniffer canBusSniffer

Connect to the canbus, read messages from the force torque sensor and dump them
to file for calibration.

\author Marco Randazzo

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/canBusSniffer/main.cpp.
**/

#include <stdio.h>
//#include <conio.h>
//#include <time.h>
#include <iostream>
#include <yarp/os/Network.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <deque>
//#include <direct.h>

#include <gtk/gtk.h>
#include <gtk/gtkmain.h>


//YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp;

using namespace std;

const int SNIFFER_THREAD_RATE=50;
const int DISPLAY_THREAD_RATE=200;
const int CAN_DRIVER_BUFFER_SIZE=2047;
const int localBufferSize=2048;
#define NSAMPLES 1000

bool done=false;
GtkWidget *window			 = NULL;
GtkWidget *button_next       = NULL;
GtkWidget *button1           = NULL;
GtkWidget *button2           = NULL;
GtkWidget *button3           = NULL;
GtkWidget *button4           = NULL;
GtkWidget *button5           = NULL;
GtkWidget *button6           = NULL;
GtkWidget *button7           = NULL;
GtkWidget *button8           = NULL;
GtkWidget *button9           = NULL;
GtkWidget *button10          = NULL;
GtkWidget *button11          = NULL;
GtkWidget *button12          = NULL;
GtkWidget *button13          = NULL;
GtkWidget *fix1              = NULL;
GtkWidget *label_read_title  = NULL;
GtkWidget *label_read[6]     ;
GtkWidget *label_expected_title  = NULL;
GtkWidget *label_expected[6] ;
GtkWidget *label_error_title  = NULL;
GtkWidget *label_error[6] ;
GtkWidget *label_message	 = NULL;

bool        button_completed[13];
int			current_button_num=-1;
GtkButton*  current_button_handle= NULL;

char text_buffer[5000];
char text_buffer2[5000];
#define btex  text_buffer +strlen(text_buffer) 
#define btex2 text_buffer2+strlen(text_buffer2) 

void turn_off_buttons();
void turn_on_buttons();
void close_files();
int current_trial = -1;

gboolean timer_graphic_updater (gpointer data);
guint    timer_graphic_updater_h = 0;
#define START_TIMER {if (timer_graphic_updater_h==0) timer_graphic_updater_h = g_timeout_add (500, timer_graphic_updater, NULL);}
#define STOP_TIMER  {if (timer_graphic_updater_h>0)  g_source_remove(timer_graphic_updater_h); timer_graphic_updater_h=0;}

#include "expected_values.h"
expected_values_handler_class expected_values_handler;

//#define ATI_SENS
#ifdef ATI_SENS
#include "devMeasurementFns.h"
devMeasurementFns* ATIsens;
#endif



/******************************************************************************/

bool log_start=false;
FILE *fp=0;
//FILE *fp2=0;
int trial;
deque<unsigned_elem_class>  queue_unsigned_gaugeData;
deque<signed_elem_class>    queue_signed_gaugeData;
signed_elem_class           trial_bias;
signed_elem_class           last_value;

/******************************************************************************/
void btex_show()
{
	char tmp [10000];
	strcpy(tmp,text_buffer);
	strcat(tmp,text_buffer2);
	gtk_label_set_text(GTK_LABEL(label_message),tmp); 
}

/******************************************************************************/
void clear_text_buffer(int i)
{
	if (i==1) memset(text_buffer,0,5000);
	if (i==2) memset(text_buffer2,0,5000);
}

/******************************************************************************/
int remap_trials ()
{
	//{8020,1001,8010,8030,8040,8050,8060,8070,8080,3001,4001,5001,6001};

	switch (trial)
	{
	    case 1001: return -1;
		case 1002: return 0;
		case 1003: return 1;
		case 1004: return 2;
		case 1005: return 3;
		
		case 3001: return -1;
		case 3002: return 4;
		case 3003: return 5;
		case 3004: return 6;

		case 4001: return -1;
		case 4002: return 7;
		case 4003: return 8;
		case 4004: return 9;
			
		case 5001: return -1;
		case 5002: return 10;
		case 5003: return 11;
		case 5004: return 12;

		case 6001: return -1;
		case 6002: return 13;
		case 6003: return 14;
		case 6004: return 15;

		case 8010: return -1;
		case 8011: return 16;

		case 8020: return -1;
		case 8021: return 17;

		case 8030: return -1;
		case 8031: return 18;

		case 8040: return -1;
		case 8041: return 19;

		case 8050: return -1;
		case 8051: return 20;

		case 8060: return -1;
		case 8061: return 21;

		case 8070: return -1;
		case 8071: return 22;

		case 8080: return -1;
		case 8081: return 23;

		case -1:   
			//printf("DEBUG: section completed, back to the menu\n");
		return -1;

		default: 
			printf("DEBUG: **** ERROR, invalid trial (%d) selected! ****\n", trial);
		return -1;
	}
}
/******************************************************************************/
void update_gtk_labels()
{
	char tempbuf[255];
	int  i=0;

	current_trial=remap_trials();
	signed_elem_class exp_vals;
	signed_elem_class err_vals;
	signed_elem_class in_bound;
	
	if (current_trial==-1)
	{
		for (i=0; i<6;i++)
			{
				GdkColor color;
				gdk_color_parse ("black", &color);

				sprintf(tempbuf,"%d",last_value.dat[i]);
				gtk_label_set_text(GTK_LABEL(label_read[i]),tempbuf); 

				sprintf(tempbuf,"- - -");
				gtk_label_set_text(GTK_LABEL(label_expected[i]),tempbuf); 
				gtk_widget_modify_fg (label_expected[i], GTK_STATE_NORMAL, &color);

				sprintf(tempbuf,"- - -");
				gtk_label_set_text(GTK_LABEL(label_error[i]),tempbuf); 
				gtk_widget_modify_fg (label_error[i], GTK_STATE_NORMAL, &color);
			}
	}
	else
	{
		expected_values_handler.get_current_expected_values(exp_vals, current_trial);
		for (i=0; i<6;i++)
			{
				sprintf(tempbuf,"%d",exp_vals.dat[i]);
				gtk_label_set_text(GTK_LABEL(label_expected[i]),tempbuf); 
			}

		signed_elem_class last;
		last=last_value;
		last.remove_bias(trial_bias);
		for (i=0; i<6;i++)
		{
			sprintf(tempbuf,"%d",last.dat[i]);
			gtk_label_set_text(GTK_LABEL(label_read[i]),tempbuf); 
		}
		bool in_boundary = expected_values_handler.check_vals(last,current_trial,err_vals,in_bound);
		for (i=0; i<6; i++)
		{
			sprintf(tempbuf,"%d",err_vals.dat[i]);
			gtk_label_set_text(GTK_LABEL(label_error[i]),tempbuf); 
			if (in_bound.dat[i]==0)
			{
				GdkColor color;
				gdk_color_parse ("red", &color);
				gtk_widget_modify_fg (label_error[i], GTK_STATE_NORMAL, &color);
			}
			else if (in_bound.dat[i]==1)
			{
				GdkColor color;
				gdk_color_parse ("green", &color);
				gtk_widget_modify_fg (label_error[i], GTK_STATE_NORMAL, &color);
			}
			else if (in_bound.dat[i]==2)
			{
				GdkColor color;
				gdk_color_parse ("orange", &color);
				gtk_widget_modify_fg (label_error[i], GTK_STATE_NORMAL, &color);
			}
		}
		if (in_boundary==false)
		{
			 gtk_button_set_label (GTK_BUTTON(button_next), "Strange values detected.\n            Acquire? ");
		}
		else
		{	 
			 gtk_button_set_label (GTK_BUTTON(button_next), "Acquire data");
		}			
	}
}

gboolean timer_graphic_updater (gpointer data)
{
		update_gtk_labels();
		return true;
};

 class SnifferThread: public RateThread
{
    PolyDriver driver;
    ICanBus *iCanBus;
    ICanBufferFactory *iBufferFactory;
    CanBuffer messageBuffer;
	unsigned long int cnt;    

	/* dimension of local buffer, number of recieved messages */
    unsigned int messages, readMessages;

	/* variables to be sniffed from the Can Bus */
	bool flag1, flag2;
	unsigned short unsigned_gaugeData[6];
	signed short signed_gaugeData[6];

public:
    SnifferThread(int r=SNIFFER_THREAD_RATE): RateThread(r)
    {
		messages = localBufferSize;
		// create data directory if it does not exist
		yarp::os::mkdir("data");

	}

    bool threadInit()
    {
		#ifdef USE_ICUB_MOD
		yarp::dev::DriverCollection dev;
		#endif
		
		/* load configuration parameters into the options property collector */
        Property prop;
		flag1 = false;
    	flag2 = false;

        const char devicetype[] = "socketcan"; // for windows use: "ecan"

		/* set driver properties */
        //prop.put("device", "socketcan");
        prop.put("device", devicetype);

        prop.put("CanTxTimeout", 500);
        prop.put("CanRxTimeout", 500);
        prop.put("CanDeviceNum", 0);

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
		fprintf(stderr, "Adding CanID: 0x2D0 0x3DA 0x3DB\n");
		iCanBus->canIdAdd(0x2D0);
		iCanBus->canIdAdd(0x3DA);
	    iCanBus->canIdAdd(0x3DB);
		/*fprintf(stderr, "Adding CanID: 0x250 0x35A 0x35B\n");
		iCanBus->canIdAdd(0x250);
		iCanBus->canIdAdd(0x35a);
	    iCanBus->canIdAdd(0x35b);*/
		//iCanBus->canIdAdd(0x111);
        //iCanBus->canIdAdd(0x172);
		//iCanBus->canIdAdd(0x177);

        messageBuffer=iBufferFactory->createBuffer(localBufferSize);
		//		CanBuffer TxBuffer = iCanBus->createBuffer(1);
	    //turn force-torque sensor on
		messageBuffer[0].setId(0x20D);
		messageBuffer[0].setLen(2);
		messageBuffer[0].getData()[0]=7;
		messageBuffer[0].getData()[1]=3;
		bool res=iCanBus->canWrite(messageBuffer,1,&readMessages);
		
		if (res)
		{
			fprintf(stderr,"0x20D 07 03: sensor started\n");
			return true;
		}
		else
		{
			fprintf(stderr,"Error starting the force/torque sensor\n");
			return false;
		}
		/**/
    }

    void run()
    {
			static int error_count=0;
			readMessages = 0; 
			/* read from the Can Bus messages with the id that has been 
			   specified */
			bool res=iCanBus->canRead(messageBuffer, messages, &readMessages);
		
			/* cycle from zero to the number of read messages */   
			if (readMessages==0)
			{
				printf("   *** PROBLEM ON CAN NET (zero readings) ***			\n");
				error_count++;
				if (error_count==5)
				{
					printf("Five can errors in a row, restarting sensor 0x20D 07 03 \n");
					yarp::os::Time::delay(2);
					messageBuffer[0].setId(0x20D);
					messageBuffer[0].setLen(2);
					messageBuffer[0].getData()[0]=7;
					messageBuffer[0].getData()[1]=3;
					iCanBus->canWrite(messageBuffer,1,&readMessages);
					error_count=0;
				}
			}
			if (res==false)
			{
				printf("   *** PROBLEM ON CAN NET (canRead failed) ***			\n");
				yarp::os::Time::delay(2);
			}

			for(int i=0; i<int(readMessages); i++)
			{
				if (messageBuffer[i].getId() == 0x2D0)
				{
					printf("   *** PROBLEM ON CAN NET (power down) ***			\n");
					printf("   restarting sensor 0x20D 07 03 \n");
					yarp::os::Time::delay(2);
					messageBuffer[0].setId(0x20D);
					messageBuffer[0].setLen(2);
					messageBuffer[0].getData()[0]=7;
					messageBuffer[0].getData()[1]=3;
					iCanBus->canWrite(messageBuffer,1,&readMessages);
				}
				if (messageBuffer[i].getId() == 0x3Da)
				{
					unsigned_gaugeData[0] = (messageBuffer[i].getData()[1]<<8)|messageBuffer[i].getData()[0];
					unsigned_gaugeData[1] = (messageBuffer[i].getData()[3]<<8)|messageBuffer[i].getData()[2];
					unsigned_gaugeData[2] = (messageBuffer[i].getData()[5]<<8)|messageBuffer[i].getData()[4];
					signed_gaugeData[0] = unsigned_gaugeData[0]-0x7fff;
					signed_gaugeData[1] = unsigned_gaugeData[1]-0x7fff;
					signed_gaugeData[2] = unsigned_gaugeData[2]-0x7fff;
					flag1= true;
				}
				if (messageBuffer[i].getId() == 0x3Db)
				{
					unsigned_gaugeData[3] = (messageBuffer[i].getData()[1]<<8)|messageBuffer[i].getData()[0];
					unsigned_gaugeData[4] = (messageBuffer[i].getData()[3]<<8)|messageBuffer[i].getData()[2];
					unsigned_gaugeData[5] = (messageBuffer[i].getData()[5]<<8)|messageBuffer[i].getData()[4];
					signed_gaugeData[3] = unsigned_gaugeData[3]-0x7fff;
					signed_gaugeData[4] = unsigned_gaugeData[4]-0x7fff;
					signed_gaugeData[5] = unsigned_gaugeData[5]-0x7fff;
					flag2 = true;
				}
				
				if (flag1==true && flag2==true)
				{
					//fprintf(fp,"%d %d %d %d %d %d %d\n",cnt,signed_gaugeData[0],signed_gaugeData[1],signed_gaugeData[2],signed_gaugeData[3],signed_gaugeData[4],signed_gaugeData[5]);
					//printf("samples:%3d [0]:%+6d [1]:%+6d [2]:%+6d [3]:%+6d [4]:%+6d [5]:%+6d\r\n",cnt,signed_gaugeData[0],signed_gaugeData[1],signed_gaugeData[2],signed_gaugeData[3],signed_gaugeData[4],signed_gaugeData[5]);
					while (queue_signed_gaugeData.size()>NSAMPLES)
					{
						queue_signed_gaugeData.pop_back();
					}
					while (queue_unsigned_gaugeData.size()>NSAMPLES)
					{
						queue_unsigned_gaugeData.pop_back();
					}
					
					signed_elem_class s_dat;
					unsigned_elem_class us_dat;

					int j=0;
					for (j=0; j<6; j++)
					{
						s_dat.dat[j]=signed_gaugeData[j];
						us_dat.dat[j]=unsigned_gaugeData[j];
					}
					last_value=s_dat;
					queue_signed_gaugeData.push_front(s_dat);
					queue_unsigned_gaugeData.push_front(us_dat);


					flag1=false;
			        flag2=false;
				}
			}
			int ssize=queue_signed_gaugeData.size();
			static bool wait_size=true;
			if (ssize>=NSAMPLES && wait_size)
			{
				printf("Acquired %d samples. Ready!	\n",ssize);
				wait_size=false;
			}
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
		close_files();
    }
};

bool check_skip(bool ask=true)
{
	char temp[255];
	if (ask)
	{
		printf("\n   press any key and return to acquire the data, s to skip \n");
		cin >> temp;
	}

	printf("\n");

	if (strcmp(temp,"s")==0)
	{
		trial++;
		return true;
	}
	else return false;

}

void close_files()
{
	if (fp!=0)
		{
			fclose(fp);
			fp=0;
		}
	//if (fp2!=0)
	//	{
	//		fclose(fp2);
	//		fp2=0;
	//	}
}

void terminate_section()
{
	trial=-1;
	close_files();
	turn_on_buttons();
	GdkColor color;
	gdk_color_parse ("green", &color);
	gtk_widget_modify_text (GTK_WIDGET(current_button_handle), GTK_STATE_NORMAL, &color);
}

SnifferThread s_thread;

void bias_sensor()
{
	#ifdef ATI_SENS
		ATIsens->BiasForceTorqueSensor();
	#endif
}

#ifdef ATI_SENS
void acquire_1000_samples()
{
	char temp[255];
	float data[6]={0,0,0,0,0,0};
	printf("\n   press return to acquire ***ATI sensor*** data or 'r' to reset\n");
	cin >> temp;

	if (strcmp(temp,"r")==0) 
	{
		ATIsens->BiasForceTorqueSensor();
		printf("\n   bias done. press return to acquire ***ATI sensor*** data\n");
		cin >> temp;
	}

	int i=0;
	int j=0;
	float data_mean[6] = {0,0,0,0,0,0};
	for (i=0; i<1000; i++)
	{
		Sleep(5);
		ATIsens->ReadFTsensorData();
		ATIsens->GetFTData(data);
		fprintf(fp,"%3d %+f %+f %+f %+f %+f %+f\n",i,data[0],data[1],data[2],data[3],data[4],data[5]);
		printf("samples:%3d [0]:%+6.2f [1]:%+6.2f [2]:%+6.2f [3]:%+6.2f [4]:%+6.2f [5]:%+6.2f\r\n",i,data[0],data[1],data[2],data[3],data[4],data[5]);
		for (j=0; j<6; j++)
		{
			data_mean[j]=data_mean[j]+data[j];
		}
	}
	for (j=0; j<6; j++)
		{
			data_mean[j]/=1000;
		}
	//fprintf(fp2,"%3d %+f %+f %+f %+f %+f %+f\n",i,data_mean[0],data_mean[1],data_mean[2],data_mean[3],data_mean[4],data_mean[5]);
	printf("means:%3d [0]:%+6.2f [1]:%+6.2f [2]:%+6.2f [3]:%+6.2f [4]:%+6.2f [5]:%+6.2f\r\n",i,data_mean[0],data_mean[1],data_mean[2],data_mean[3],data_mean[4],data_mean[5]);


	trial++;
}

#else
bool acquire_1000_samples()
{
	s_thread.suspend();

	int i=0;
	int j=0;
	
	signed short   signed_gaugeData[6];
	unsigned short unsigned_gaugeData[6];

	int ssize=queue_signed_gaugeData.size();
	int usize=queue_unsigned_gaugeData.size();
	printf("signed size:%d\n", ssize);
	printf("unsigned size:%d\n", usize);

	if (ssize < NSAMPLES || usize <NSAMPLES)
	{
		printf("\n   WARNING: not enought data were acquired! \n");
		s_thread.resume();
		return false;
	}
	do
	{
		for (j=0; j<6; j++)
		{
			signed_gaugeData[j]=queue_signed_gaugeData[i].dat[j];
			unsigned_gaugeData[j]=queue_unsigned_gaugeData[i].dat[j];
		}
		fprintf(fp,"%d %d %d %d %d %d %d\n",i,signed_gaugeData[0],signed_gaugeData[1],signed_gaugeData[2],signed_gaugeData[3],signed_gaugeData[4],signed_gaugeData[5]);
		printf("samples:%3d [0]:%+6d [1]:%+6d [2]:%+6d [3]:%+6d [4]:%+6d [5]:%+6d\r\n",i,signed_gaugeData[0],signed_gaugeData[1],signed_gaugeData[2],signed_gaugeData[3],signed_gaugeData[4],signed_gaugeData[5]);
		i++;
	}
	while (i<NSAMPLES);

	s_thread.resume();
	return true;
}
#endif

bool menu ()
{
	clear_text_buffer(2);
	{
		switch (trial)
			{
				case  0: 
						  
						  fp = fopen("./data/output0.dat","w");
						  //fp2 = fopen("./data/mean0.dat","w");
						  sprintf(btex2,"__________________________________________________________\n");
						  sprintf(btex2,"                                                          \n");
						  sprintf(btex2,"          beginning calibration ...                       \n");
						  sprintf(btex2,"_________________________________________________________ \n");
						  sprintf(btex2,"\n");
						  sprintf(btex2,"   remove all loads from the sensor                       \n");
						  sprintf(btex2,"   orient the sensor with the flat surface facing upwards \n");
						  sprintf(btex2,"   turn the sensor on (CAN message 205 2 7 0)             \n");
						  btex_show();
						  terminate_section();
						  break;

				case  1001:  
						  fp = fopen("./data/output1.dat","w");
						  //fp2 = fopen("./data/mean1.dat","w");
						  sprintf(btex,"____________________________________________________________________\n");
						  sprintf(btex,"\n       (2)     z+ pointing DOWNwards    5kg torques      file: output1.dat \n");
						  sprintf(btex,"____________________________________________________________________\n\n");
						  sprintf(btex2,"   1. assemble the assembly with a M10 nut                          \n");
						  sprintf(btex2,"   2. orient the assembly with the z+ axis pointing downwards       \n");
						  sprintf(btex2,"\n");
						  sprintf(btex2,"   3. remove loads\n"); 
						  btex_show();
						  break;

				case  1002:  sprintf(btex2,"   4.  apply 5.00 kg on the y- axis\n"); 
						  btex_show();
						  break;

				case  1003:  sprintf(btex2,"   5.   apply 5.00 kg on the x+ axis\n"); 
						  btex_show();
						  break;

				case  1004:  sprintf(btex2,"   6.   apply 5.00 kg on the y+ axis\n"); 
						  btex_show(); 
						  break;

				case  1005:  sprintf(btex2,"   7.   apply 5.00 kg on the x- axis\n"); 
						  btex_show(); 
						  break;
				case  1006:
						  terminate_section();
						  break;
					
				case 3001:
						  fp = fopen("./data/output3.dat","w");
						  //fp2 = fopen("./data/mean3.dat","w");
						  sprintf(btex,"____________________________________________________________________\n");
						  sprintf(btex,"\n       (10)     x+ axis pointing UPwards     5kg laterals      file: output3.dat \n");
						  sprintf(btex,"____________________________________________________________________\n\n");
  						  sprintf(btex2,"   1. assemble the assembly with a M10 knob                         \n");
						  sprintf(btex2,"   2. orient the structure with the x+ axis pointing upwards        \n");
						  sprintf(btex2,"   3. remove loads\n");
						  btex_show();
						  break;

				case 3002:  sprintf(btex2,"   4.   apply 5.00 kg on the y- axis\n"); 
						  btex_show();
						  break;

				case 3003:  sprintf(btex2,"   5.   apply 5.00 kg on the z+ axis\n"); 
						  btex_show();
						  break;

				case 3004:  sprintf(btex2,"   6.   apply 5.00 kg on the y+ axis\n");
						  btex_show();
						  break;
				case 3005:
						  terminate_section();
						  break;

				case 4001:
					      fp = fopen("./data/output4.dat","w");
						  //fp2 = fopen("./data/mean4.dat","w");
						  sprintf(btex,"____________________________________________________________________\n");
						  sprintf(btex,"\n       (11)     y+ axis pointing UPwards     5kg laterals      file: output4.dat \n");
						  sprintf(btex,"____________________________________________________________________\n\n");
  						  sprintf(btex2,"   1. assemble the assembly with a M10 knob                         \n");
						  sprintf(btex2,"   2. orient the structure with the y+ axis pointing upwards        \n");
						  sprintf(btex2,"   3. remove loads\n");
						  btex_show();
						  break;

				case 4002:  sprintf(btex2,"   4.   apply 5.00 kg on the x+ axis\n"); 
						  btex_show();
						  break;

				case 4003:  sprintf(btex2,"   5.   apply 5.00 kg on the z+ axis\n"); 
						  btex_show();
						  break;

				case 4004:  sprintf(btex2,"   6.   apply 5.00 kg on the x- axis\n"); 
						  btex_show();
						  break;
				case 4005:
						  terminate_section();
						  break;
					
				case 5001:
					      fp = fopen("./data/output5.dat","w");
						  sprintf(btex,"____________________________________________________________________\n");
						  sprintf(btex,"\n       (12)     x- axis pointing UPwards     5kg laterals      file: output5.dat \n");
						  sprintf(btex,"____________________________________________________________________\n\n");
						  //fp2 = fopen("./data/mean5.dat","w");
   					      sprintf(btex2,"   1. assemble the assembly with a M10 knob                         \n");
						  sprintf(btex2,"   2. orient the structure with the x- axis pointing upwards        \n");
						  sprintf(btex2,"   3. remove loads\n");
						  btex_show();
						  break;

				case 5002:  sprintf(btex2,"   4.   apply 5.00 kg on the y+ axis\n"); 
						  btex_show();
						  break;

				case 5003:  sprintf(btex2,"   5.   apply 5.00 kg on the z+ axis\n"); 
						  btex_show();
						  break;

				case 5004:  sprintf(btex2,"   6.   apply 5.00 kg on the y- axis\n"); 
						  btex_show();
						  break;
				case 5005:
						  terminate_section();
						  break;

				case 6001:
						  sprintf(btex,"____________________________________________________________________\n");
						  sprintf(btex,"\n       (13)     y- axis pointing UPwards     5kg laterals      file: output6.dat \n");
						  sprintf(btex,"____________________________________________________________________\n\n");
					      fp = fopen("./data/output6.dat","w");
						  //fp2 = fopen("./data/mean6.dat","w");
      					  sprintf(btex2,"   1. assemble the assembly with a M10 knob                         \n");
						  sprintf(btex2,"   2. orient the structure with the y- axis pointing upwards        \n");
						  sprintf(btex2,"   3. remove loads\n");
						  btex_show();
						  break;

				case 6002:  sprintf(btex2,"   4.   apply 5.00 kg on the x- axis\n"); 
						  btex_show();
						  break;

				case 6003:  sprintf(btex2,"   5.   apply 5.00 kg on the z+ axis\n"); 
						  btex_show();
						  break;

				case 6004:  sprintf(btex2,"   6.   apply 5.00 kg on the x+ axis\n"); 
						  btex_show();
						  break;
				case 6005:
						  terminate_section();
						  break;

				case 8010:
	  					  sprintf(btex,"____________________________________________________________________\n");
						  sprintf(btex,"\n       (3)     z+ pointing UPwards      25kg compression      file: output81.dat \n");
						  sprintf(btex,"____________________________________________________________________\n\n");
  						  sprintf(btex2,"   1. assemble the assembly with a M10 nut                          \n");
						  sprintf(btex2,"   2. orient the assembly with the z+ axis pointing upwards       \n");
						  sprintf(btex2,"\n");
						  sprintf(btex2,"   3. remove loads\n"); 

						  fp = fopen("./data/output81.dat","w");
						  //fp2 = fopen("./data/mean81.dat","w");
						  btex_show();
						  break;
				case 8011:	
					      sprintf(btex2,"   4. apply 25 kg in the z- direction (compression)\n"); 
						  btex_show();
						  break;
				case 8012:
						  terminate_section();
						  break;

				case 8020:
						  sprintf(btex,"____________________________________________________________________\n");
						  sprintf(btex,"\n       (1)     z+ pointing DOWNwards      25kg traction      file: output82.dat \n");
						  sprintf(btex,"____________________________________________________________________\n\n");
  						  sprintf(btex2,"   1. assemble the assembly with a M10 nut                          \n");
						  sprintf(btex2,"   2. screw the M10 ring on the top of the assembly                 \n");						  
						  sprintf(btex2,"   3. orient the assembly with the z+ axis pointing downwards       \n");
						  sprintf(btex2,"\n");
						  sprintf(btex2,"   4. remove loads\n"); 
						  fp = fopen("./data/output82.dat","w");
						  //fp2 = fopen("./data/mean82.dat","w");
						  btex_show();
						  break;
				case 8021:  
						  sprintf(btex2,"   5. apply a 25 kg load in the z+ direction (traction)\n"); 
						  btex_show();
						  break;
				case 8022:
						  terminate_section();
						  break;
				case 8030:
						  sprintf(btex,"____________________________________________________________________\n");
						  sprintf(btex,"\n       (4)     x+ pointing UPwards       25kg      file: output83.dat \n");
						  sprintf(btex,"____________________________________________________________________\n\n");
 						  sprintf(btex2,"   1. assemble the assembly with a M10 knob                          \n");
						  sprintf(btex2,"   2. orient the plate with the x+ axis pointing upwards     \n");
						  sprintf(btex2,"   3. remove loads\n"); 
						  fp = fopen("./data/output83.dat","w");
						  //fp2 = fopen("./data/mean83.dat","w");
						  btex_show();
						  break;
				case 8031:
					      sprintf(btex2,"   4. apply a 25 kg load in the x- direction\n"); 
						  btex_show();
						  break;
				case 8032:
						  terminate_section();
						  break;
				case 8040: 
						  sprintf(btex,"____________________________________________________________________\n");
						  sprintf(btex,"\n       (5)     x- pointing UPwards     25kg      file: output84.dat \n");
						  sprintf(btex,"____________________________________________________________________\n\n");
  						  sprintf(btex2,"   1. assemble the assembly with a M10 knob                          \n");
						  sprintf(btex2,"   2. orient the plate with the x- axis pointing upwards     \n\n");
						  sprintf(btex2,"   3. remove loads\n"); 
						  fp = fopen("./data/output84.dat","w");
						  //fp2 = fopen("./data/mean84.dat","w");
						  btex_show();
						  break;
				case 8041:  
					      sprintf(btex2,"   4. apply a 25 kg load in the x+ direction\n"); 
						  btex_show();
						  break;
				case 8042:
						  terminate_section();
						  break;
				case 8050:
						  sprintf(btex,"____________________________________________________________________\n");
						  sprintf(btex,"\n       (6)     label 1 pointing DOWNwards       25kg      file: output85.dat \n");
						  sprintf(btex,"____________________________________________________________________\n\n");
  						  sprintf(btex2,"   1. assemble the assembly with label 1 pointing DOWN\n\n");
						  sprintf(btex2,"   3. remove loads\n"); 
						  fp = fopen("./data/output85.dat","w");
						  //fp2 = fopen("./data/mean85.dat","w");
						  btex_show();
						  break;
				case 8051:  
					      sprintf(btex2,"   4. apply 25kg\n"); 
						  btex_show();
						  break;
				case 8052:
						  terminate_section();
						  break;
				case 8060:
						  sprintf(btex,"____________________________________________________________________\n");
						  sprintf(btex,"\n       (7)     label 2 pointing DOWNwards       25kg      file: output86.dat \n");
						  sprintf(btex,"____________________________________________________________________\n\n");
  						  sprintf(btex2,"   1. assemble the assembly with a M10 knob                         \n");
						  sprintf(btex2,"   2. orient the plate with label 2 pointing DOWN\n\n");
						  sprintf(btex2,"   3. remove loads\n");
						  fp = fopen("./data/output86.dat","w");
						  //fp2 = fopen("./data/mean86.dat","w");
						  btex_show();
						  break;
				case 8061:  
					      sprintf(btex2,"   4. apply 25kg \n"); 
						  btex_show();
						  break;
				case 8062:
						  terminate_section();
						  break;
				case 8070:
						  sprintf(btex,"____________________________________________________________________\n");
						  sprintf(btex,"\n       (8)     label 3 pointing DOWNwards       25kg      file: output87.dat \n");
						  sprintf(btex,"____________________________________________________________________\n\n");
  						  sprintf(btex2,"   1. assemble the assembly with a M10 knob                         \n");
						  sprintf(btex2,"   2. orient the plate with label 3 pointing DOWN\n\n");
						  sprintf(btex2,"   3. remove loads\n");
						  fp = fopen("./data/output87.dat","w");
						  //fp2 = fopen("./data/mean87.dat","w");
						  btex_show();
						  break;
				case 8071:  
					      sprintf(btex2,"   4. apply 25kg \n"); 
						  btex_show();
						  break;
				case 8072:
						  terminate_section();
						  break;
				case 8080:  
						  sprintf(btex,"____________________________________________________________________\n");
						  sprintf(btex,"\n       (9)     label 4 pointing DOWNwards       25kg      file: output88.dat \n");
						  sprintf(btex,"____________________________________________________________________\n\n");
  						  sprintf(btex2,"   1. assemble the assembly with a M10 knob                         \n");
						  sprintf(btex2,"   2. orient the plate with label 4 pointing DOWN\n\n");
						  sprintf(btex2,"   3. remove loads\n");
						  fp = fopen("./data/output88.dat","w");
						  //fp2 = fopen("./data/mean88.dat","w");
						  btex_show();
						  break;
				case 8081:  
					      sprintf(btex2,"   4. apply 25kg \n"); 
						  btex_show();
						  break;
				case 8082:
						  terminate_section();
						  break;

				case 9999:  
						  bias_sensor(); 
						  trial=-1;
						  break;
				default: 
						  trial++;
						  break;
			}			
		}
	return false;
}

//*********************************************************************************
// This callback exits from the gtk_main() main loop when the main window is closed
static void destroy_main (GtkWindow *window,	gpointer   user_data)
{
   gtk_widget_destroy (GTK_WIDGET(window));
   gtk_main_quit ();
}

void turn_on_buttons()
{
	gtk_widget_set_sensitive (button1,  true);    
	gtk_widget_set_sensitive (button2,  true);  
	gtk_widget_set_sensitive (button3,  true);    
	gtk_widget_set_sensitive (button4,  true);  
	gtk_widget_set_sensitive (button5,  true);    
	gtk_widget_set_sensitive (button6,  true); 
	gtk_widget_set_sensitive (button7,  true);    
	gtk_widget_set_sensitive (button8,  true); 
	gtk_widget_set_sensitive (button9,  true); 
	gtk_widget_set_sensitive (button10, true);    
	gtk_widget_set_sensitive (button11, true); 
	gtk_widget_set_sensitive (button12, true);    
	gtk_widget_set_sensitive (button13, true); 	
	gtk_widget_set_sensitive (button_next, false); 
	gtk_button_set_label (GTK_BUTTON(button_next), "Acquire data");

	clear_text_buffer(1);
	clear_text_buffer(2);
	sprintf(btex,"Select an acquisition from the menu.\n");
	gtk_label_set_text(GTK_LABEL(label_message),text_buffer);  
}

void turn_off_buttons()
{
	gtk_widget_set_sensitive (button1,  false);    
	gtk_widget_set_sensitive (button2,  false);  
	gtk_widget_set_sensitive (button3,  false);    
	gtk_widget_set_sensitive (button4,  false);  
	gtk_widget_set_sensitive (button5,  false);    
	gtk_widget_set_sensitive (button6,  false); 
	gtk_widget_set_sensitive (button7,  false);    
	gtk_widget_set_sensitive (button8,  false); 
	gtk_widget_set_sensitive (button9,  false); 
	gtk_widget_set_sensitive (button10, false);    
	gtk_widget_set_sensitive (button11, false); 
	gtk_widget_set_sensitive (button12, false);    
	gtk_widget_set_sensitive (button13, false);
	gtk_widget_set_sensitive (button_next, true); 
	gtk_button_set_label (GTK_BUTTON(button_next), "Acquire data");
	
	clear_text_buffer(1);
	clear_text_buffer(2);
}

static int click_next (GtkButton *button,	gpointer   user_data)
{
	int rtrial=remap_trials();
	if (acquire_1000_samples()==false)
	{
		sprintf(btex,"Not enough data acquired. Repeat acquisition.\n");
		gtk_label_set_text(GTK_LABEL(label_message),text_buffer);  
		return -1;
	}

	if (rtrial==-1)
	{
		trial_bias=last_value;
	}

	trial++;
	printf("%d\n",trial); //debug only
	menu();
	return 0;
}

static int click_bx (GtkButton *button,	gpointer val)
{
	int t=*((int*)val);
	current_button_handle=button;
	GdkColor color;
	gdk_color_parse ("green", &color);
	gtk_widget_modify_text (button3, GTK_STATE_NORMAL, &color);
	gtk_widget_modify_base (button3, GTK_STATE_NORMAL, &color);
	gtk_widget_modify_fg (button3, GTK_STATE_NORMAL, &color);
	gtk_widget_modify_bg (button3, GTK_STATE_NORMAL, &color);
	gtk_widget_show (button3);

	current_button_num=t;
	turn_off_buttons();
	trial=t;
	menu();
	return 0;
}

int bi[13]={8020,1001,8010,8030,8040,8050,8060,8070,8080,3001,4001,5001,6001};
void create_menu_buttons()
{
	int x_off_1=10;
	int x_off_2=x_off_1+190;
	int y_off_1=0;
	int y_v=y_off_1;

    button1 = gtk_button_new_with_mnemonic ("1. z+ pointing DOWNwards\n    25kg traction");
	gtk_fixed_put	(GTK_FIXED(fix1), button1 ,x_off_1, y_v);
    g_signal_connect (button1, "clicked", G_CALLBACK (click_bx),&bi[0]);
    gtk_widget_set_size_request     (button1, 161, 40);

	button2 = gtk_button_new_with_mnemonic ("2. z+ pointing DOWNwards\n    5kg torques");
	gtk_fixed_put	(GTK_FIXED(fix1), button2 ,x_off_1, y_v+=50);
    g_signal_connect (button2, "clicked", G_CALLBACK (click_bx),&bi[1]);
    gtk_widget_set_size_request     (button2, 161, 40);

    button3 = gtk_button_new_with_mnemonic ("3. z+ pointing UPwards  \n    25kg compression");
	gtk_fixed_put	(GTK_FIXED(fix1), button3 ,x_off_1, y_v+=50);
    g_signal_connect (button3, "clicked", G_CALLBACK (click_bx),&bi[2]);
    gtk_widget_set_size_request     (button3, 161, 40);

    button4 = gtk_button_new_with_mnemonic ("4. x+ pointing UPwards  \n   25kg");
	gtk_fixed_put	(GTK_FIXED(fix1), button4 ,x_off_1, y_v+=50);
    g_signal_connect (button4, "clicked", G_CALLBACK (click_bx),&bi[3]);
    gtk_widget_set_size_request     (button4, 161, 40);

	button5 = gtk_button_new_with_mnemonic ("5. x- pointing UPwards  \n   25kg");
	gtk_fixed_put	(GTK_FIXED(fix1), button5 ,x_off_1, y_v+=50);
    g_signal_connect (button5, "clicked", G_CALLBACK (click_bx),&bi[4]);
    gtk_widget_set_size_request     (button5, 161, 40);

	button6 = gtk_button_new_with_mnemonic ("6.  1 pointing DOWNwards\n   25kg");
	gtk_fixed_put	(GTK_FIXED(fix1), button6 ,x_off_1, y_v+=50);
    g_signal_connect (button6, "clicked", G_CALLBACK (click_bx),&bi[5]);
    gtk_widget_set_size_request     (button6, 161, 40);

	button7 = gtk_button_new_with_mnemonic ("7.  2 pointing DOWNwards\n   25kg");
	gtk_fixed_put	(GTK_FIXED(fix1), button7 ,x_off_2, y_v=y_off_1);
    g_signal_connect (button7, "clicked", G_CALLBACK (click_bx),&bi[6]);
    gtk_widget_set_size_request     (button7, 161, 40);

    button8 = gtk_button_new_with_mnemonic ("8.  3 pointing DOWNwards\n   25kg");
	gtk_fixed_put	(GTK_FIXED(fix1), button8 ,x_off_2, y_v+=50);
    g_signal_connect (button8, "clicked", G_CALLBACK (click_bx),&bi[7]);
    gtk_widget_set_size_request     (button8, 161, 40);

    button9 = gtk_button_new_with_mnemonic ("9.  4 pointing DOWNwards\n   25kg");
	gtk_fixed_put	(GTK_FIXED(fix1), button9 ,x_off_2, y_v+=50);
    g_signal_connect (button9, "clicked", G_CALLBACK (click_bx),&bi[8]);
    gtk_widget_set_size_request     (button9, 161, 40);

	button10 = gtk_button_new_with_mnemonic ("10. x+ axis pointing UPwards\n   5kg laterals");
	gtk_fixed_put	(GTK_FIXED(fix1), button10 ,x_off_2, y_v+=50);
    g_signal_connect (button10, "clicked", G_CALLBACK (click_bx),&bi[9]);
    gtk_widget_set_size_request     (button10, 161, 40);

	button11 = gtk_button_new_with_mnemonic ("11. y+ axis pointing UPwards\n   5kg laterals");
	gtk_fixed_put	(GTK_FIXED(fix1), button11,x_off_2, y_v+=50);
    g_signal_connect (button11, "clicked", G_CALLBACK (click_bx),&bi[10]);
    gtk_widget_set_size_request     (button11, 161, 40);

	button12 = gtk_button_new_with_mnemonic ("12. x- axis pointing UPwards\n   5kg laterals");
	gtk_fixed_put	(GTK_FIXED(fix1), button12 ,x_off_2, y_v+=50);
    g_signal_connect (button12, "clicked", G_CALLBACK (click_bx),&bi[11]);
    gtk_widget_set_size_request     (button12, 161, 40);

	button13 = gtk_button_new_with_mnemonic ("13. y- axis pointing UPwards\n   5kg laterals");
	gtk_fixed_put	(GTK_FIXED(fix1), button13,x_off_2, y_v+=50);
    g_signal_connect (button13, "clicked", G_CALLBACK (click_bx),&bi[12]);
    gtk_widget_set_size_request     (button13, 161, 40);

	for (int i=0;i<13;i++) button_completed[i]=false;
}
//*********************************************************************************
int main(int argc, char *argv[]) 
{
//    YARP_REGISTER_DEVICES(icubmod)

	yarp::os::Time::turboBoost();

	bool b = expected_values_handler.init("good_vals.txt");
    if (b==false) return -1;

	clear_text_buffer(1);
	clear_text_buffer(2);
    fp=0;
	trial = -1;
#ifdef ATI_SENS
	ATIsens = new devMeasurementFns();
#endif


	s_thread.start();
	

	gtk_init (&argc, &argv);
    //create the main window, and sets the callback destroy_main() to quit
    //the application when the main window is closed
    window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title (GTK_WINDOW (window), "Calibrator V3.0");
	g_signal_connect (window, "destroy",G_CALLBACK (destroy_main), &window);
	gtk_container_set_border_width (GTK_CONTAINER (window), 8);
 
	fix1 = gtk_fixed_new ();
    gtk_container_add	 (GTK_CONTAINER (window), fix1);
	
	int i=0;
	for (i=0; i<6; i++)
	{
		label_read[i]     = gtk_label_new_with_mnemonic	("32000");
		label_expected[i] = gtk_label_new_with_mnemonic ("32000");
		label_error[i]    = gtk_label_new_with_mnemonic ("00000");
		gtk_fixed_put	(GTK_FIXED(fix1), label_read[i] ,400+i*80, 50);
		gtk_fixed_put	(GTK_FIXED(fix1), label_expected[i] ,400+i*80, 100);
		gtk_fixed_put	(GTK_FIXED(fix1), label_error[i] ,400+i*80, 150);
	}
	label_read_title= gtk_label_new_with_mnemonic       ("current values:");
	label_expected_title = gtk_label_new_with_mnemonic  ("expected values:");
	label_error_title = gtk_label_new_with_mnemonic  ("current error (<2000):");
	label_message = gtk_label_new_with_mnemonic			("text");
	gtk_fixed_put	(GTK_FIXED(fix1), label_read_title		  ,400, 50-20);
	gtk_fixed_put	(GTK_FIXED(fix1), label_expected_title	  ,400, 100-20);
	gtk_fixed_put	(GTK_FIXED(fix1), label_error_title		  ,400, 150-20);
	gtk_fixed_put	(GTK_FIXED(fix1), label_message			  ,400, 200);

		button_next = gtk_button_new_with_mnemonic ("Acquire data");
		gtk_fixed_put	(GTK_FIXED(fix1), button_next ,100, 350);
        g_signal_connect (button_next, "clicked", G_CALLBACK (click_next),NULL);
        gtk_widget_set_size_request     (button_next, 170, 40);

	create_menu_buttons();
	turn_on_buttons();

    gtk_window_set_default_size     (GTK_WINDOW (window), 900, 500);
	gtk_widget_set_size_request     (fix1, 900,500);
    gtk_window_set_resizable (GTK_WINDOW (window), false);
	if (!GTK_WIDGET_VISIBLE (window))
        gtk_widget_show_all (window);
    else
    {
        gtk_widget_destroy (window);
        window = NULL;
    }

	START_TIMER
    gtk_main ();

	STOP_TIMER
	s_thread.stop();
	printf("Program ended\n"); 

    return 0;
}
