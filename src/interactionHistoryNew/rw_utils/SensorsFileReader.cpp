// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <stdio.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/iha/debug.h>
#include <iCub/iha/file_utils.h>
#include <iCub/iha/mem_util.h> // from ControlBoardInterfaces.inl
#include <iCub/iha/iha_utils.h>
#include <iostream>

using namespace yarp::os;
using namespace yarp::sig;

const char * dataDir;
char * sensorFilename;
char * imageBaseName;

BufferedPort<ImageOf<PixelRgb> > imgPortOut;
BufferedPort<Bottle> dataPortOut;

bool readSensorData;
bool readTimestepImages;
bool readAllImages;

yarp::os::Semaphore imageMutex;

#define MAX_BUF_SIZE 10000

bool killme=false;

/**
 * Loop for reading sensor data from a file
 */
void mainLoop() {
	int prev_ts = 0;
	ACE_Time_Value last_write;

	killme=false;
	while (!killme) {
		//fprintf(stderr,"loop\n");
		// read a line from the file
		char buffer[MAX_BUF_SIZE];
		if (!readFromInputFile(buffer, MAX_BUF_SIZE)){
			IhaDebug::pmesg(DBGL_INFO, "SensorFRThread: Cannot read from file. Stopping.\n");
			killme=true;
		} else {
			IhaDebug::pmesg( DBGL_DEBUG1, "READ: %s\n",buffer );

			// get the timestep which is the first field in each line
			int timestep = atoi( strtok( buffer, " " ) );
			IhaDebug::pmesg( DBGL_STATUS1, "Timestep %d\n",timestep);

			// work out the delay needed from the last timestep
			// wait the remaining time
			ACE_Time_Value sleep_period;
			if (prev_ts!=0) {
				int gap = timestep - prev_ts; // this is in millisecs

				ACE_Time_Value curr_time = ACE_OS::gettimeofday();
				ACE_Time_Value elapsed_time = curr_time - last_write;
				unsigned long elapsed = elapsed_time.msec(); // should also be in millisecs
				
				int sleep_time = gap - (int)elapsed;

				// the time stamp could be a time number not number of seconds
				// in this case we need to handle that 59->00 etc.
				// couldn't be bothered to put in the month/year transfer
				// actually that would be greater than sizeof(int)
				if (sleep_time > 760000000) sleep_time -= 760000000; // day barrier
				if (sleep_time >   4000000) sleep_time -=   4000000; // hour barrier
				if (sleep_time >     40000) sleep_time -=     40000; // minute barrier
				if (sleep_time > 0) {
					sleep_period.set(0,sleep_time * 1000); // set sleep period in us 1,000,000us = 1s
				}

				ACE_OS::sleep(sleep_period);
			}

			// write the data to the port (with a new timestep?)
			Bottle & b = dataPortOut.prepare(); // Get a message bottle from the port
			b.clear();
			b.addInt(timestep); // original timestep

			IhaDebug::pmesg( DBGL_DEBUG2, "Write data to port\n");
			char *nexttok = NULL;
			nexttok = strtok(NULL, " "); // continue tokenisation where we left off
			while (nexttok!=NULL) {
				b.addDouble(atof(nexttok));
				nexttok = strtok(NULL, " ");
			}
			dataPortOut.writeStrict();

			last_write = ACE_OS::gettimeofday();
			prev_ts=timestep;
							
			// extra slow down
			//sleep_period.set(0,1000000);
			//ACE_OS::sleep(sleep_period);

			if (readTimestepImages) {
				// get the approprite image if it exists
				char imagefname[100];
				sprintf(imagefname, "%s%09d.ppm",imageBaseName,timestep);
				IhaDebug::pmesg(DBGL_DEBUG2, "Reading image %s\n",imagefname);
				ImageOf<PixelRgb> & img = imgPortOut.prepare();
				if (yarp::sig::file::read(img, imagefname)) {
					// write it on the image port
					imgPortOut.write(); // the true flag ensures it waits till it can write the image
				} else {
					IhaDebug::pmesg(DBGL_DEBUG1,"Error reading image %s\n",imagefname);
					imgPortOut.write(); // the true flag ensures it waits till it can write the image
				}
			}

		} // if read
	} // while
	fprintf(stderr,"sensors_file_reader: Thread::run() ended.\n");
}

// simple signal handler stops out program
void term_sighandler(int sig) {
	fprintf(stderr,"sensors_file_reader: Got term signal (%d).\n",sig);
	killme=true;
}

// main
int main(int argc, char *argv[]) {
	signal(SIGTERM, term_sighandler); // register a SIGTERM handler
	//signal(SIGHUP, term_sighandler); // register a SIGHUP handler

	printHeader(argv);
    IhaDebug::setLevel(DBGL_STATUS1);

	Network::init();
	
	Property cmdLine;
	cmdLine.fromCommand(argc,argv);

	if (!cmdLine.check("file")) {
		fprintf(stderr,"Please call with: --file config.txt\n");
		exit(1);
	}
	ConstString fname = cmdLine.find("file").asString();
	
	if (!cmdLine.check("dir")) {
		fprintf(stderr,"Please call with: --dir <path-to-data-dir>\n");
		exit(1);
	}
	ConstString ddir = cmdLine.find("dir").asString();
	dataDir = ddir.c_str();

	Property config;
	config.fromConfigFile(fname);
	if (config.check("dbg")) { IhaDebug::setLevel(config.find("dbg").asInt()); }
	if (cmdLine.check("dbg")) { IhaDebug::setLevel(cmdLine.find("dbg").asInt()); }
 	ACE_OS::fprintf(stderr, "Debug level : %d\n",IhaDebug::getLevel());
	
	//------------------------------------------------------
	// config file read: switches
	// image data can only be written when sensor data is also being written
	// get config flag for controlling image file output
	ConstString str;
	str = config.check("read_sensor_data", Value("TRUE")).asString();
	readSensorData = boolStringTest(str);
	IhaDebug::pmesg(DBGL_STATUS1,"readSensorData = %s\n", readSensorData ? "true" : "false");
	
	str = config.check("read_timestamp_images", Value("TRUE")).asString();
	readTimestepImages = boolStringTest(str);
	IhaDebug::pmesg(DBGL_STATUS1,"readTimestepImages = %s\n", readTimestepImages ? "true" : "false");
	
	str = config.check("read_all_images", Value("FALSE")).asString();
	readAllImages = boolStringTest(str); 
	IhaDebug::pmesg(DBGL_STATUS1,"readAllImages = %s\n", readAllImages ? "true" : "false");

	//------------------------------------------------------
	// config file read: filenames and directories
	// get the output directory basename
		
	sensorFilename	= allocAndCheck<char> (250);
	imageBaseName	= allocAndCheck<char> (250);
	
	// check the data directory exists
	if (!dirExists(dataDir)) {
		fprintf(stderr, "Error: dir %s does not exist\n",dataDir);
		exit(1);
	}
		
	if (readSensorData) {
		sprintf(sensorFilename,"%s/%s",dataDir, config.check("sensor_filename", Value("sensors.out")).asString().c_str());
		if (!fileExists(sensorFilename)) {
			fprintf(stderr, "Error: sensor file %s does not exist\n",sensorFilename);
			exit(1);
		}
		IhaDebug::pmesg(DBGL_INFO,"Reading sensors data from %s\n",sensorFilename);
			
		openInputFile(sensorFilename);
	}

	if (readTimestepImages || readAllImages ) {
		sprintf(imageBaseName, "%s%s",dataDir, config.check("image_base_name", Value("image")).asString().c_str());
		IhaDebug::pmesg(DBGL_STATUS1,"Images prefix: %s\n",imageBaseName);
	}
		
	//------------------------------------------------------
	// get the basename of ports etc 
	ConstString name = config.check("name",Value("iha")).asString();
	
	//------------------------------------------------------
	// create a port for sensor data
	char portname[30];
	sprintf(portname,"/%s/filereader/data:out",name.c_str());
	IhaDebug::pmesg(DBGL_INFO,"Writing sensor data to port %s\n",portname);
	
	//------------------------------------------------------
	// get the port for image data
	char iportname[30];
	sprintf(iportname,"/%s/filereader/img:out",name.c_str());
	IhaDebug::pmesg(DBGL_INFO,"Writing image data to port %s\n",iportname);
	
	// open the ports
	dataPortOut.open(portname);
	dataPortOut.setStrict();
	imgPortOut.open(iportname);
	
	// if required we can connect to the data store
	if (cmdLine.check("connect_to_ds")) {
		ConstString dsport = cmdLine.find("connect_to_ds").asString();
		IhaDebug::pmesg(DBGL_INFO,"Waiting for data store port %s ",dsport.c_str());

		while(!Network::sync(dsport.c_str())) {
				Time::delay(0.25);
				IhaDebug::pmesg(DBGL_INFO,".");
		}
		IhaDebug::pmesg(DBGL_INFO," ready. Now connecting ");
		if (!Network::connect(portname,cmdLine.find("connect_to_ds").asString().c_str())) {
			IhaDebug::pmesg(DBGL_INFO,"Error.\n");
		}
		IhaDebug::pmesg(DBGL_INFO," connected.\n");
	}

	// This thread reads the sensor file and writes the data in 
	// a time controlled manner to the port above
	
	mainLoop();
	
	IhaDebug::pmesg(DBGL_STATUS1,"sensors_file_reader: Closing input file\n");
	closeInputFile();

	dataPortOut.close();
	imgPortOut.close();
	
	IhaDebug::pmesg(DBGL_STATUS1,"sensors_file_reader: cleanp\n");
	checkAndDestroy<char> (sensorFilename);
	checkAndDestroy<char> (imageBaseName);

	Network::fini();

	IhaDebug::pmesg(DBGL_INFO,"sensors_file_reader: end.\n");

	return 0;
}
