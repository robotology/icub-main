// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <stdio.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/iha/debug.h>
#include <iCub/iha/file_utils.h>
#include <iCub/iha/mem_util.h> // from ControlBoardInterfaces.inl
#include <iCub/iha/iha_utils.h>

namespace iCub {
	namespace iha {
		class SensorFWThread;
		class ImageFWThread;
	}
}

using namespace iCub::iha;

using namespace yarp::os;
using namespace yarp::sig;

bool running = true;
// simple signal handler stops out program
void term_sighandler(int sig) {
	fprintf(stderr,"Got term signal\n");
	running = false;
}

char * outputDir;
char * sensorFilename;
char * imageBaseName;

BufferedPort<ImageOf<PixelRgb> > imgPortIn;
BufferedPort<Bottle> dataPortIn;

bool writeDataToFile;
bool writeTimestepImages;
bool writeAllImages;

ImageOf<PixelRgb> *imgIn;
ImageOf<PixelRgb> *currentImg;
yarp::os::Semaphore imageMutex;

/**
 * Thread for writing sensor data to a file
 * Will also write current image if writeTimestepImages=true
 */
class iCub::iha::SensorFWThread : public yarp::os::Thread
{
	public:
		SensorFWThread() {
		}

		~SensorFWThread() {
		}

		void run() {
			while (!isStopping()) {
				b = dataPortIn.read(true);
				if (writeDataToFile) {
					writeToOuputFile(b->toString());
					writeToOuputFile("\n");
				}
				
				// Also write the current image with this timestep
				// as a label
				if (currentImg!=0 && writeTimestepImages) {
					int timestep = b->get(0).asInt();
					char imgfilename[50];
					sprintf(imgfilename,"%s%09d.ppm", imageBaseName, timestep);
					IhaDebug::pmesg(DBGL_DEBUG1,"Saving image %s\n",imgfilename);
					
					imageMutex.wait();
					yarp::sig::file::write(*currentImg,imgfilename);
					imageMutex.post();
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
		Bottle *b;
};

/**
 * Thread for writing images to files
 * If writeAllImages=true this writes out all the images on the imgIn port
 * which means it will write way too many images.
 */
class iCub::iha::ImageFWThread : public yarp::os::Thread
{
	public:
		ImageFWThread() {
			imgno=0;
		}

		~ImageFWThread() {
		}

		void run() {
			while (!isStopping()) {
				imgIn = imgPortIn.read(true);
				
				imageMutex.wait();
				currentImg = imgIn;
				imageMutex.post();
				
				if (imgIn!=0 && writeAllImages) {
					// create a unique file name
					char imgfilename[50];
					sprintf(imgfilename,"%s%06d.jpg", imageBaseName, imgno);
					IhaDebug::pmesg(DBGL_DEBUG1,"Saving image %s\n",imgfilename);
					yarp::sig::file::write(*imgIn,imgfilename);
					imgno++;
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
		int imgno;
};

int main(int argc, char *argv[]) {
	signal(SIGTERM, term_sighandler); // register a SIGTERM handler

	printHeader(argv);
    IhaDebug::setLevel(DBGL_INFO);

	Network::init();
	
	Property cmdLine;
	cmdLine.fromCommand(argc,argv);

	if (!cmdLine.check("file")) {
		fprintf(stderr,"Please call with: --file config.txt\n");
		exit(1);
	}
	ConstString fname = cmdLine.find("file").asString();
	
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
	str = config.check("write_data_to_file", Value("TRUE")).asString();
	writeDataToFile = boolStringTest(str);
	IhaDebug::pmesg(DBGL_INFO,"writeDataToFile = %s\n", writeDataToFile ? "true" : "false");
	
	str = config.check("write_timestamp_images", Value("TRUE")).asString();
	writeTimestepImages = boolStringTest(str);
	IhaDebug::pmesg(DBGL_INFO,"writeTimestepImages = %s\n", writeTimestepImages ? "true" : "false");
	
	str = config.check("write_all_images", Value("FALSE")).asString();
	writeAllImages = boolStringTest(str); 
	IhaDebug::pmesg(DBGL_INFO,"writeAllImages = %s\n", writeAllImages ? "true" : "false");

	//------------------------------------------------------
	// config file read: filenames and directories
	// get the output directory basename
		
	outputDir = allocAndCheck<char> (200);
	sensorFilename  = allocAndCheck<char> (250);
	imageBaseName  = allocAndCheck<char> (250);
	
	if (writeDataToFile || writeTimestepImages || writeAllImages) {
		// create the new directory for data output
		ConstString outDirBase = config.check("out_dir_base",Value("../data")).asString();
		if (!prepareDirectoryNew(outDirBase, outputDir)) {
			fprintf(stderr,"Error creating new data directory\n");
			return false;
		}
		IhaDebug::pmesg(DBGL_STATUS1,"Created new data directory %s\n",outputDir);
	}
		
	if (writeDataToFile) {
		sprintf(sensorFilename,"%s/%s",outputDir, config.check("sensor_filename", Value("sensors.out")).asString().c_str());
		IhaDebug::pmesg(DBGL_INFO,"Writing sensors data to %s\n",sensorFilename);
		// check if the file already exists
		if (fileExists(sensorFilename)) {
			fprintf(stderr,"Error: file %s exists\n",sensorFilename);
			return false;
		}
			
		openOutputFile(sensorFilename);
	}

	if (writeTimestepImages || writeAllImages ) {
		sprintf(imageBaseName, "%s/%s",outputDir, config.check("image_base_name", Value("image")).asString().c_str());
		IhaDebug::pmesg(DBGL_STATUS1,"Images prefix: %s\n",imageBaseName);
	}
		
	//------------------------------------------------------
	// get the basename of ports etc 
	ConstString name = config.check("name",Value("iha")).asString();
	
	//------------------------------------------------------
	// create a port for sensor data
	char portname[30];
	sprintf(portname,"/%s/filewriter/data:in",name.c_str());
	IhaDebug::pmesg(DBGL_INFO,"Reading sensor data from port %s\n",portname);
	
	// open the port
	dataPortIn.open(portname);
	
	// if required we can connect to the sensors
	if (cmdLine.check("connect_to_sensors")) {
		if (connectToParamNonModule(cmdLine,"connect_to_sensors",portname, 0.25, running)) {
			IhaDebug::pmesg(DBGL_INFO,"Connected to Sensors\n");
		}
	}

	//------------------------------------------------------
	// get the port for image data
	char iportname[30];
	sprintf(iportname,"/%s/filewriter/img:in",name.c_str());
	IhaDebug::pmesg(DBGL_INFO,"Reading image data from port %s\n",iportname);
	
	// open the port
	imgPortIn.open(iportname);

	// if required we can connect to the image source
	if (cmdLine.check("connect_to_image")) {
		if (connectToParamNonModule(cmdLine,"connect_to_image",iportname, 0.25, running)) {
			IhaDebug::pmesg(DBGL_INFO,"Connected to Image\n");
		}
	}

	// This thread waits on a port and writes the data there
	// to the file opened above.
	SensorFWThread *sensorFWThread = new SensorFWThread();
	sensorFWThread->start();
	
	// This thread writes all images coming in on the imgIn port
	ImageFWThread *imageFWThread = new ImageFWThread();
	imageFWThread->start();

	while (running) {
		usleep(10);
	}
	
	sensorFWThread->stop();
	imageFWThread->stop();
	
	Network::fini();
	
	closeOutputFile();
	
	checkAndDestroy<char> (outputDir);
	checkAndDestroy<char> (sensorFilename);
	checkAndDestroy<char> (imageBaseName);
	
	return 0;
}
