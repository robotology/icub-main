// own includes
#include <handlocalization.h>

//namespaces

using namespace thesis::IO;
using namespace thesis::gui;
using namespace thesis::files;
using namespace thesis::tools;
using namespace thesis::tracking;
using namespace thesis::controlsprocessing;
using namespace thesis::imageprocessing;
using namespace thesis::imageprocessing::imgproc_helpers;

using namespace std;

// ***************************************************************************

/**
 * Implementation of the handlocalization module
 *
 * This "module" is working on input images and has the aim to acquire a model 
 * the hand.
 *
 */

// *******************************************
//          variable declarations
// *******************************************

// Ports
//const char *in_port_img			= "/handlocalization/images/i";
//const char *in_port_data		= "/handlocalization/motor/i";
//const char *out_port_result		= "/handlocalization/images/o";

const char *in_port_img			= "/i:source/files/images";
const char *in_port_data		= "/i:source/files/data";
const char *out_port_result		= "/o:processed/images/result";

// images
ImageOf<PixelMono> tmpImageMono;

// Pointers
ImageOf<PixelMono>	*thisImgMono_ptr		= NULL;
ImageOf<PixelMono>	*nextImgMono_ptr		= NULL;
ImageOf<PixelRgb>	*tmpImg_ptr			= NULL;

Bottle			*thisData_ptr			= NULL;

// other variables
int		netCounter		= 0;
int		run				= 0;
int		bufferSize		= BUFFERSIZE;
int		vv_timewindow	= TIMEWINDOW;
int		mv_timewindow	= TIMEWINDOW + 100;
int		motionth		= MOTION_THRESHOLD;
double	flowtrigger		= FLOW_TRIGGER;
int		clusternumb		= CLUSTERNUMBER;
double	trackingTrigger = TRACKING_TRIGGER;
int		angularRange	= ANGULAR_RANGE;
int		nofFeatures		= NOF_FEATURES_OPTFLOW;

// *******************************************
//          main
// *******************************************
int main(int argc, char *argv[]) {
	double t1 = Time::now();
	// Initialize the image instances and its pointers
	// Create and establish the Network
	Connection network (netCounter);
	// Create the important input/output (port) objects
	// image input from files
	//ImgInputRobot imageInput (in_port_img);
	ImgInputFile imageInput (in_port_img);

	// data input from files
	//DataInputRobot dataInput (in_port_data);
	DataInputFile dataInput (in_port_data);

	// image outputs
	ImgOutput tv4 (out_port_result);

	// connecting the source and destination ports
	//network.connect("james/cam/left", in_port_img.getPortname());
	//network.connect("james/arm/state:o", in_port_data.getPortname());
	network.connect(tv4.getPortname(), "/i:tv4");

	// Set the pointer to the first image
	tmpImg_ptr = imageInput.readImg();

	// Set pointer to the first motor information array
	thisData_ptr = dataInput.readData();

	// starting the modules
	// Filling the Buffer
	Buffer buffy (&imageInput, &dataInput);

	thisImgMono_ptr = buffy.readImg();
	thisData_ptr	= buffy.readData();

	nextImgMono_ptr = buffy.readImg();
	thisData_ptr	= buffy.readData();

	// Decider module
	Decider dModule;

	// the motion module
	Motion motionModule (&dModule, motionth, flowtrigger);

	// OpticalFlow stored for a period
	Flow opticalFlowModule (thisImgMono_ptr, &dModule, trackingTrigger, nofFeatures, angularRange);

	// Connector for the previous opened modules. for  accessing easily intermodular information  
	Connector cModule (&motionModule, &opticalFlowModule);

	// Motor information cube
	MICube motorinfoModule (thisData_ptr, mv_timewindow);

	// Regions to track (object determination)
	PatchTracker patchTrackingModule (thisImgMono_ptr, &dModule, &cModule, &motorinfoModule, vv_timewindow);

	printf("Info:\t[Application]\t{time needed to start up: %f}\n", Time::now()-t1);

	int limit = TIMEWINDOW + 150;
	string msg = "";
	string param = "";
	string val = "";
	while (true) {
		
		cout << "Your choice can be: \n\t<go>, <reset>, <paramater>, <pause> or <quit>\n";
		cin >> msg;
		cout << "you entered " << msg << endl;
		if (msg == "reset") {
			motionModule.reset();
			opticalFlowModule.reset();
			motorinfoModule.reset();
			patchTrackingModule.reset();
			//printf("dModule %d, %d\n", dModule.decide("tracking"), dModule.decide("flow"));
			dModule.reset();
			//printf("dModule %d, %d\n", dModule.decide("tracking"), dModule.decide("flow"));
			ImageOf<PixelMono> img;
			img.resize(320,240);
			img.zero();
			tv4.write(&img);
			printf("Reset accepted\n");
		}
		else if (msg == "pause") {
			printf("Pause accepted waiting for 5 seconds\n");
			Time::delay(5.0);
		}
		else if (msg == "quit") {
			printf("Quit accepted\n");
			//Time::delay(1.0);
			//exit(0);
			break;
		}
		else if (msg == "parameter") {
			cout << "Paramater change available: \n\t<timewindow>, <flow>, <tracking>\n";
			cin >> param;
			cout << "you entered " << param << endl;
			if (param == "timewindow") {
				int t = 100;
				printf("new time window value, (int) please !\n");
				cin >> t;
				cout << "you entered: " << t << endl;
				mv_timewindow = t + 100;
				vv_timewindow = t;
				motorinfoModule.paramChange(mv_timewindow);
				patchTrackingModule.paramChange(vv_timewindow, clusternumb);
			}
			else if (param == "tracking") {
				cout << "Paramater change available: \n\t<clusternumb>\n";
				cin >> param;
				cout << "you entered: " << param << endl;
				if (param == "clusternumb") {
					int t = 5;
					cout << "for tracking new cluster value, (int) please !\nold value clusternumb " << clusternumb << endl;
					cin >> t;
					cout << "new value = " << t << endl;
					if (t < motorinfoModule.getNofJoints()) {
						clusternumb = t;
					}
					patchTrackingModule.paramChange(vv_timewindow, clusternumb);
				}				
			}
			else if (param == "flow") {
				cout << "Paramater change available: \n\t<trackingtrigger>, <nofFeatures>, <angularRange>\n";
				cin >> param;
				cout << "you entered: " << param << endl;
				if (param == "trackingtrigger") {
					double t = 0.05;
					cout << "for tracking new trigger value, (double) please !\nold value trackingtrigger " << trackingTrigger << endl;
					cin >> t;
					cout << "new value = " << t << endl;
					if (t < 1.0) {
						trackingTrigger = t;
					}
					opticalFlowModule.paramChange(trackingTrigger, nofFeatures, angularRange);
				}
				else if (param == "nofFeatures") {
					int t = 500;
					cout << "for searching features in image new (int) value, please !\nold value nofFeatures " << nofFeatures << endl;
					cin >> t;
					cout << "new value " << t << endl;
					if (t < 1500) {
						nofFeatures = t;
					}
					opticalFlowModule.paramChange(trackingTrigger, nofFeatures, angularRange);
				}
				else if (param == "angularRange") {
					int t = 25;
					cout << "for searching features in image new (int) value, please !\nold value angularRange " << angularRange << endl;
					cin >> t;
					cout << "new value " << t << endl;
					if (t < 45) {
						angularRange = t;
					}
					opticalFlowModule.paramChange(trackingTrigger, nofFeatures, angularRange);
				}
				else {
					cout << "main menu" << endl;
				}
			}
			else if (msg == "cancel") {
				break;
			} 
			else { 
				printf("also ok - nothing\n");
			}
		}
		else if (msg == "go") {
			printf("Go accepted\n");
			limit = UPTOFILE;
			run = 0;
			//printf("runs = %d\n", limit);
			while (run < limit) {
				//printf("run %d:\n------------\n", run);
				if ( run > 0) {
					thisImgMono_ptr = nextImgMono_ptr;
					nextImgMono_ptr = buffy.readImg();
					thisData_ptr	= buffy.readData();
				}

				// Processing the information
				if (dModule.decide("flow") == false) {
					// Motion
					motionModule.detectMotion(thisImgMono_ptr, nextImgMono_ptr);
					tv4.write(thisImgMono_ptr);
				}
				else if (dModule.decide("tracking") == false) {
					// Optical Flow
					opticalFlowModule.updateFlow(thisImgMono_ptr, nextImgMono_ptr);
					tv4.write(thisImgMono_ptr);
				}
				else {
					//printf("should enter here\n");
					motorinfoModule.update(thisData_ptr);
					opticalFlowModule.updateFlow(thisImgMono_ptr, nextImgMono_ptr);
					patchTrackingModule.updatePatches(thisImgMono_ptr);

					if (patchTrackingModule.getTime() == 0) {
						limit = run + vv_timewindow + 1;
					}
					else if (patchTrackingModule.getTime() < vv_timewindow) {
						// Start tracking if all thresholds are satisfied
						tv4.write(thisImgMono_ptr);
					}
					else if  (patchTrackingModule.getTime() == (vv_timewindow)){
						patchTrackingModule.sortOut();
						tv4.write(patchTrackingModule.getTmp());
					}
					else if (patchTrackingModule.getTime() == (vv_timewindow + 1)) {
						tv4.write(patchTrackingModule.getTmp());
					}
					else {
						printf("Info:\t[Application]\t{time needed to end all: %f}\n", Time::now()-t1);
						//printf("");
						run = limit;
						//limit = UPTOFILE;
					}
				}
				run++;
			} // while run
		} // end else if go
		else {
			printf("enter sth again, please\n");
			//Time::delay(5.0);
		}
	}
	printf("Info:\t[Application]\t{time needed to end all: %f}\n", Time::now()-t1);
	//network.disconnect("james/cam/left", in_port_img.getPortname());
	//network.disconnect("james/arm/state:o", in_port_data.getPortname());
	network.disconnect(tv4.getPortname(), "/i:tv4");
	return 0;
}

