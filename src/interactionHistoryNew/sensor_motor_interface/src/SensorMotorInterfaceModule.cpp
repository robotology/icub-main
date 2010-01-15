#include <iCub/iha/SensorMotorInterfaceModule.h>

#include <iCub/iha/iha_utils.h>
#include <iCub/iha/image_utils.h>

#include <iCub/iha/FaceDetectLoop.h>
#include <iCub/iha/ImageReadLoop.h>
#include <iCub/iha/GazeReadLoop.h>
#include <iCub/iha/SoundSensorReadLoop.h>

using namespace iCub::iha;

/**
 * @addtogroup icub_iha2_SensorMotorInterface
 *

This module collects the following data from supporting IHA processes:
#- the \ref icub_iha2_IcubControl process
  - provides the timestep to be used as the timestep for the output.
  - The rate of the output from the sensor motor interface process should be the same as the rate of the IcubControl encoders value process.
  - unnormalized encoders values from the robot motors
  - the current action number
#- the robot camera images port
  - These are processed to produce "sensor" values by pixelating the image and providing a "luminance" value for each larger pixel.
  - Luminance is calculated as 0.3 R + 0.59 G + 0.11 B. See the function imageToSensorArray()
  - the pixelation detail is controlled by the parameters \c --num_image_sensors_x and \c --num_image_sensors_y
#- the \ref icub_iha2_IhaFaceDetect process
  - although this receives the coordinate of the face, the output is either 0="no face" or 1="face detected"
#- the \ref icub_iha2_MobileEye process
  - although this receives the coordinates for gaze direction and robot face location in the scene image. The output is either 0="person not looking at robot's face" or 1="person looking at robot's face"
#- the \ref icub_iha_SoundSensor process
  - value of sound intensity between [0,1]
#- the \ref icub_iha2_AudioAnalyser process
 - receives and record the number of beats currently detected 

This process passes the sensor data on to \ref icub_iha2_Dynamics, where the values are
used to compute the current reward. Special sensor data that has been specified to be
relevant to the drumming and peek-a-boo turn-taking tasks are passed to \ref icub_iha2_Memory
for processing.


\section lib_sec Libraries
- YARP libraries.
- IHA Debug Library

\section parameters_sec Parameters
\verbatim
--dbg [INT]   : debug printing level
--name [STR]  : process name for ports
--file [STR]  : config file

--connect_to_image [STR]       : connect to specified port for images
--connect_to_coords [STR]      : connect to specified port for detected face coordinates
--connect_to_gaze_coords [STR]      : connect to specified port for gaze coordinates
--connect_to_soundsensor [STR] : connect to specified port for sound sensor value
--connect_to_encoders [STR]    : connect to specified port for encoders

 --num_image_sensors_x [INT] : image pixelation
 --num_image_sensors_y [INT] : image pixelation
 --echo_output [STR]          : TRUE/FALSE print sensor output
 --gaze_input [STR]          : TRUE/FALSE use gaze sensor
 --beat_input [STR]          : TRUE/FALSE use audio analyser
\endverbatim

\section portsa_sec Ports Accessed
 - /iCub/cam/left (or any other camera image port)
 - action:cmd - action output from \ref icub_iha2_IcubControl 
 - encoders:out - encoder output from \ref icub_iha2_IcubControl
 - facedetect:coords - face coords from \ref icub_iha2_IhaFaceDetect
 - mobileeye:coords - gaze coords from \ref icub_iha2_MobileEye
 - sndsensor:out - beat message from \ref icub_iha2_AudioAnalyser
 - sndsensor:out - sound value from \ref icub_iha_SoundSensor

- monitor:in - status information collection port from \ref icub_iha_StatusMonitor

\section portsc_sec Ports Created
 - sensor:out - consolidated sensor stream
 - memsensor:out - sensor data for short term memory
 
 - image:in  - to recieve images
 - facecoords:in - to receive face detect coords
 - gazecoords:in - to receive gaze and visual attention coords
 - soundsensor:in - to receive sound intensity value
 - beat:in - to receive beats from audio analyser
 - encoders:in - to recieve motor positions

 - status:out - to write status info for monitor process
 - quit  - module quit port

\section conf_file_sec Configuration Files
conf/ihaSensorMotorInterface.ini

Sample INI file:
\verbatim
name iha
dbg 40

############################################################
# section has the names of the sensors and the hi/lo range
# to allow binning calculations
# remember to put reward and action as last two items

SENSORS HEAD_PITCH HEAD_YAW HEAD_PAN EYES_UD EYES_RL EYES_CD LSH_ROT LSH_ELV LSH_TWST LELB_FLX LELB_TWST LWR_ABD LWR_FLX LDIG_1 LDIG_2 LDIG_3 LDIG_4 LDIG_5 LDIG_6 LDIG_7 LDIG_8 LDIG_9 RSH_ROT RSH_ELV RSH_TWST RELB_FLX RELB_TWST RWR_ABD RWR_FLX RDIG_1 RDIG_2 RDIG_3 RDIG_4 RDIG_5 RDIG_6 RDIG_7 RDIG_8 RDIG_9 FACE SOUNDS GAZE BEAT DRUM_MEM HIDE_MEM ACTION REWARD

#          0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36  37 38  39  40  41  42 43 44 45

LIMIT_HI  30  60  55  15  52  45  90 161 100 106  90  10  40  30 105  90  90  90  90  90  90 120  90 161 100 106  90  10  40  30 105  90  90  90  90  90  90 120  1   1	 1   10	 1  1  21 1
LIMIT_LO -40 -70 -55 -35 -50  -1 -96  -5 -39  -2 -90 -90 -20 -10 -15  -2  -2  -2  -2  -2  -2  -2 -96  -5 -39  -2 -90 -90 -20 -10 -15  -2  -2  -2  -2  -2  -2  -2  0   0	 0   0	 -1 -1 0  0

ts_offset 1

num_encoders 38
face_offset 38
sound_offset 39
gaze_offset 40
beat_offset 41
action_offset 42
reward_offset 43
insert_offset 42


#Note that joint names and their corresponding motor indices
#are listed in iha_actiondefs.ini. Not all motor encoders are
#in the sensor list


###########################################################
# For image pixelation
num_image_sensors_x 8
num_image_sensors_y 8
#
###########################################################
\endverbatim


\section tested_os_sec Tested OS
Linux

\section example_sec Example Instantiation of the Module
ihaNewSensorMotorInterface --name /iha/sm --file /usr/local/src/robot/iCub/app/ihaNew/conf/ihaSensorMotorInterface.ini --connect_to_image /icub/cam/left --connect_to_face_coords /iha/fd/facedetect:coords --connect_to_gaze_coords /iha/me/mobileeye:coords --connect_to_soundsensor /iha/sound/sndsensor:out --connect_to_encoders /iha/controller/encoders:out --gaze_input TRUE

See also the script $ICUB_ROOT/app/ihaNew/iha_smi.sh

\author Assif Mirza and Frank Broz

Copyright (C) 2009 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at \in src/interactionHistoryNew/sensor_motor_interface/src/SensorMotorInterfaceModule.cpp.
*/

SensorMotorInterfaceModule::SensorMotorInterfaceModule(){
    imageSensorarray=NULL;
}

SensorMotorInterfaceModule::~SensorMotorInterfaceModule(){ 
    if (imageSensorarray!=NULL) delete[] imageSensorarray;
}


bool SensorMotorInterfaceModule::open(Searchable& config){
   
	if (config.check("dbg")) { IhaDebug::setLevel(config.find("dbg").asInt()); }
 	ACE_OS::fprintf(stderr, "Debug level : %d\n",IhaDebug::getLevel());

    if (config.check("help","if present, display usage message")) {
		cerr << "Usage : " << "\n"
		<< "------------------------------------------" << "\n"
		<< "  --dbg [INT]   : debug printing level" << "\n"
		<< "  --name [STR]  : process name for ports" << "\n"
		<< "  --file [STR]  : config file" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
        << "  --connect_to_image [STR]       : connect to specified port for images" << "\n"
        << "  --connect_to_coords [STR]      : connect to specified port for detected face coordinates" << "\n"
        << "  --connect_to_gaze_coords [STR]      : connect to specified port for gaze coordinates" << "\n"
        << "  --connect_to_soundsensor [STR] : connect to specified port for sound sensor value" << "\n"
        << "  --connect_to_encoders [STR]    : connect to specified port for encoders" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
        << "  --num_image_sensors_x [INT] : image pixelation" << "\n"
        << "  --num_image_sensors_y [INT] : image pixelation" << "\n"
        << "  --echo_output [STR]          : TRUE/FALSE print sensor output" << "\n"
        << "  --gaze_input [STR]          : TRUE/FALSE use gaze sensor" << "\n"
        << "  --beat_input [STR]          : TRUE/FALSE use audio analyser" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
		<< "\n";
        return false;
    }

    bool ok = true;

	//------------------------------------------------------
    // Read parameters
    //
	echo_output = boolStringTest(config.check("echo_output",Value("FALSE")).asString());
	IhaDebug::pmesg(DBGL_INFO,"echo_output %s\n",echo_output?"TRUE":"FALSE");


	use_gaze = boolStringTest(config.check("gaze_input",Value("FALSE")).asString());
	IhaDebug::pmesg(DBGL_INFO,"gaze_input %s\n",use_gaze?"TRUE":"FALSE");

    use_beats = boolStringTest(config.check("beat_input",Value("FALSE")).asString());
	IhaDebug::pmesg(DBGL_INFO,"beat_input %s\n",use_beats?"TRUE":"FALSE");

	// image pixelation
	num_image_sensors_x = config.check("num_image_sensors_x",Value(8)).asInt();
	num_image_sensors_y = config.check("num_image_sensors_y",Value(8)).asInt();
	IhaDebug::pmesg(DBGL_INFO,"num_image_sensors x/y : %d/%d\n",num_image_sensors_x,num_image_sensors_y);

    // allocate the memory for the sensor array
    imageSensorarray=new int[num_image_sensors_x*num_image_sensors_y];
    memset(imageSensorarray,0,sizeof(int)*num_image_sensors_x*num_image_sensors_y);

  

	//------------------------------------------------------
    // Main Sensor Output Port
    ConstString sensorOutputPortName = getName("sensor:out");
	// open the sensor output port 
	IhaDebug::pmesg(DBGL_INFO,"Writing sensor output to port %s\n",sensorOutputPortName.c_str());
	sensorOutputPort.open(sensorOutputPortName.c_str());
	//------------------------------------------------------

	//------------------------------------------------------
    // Sensor Output Port to Short Term Memory
    ConstString memOutputPortName = getName("memsensor:out");
	// open the memory output port 
	IhaDebug::pmesg(DBGL_INFO,"Writing memory sensor output to port %s\n",memOutputPortName.c_str());
	memOutputPort.open(memOutputPortName.c_str());
	//------------------------------------------------------


	//------------------------------------------------------
	// create names of the sensor ports
    ConstString imagePortName = getName("image:in");
    ConstString faceCoordsPortName = getName("facecoords:in");
    ConstString gazeCoordsPortName = getName("gazecoords:in");
    ConstString soundSensorPortName = getName("soundsensor:in");
    ConstString beatPortName = getName("beat:in");
    ConstString sensorMotorPortName = getName("encoders:in");

	//------------------------------------------------------
	// open the ports
	imagePort.open(imagePortName.c_str());
	faceCoordsPort.open(faceCoordsPortName.c_str());
    if (use_gaze)
        gazeCoordsPort.open(gazeCoordsPortName.c_str());
    if (use_beats)
        beatPort.open(beatPortName.c_str());
	soundSensorPort.open(soundSensorPortName.c_str());
	sensorMotorPort.open(sensorMotorPortName.c_str());

	//------------------------------------------------------
    // Create the sensor thread objects
	imageReadLoop = new ImageReadLoop (imagePort);
	faceDetectLoop = new FaceDetectLoop (faceCoordsPort);
    if (use_gaze)
        gazeReadLoop = new GazeReadLoop (gazeCoordsPort);
    if (use_beats)
        beatReadLoop = new BeatReadLoop (beatPort);
	soundSensorReadLoop = new SoundSensorReadLoop (soundSensorPort);

	//------------------------------------------------------
	// start the sensor threads
	imageReadLoop->start();
	faceDetectLoop->start();
    if(use_gaze) {
        gazeReadLoop->start();
    }
    if(use_beats) {
        beatReadLoop->start();
    }
	soundSensorReadLoop->start();

	//------------------------------------------------------
	// make automatic connections
	if (config.check("connect_to_image")) {
		if (connectToParam(config,"connect_to_image",imagePortName.c_str(), 0.25, this)) {
			IhaDebug::pmesg(DBGL_INFO,"Connected to Image\n");
        } else {
            ok = false;
        }
	}
	if (config.check("connect_to_face_coords")) {
		if (connectToParam(config,"connect_to_face_coords",faceCoordsPortName.c_str(), 0.25, this)) {
            IhaDebug::pmesg(DBGL_INFO,"Connected to Face Coords\n");
        } else {
            ok = false;
        }
	}
    if(use_gaze) {
        if (config.check("connect_to_gaze_coords")) {
            if (connectToParam(config,"connect_to_gaze_coords",gazeCoordsPortName.c_str(), 0.25, this)) {
                IhaDebug::pmesg(DBGL_INFO,"Connected to Gaze Coords\n");
            } else {
                ok = false;
            }
        }
    }
    if(use_beats) {
        if (config.check("connect_to_beat")) {
            if (connectToParam(config,"connect_to_beat",beatPortName.c_str(), 0.25, this)) {
                IhaDebug::pmesg(DBGL_INFO,"Connected to audio analyser\n");
            } else {
                ok = false;
            }
        }
    }

	if (config.check("connect_to_soundsensor")) {
		if (connectToParam(config,"connect_to_soundsensor",soundSensorPortName.c_str(), 0.25, this)) {
			IhaDebug::pmesg(DBGL_INFO,"Connected to sound sensor\n");
        } else {
            ok = false;
        }
	}
	if (config.check("connect_to_encoders")) {
		if (connectToParam(config,"connect_to_encoders",sensorMotorPortName.c_str(), 0.25, this)) {
			IhaDebug::pmesg(DBGL_INFO,"Connected to encoders\n");
        } else {
            ok = false;
        }
	}
	//------------------------------------------------------
    

    // Create a port to write status to
    ConstString statusPortName = getName("status:out");
    statusPort.open(statusPortName.c_str());


    ok &= quitPort.open(getName("quit"));
    attach(quitPort, false);
    //attach(quitPort);
    //attachTerminal();
    return ok;
}

bool SensorMotorInterfaceModule::close(){
    fprintf(stderr,"faceCoordsPort.close();\n");
    faceCoordsPort.close();
    if (use_gaze) {
        fprintf(stderr,"gazeCoordsPort.close();\n");
        gazeCoordsPort.close();
    }
    if (use_beats) {
        fprintf(stderr,"beatPort.close();\n");
        beatPort.close();
    }
	fprintf(stderr,"imagePort.close();\n");
	imagePort.close();
	fprintf(stderr,"soundSensorPort.close();\n");
	soundSensorPort.close();
    fprintf(stderr,"sensorMotorPort.close();\n");
    sensorMotorPort.close();
    fprintf(stderr,"statusPort.close();\n");
    statusPort.close();
    fprintf(stderr,"sensorOutputPort.close();\n");
    sensorOutputPort.close();
    fprintf(stderr,"memOutputPort.close();\n");
    memOutputPort.close();
    return true;
}

bool SensorMotorInterfaceModule::interruptModule(){

    fprintf(stderr,"sensorOutputPort.interrupt();\n");
    sensorOutputPort.interrupt();
    fprintf(stderr,"memOutputPort.interrupt();\n");
    memOutputPort.interrupt();
    fprintf(stderr,"faceCoordsPort.interrupt();\n");
    faceCoordsPort.interrupt();
    if(use_gaze) {
        fprintf(stderr,"gazeCoordsPort.interrupt();\n");
        gazeCoordsPort.interrupt();
    }
    if(use_beats) {
        fprintf(stderr,"beatPort.interrupt();\n");
        beatPort.interrupt();
    }
	fprintf(stderr,"imagePort.interrupt();\n");
	imagePort.interrupt();
	fprintf(stderr,"soundSensorPort.interrupt();\n");
	soundSensorPort.interrupt();
    fprintf(stderr,"sensorMotorPort.interrupt();\n");
    sensorMotorPort.interrupt();
    fprintf(stderr,"statusPort.interrupt();\n");
    statusPort.interrupt();
    return true;
}

bool SensorMotorInterfaceModule::updateModule(){

    IhaDebug::pmesg(DBGL_DEBUG1,"in updateModule \n");
    // Read the encoders data
    Bottle* smData = sensorMotorPort.read();

    // create an output bottle for data to the motivation dynamics
    Bottle outData;
    //create an output bottle for data to the short term memory
    Bottle outMemData;

    // timestep is first
    Value ts=Value(smData->get(0));
    outData.addInt(ts.asInt());
    writeStatus(statusPort,"SensMotIf","TS",ts.asInt());
    
    // copy all the encoders data except the last item 
    //(which is the current action) to our output
    for (int i=1;i<smData->size()-1;i++) {
        Value item = Value(smData->get(i));
        outData.addDouble(item.asDouble());
    }
    
    // face detect - just output the first item which is a boolean detect/not
    outMemData.addString("face");
    if (faceDetectLoop->getFaceState()) {
        outData.addDouble(1);
        outMemData.addDouble(1);
    } else {
        outData.addDouble(0);
        outMemData.addDouble(0);
    }
    // value of the sound sensor
    outMemData.addString("sound");
    outData.addDouble(soundSensorReadLoop->getCurrentSoundSensor());
    outMemData.addDouble(soundSensorReadLoop->getCurrentSoundSensor());

    //get something from the gaze tracker here
    if(use_gaze)
        outData.addDouble(gazeReadLoop->getCurrentGaze());
    else
        outData.addDouble(0.0);
    
    //get something from the audio analyser
    outMemData.addString("beat");
    if(use_beats){ 
	double b = double(beatReadLoop->getCurrentBeat());
        outMemData.addDouble(b);
        outData.addDouble(b);
    } else {
      outMemData.addDouble(0);
      outData.addDouble(0);
    }


    // current action value
    outMemData.addString("action");
    Value ca=Value(smData->get(smData->size()-1));
    outData.addDouble(ca.asDouble());
    outMemData.addDouble(ca.asDouble());

    // image sensors last
    yarp::sig::ImageOf< yarp::sig::PixelRgb >* ci = imageReadLoop->aquireCurrentImage();
    if (ci==NULL) {
        for (int i=0;i<num_image_sensors_x*num_image_sensors_y;i++) {
            outData.addDouble(0);
        }
    } else 	{
        imageToSensorArray(ci, imageSensorarray, num_image_sensors_x,num_image_sensors_y);
        for (int i=0;i<num_image_sensors_x*num_image_sensors_y;i++) {
            outData.addDouble(imageSensorarray[i]);
        }
    }
    imageReadLoop->releaseCurrentImage();

    // write to the output port
    sensorOutputPort.write(outData);
    if (echo_output) {
        IhaDebug::pmesg(DBGL_STATUS1,"%s\n",outData.toString().c_str());
    }

    //IhaDebug::pmesg(DBGL_INFO,"%s\n",outMemData.toString().c_str());
    // write to the stm output port
    memOutputPort.write(outMemData);
    if (echo_output) {
        IhaDebug::pmesg(DBGL_STATUS1,"%s\n",outMemData.toString().c_str());
    }

    if (IhaDebug::getLevel()==DBGL_STATUSLINE) {
        IhaDebug::pmesg(DBGL_STATUSLINE,"TS: %d \r",ts.asInt());
    } else {
        //IhaDebug::pmesg(DBGL_STATUS1,"TS: %d Reward: %f\n",ts.asInt(),rewardReadLoop->getCurrentReward());
        IhaDebug::pmesg(DBGL_STATUS1,"TS: %d \n",ts.asInt());
    }

    return true;
}

bool SensorMotorInterfaceModule::respond(const Bottle &command,Bottle &reply){
        
    fprintf(stderr,"SensorMotorInterfaceModule::respond %s\n",command.toString().c_str());

    if (command.toString()=="quit") {
        interruptModule();
        yarp::os::Time::delay(1);
        close();
        yarp::os::Time::delay(10);
    }

    return false;
} 	
