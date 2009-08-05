/*
 * Copyright (C) <2006> RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Lars Olsson, and Hatice Kose-Bagci (University of Hertfordshire)
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
 */


#include <String>
#include "InformationDistanceMatrix.h"
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>

#define ONENORM_DISTANCE 1
#define INFORMATION_DISTANCE 2
#define MAX 3

using namespace yarp::os;

BufferedPort<Bottle> dataPortIn;
int adaptive_binning_switch;
int binning_window;
int binning_histogram;


/**
 * This class takes in raw sensor data and calls necessary functions to compute the distance between sensors
 * It includes One-norm distance, and Information distance.
 * It sends out the resultant distance matrix to be used by other modules like Relax algorithm
 */
class ComputeDistance {

public:
	double **distance;
	bool normalise;
	int numSensors;
	int distanceM;

	int maxVal;
	int count;
	int start;
	int stop;

	ComputeDistance(){

	maxVal = 255;
	count = -1;
	start = 0;
	stop = 1000000;

	}

	/**
	 * For reading sensor data from a port
	 * @param numSensors - number of sensors
	 */
	double* getNextData(int numSensors){
		double *data = (double*)malloc(numSensors*sizeof(double));

		Bottle *bot;
			if(count >= start)
			try {
				bot = dataPortIn.read();
			}catch(...){
				exit(1);
			}
		else {
			while(count++ < start){
				try {
					bot=	dataPortIn.read();
				}catch(...){
					exit(1);
				}
			}
		}
		if(bot == NULL)
			return NULL;
		if(bot->get(0).asInt() == 0)
			return NULL;

		for(int i = 1; i <= numSensors; i++){
			if(!normalise){
				data[i-1] = bot->get(i).asDouble();
			}
			else
				data[i-1] = bot->get(i).asDouble() / ((double)maxVal);
		}

		if(count > stop){
			return NULL;
		}
		else{
			return data;
		}
	}

	/**
	* Computes One Norm distance matrix
	* @param numSensors - number of sensors
	*/
	void computeOneNorm(int numSensors){
		printf("computeOnenorm");

		DistanceMatrix *dm = new DistanceMatrix(numSensors);
		double *data = NULL;

		int count = 0;
		while((data = getNextData(numSensors)) != NULL){
			if(count++ != 0 && ((count % 100) == 0))
				printf("%d",count);
			else
				printf(".");


			dm->updateMatrix(data);
		}
		printf("\n");
		dm->normaliseMatrix();
		distance = dm->getNormalizedMatrix();
	}
	/**
	* Computes Information distance matrix
	* @param numSensors - number of sensors
	* @param numBins - number of bins
	*/
	void computeInformationMetric(int numSensors, int numBins){
		printf("computeInformationMetric");

		InformationDistanceMatrix *dm = new InformationDistanceMatrix(numSensors, numBins);

		if (adaptive_binning_switch) {
			dm->setAdaptiveBinning(binning_window, binning_histogram);
		}

		double *data = NULL;
		int count = 0;
		while((data = getNextData(numSensors)) != NULL){
			dm->addData(data);
			if(count++ != 0 && ((count % 100) == 0))
				printf("%d",count);
			else
				printf(".");
		}
		printf("\n");
		dm->updateMatrix();
		distance = dm->getMatrix();
	}
};

	//main
	void main(int argc, char * argv[]){
		Network::init();
		int i,j;

		Property cmdLine;
		cmdLine.fromCommand(argc,argv);

		if (!cmdLine.check("file")) {
			printf("Please call with: --file config.txt\n");
			exit(1);
		}
		ConstString fname = cmdLine.find("file").asString();

		Property config;
		config.fromConfigFile(fname.c_str());

		if (!config.check("PORTS")) {
			printf("Config file needs section [PORTS] with port names defined\n");
			exit(1);
		}

		// data output port name of sensor data source e.g. file/robot etc.
		ConstString sensorInputName = config.findGroup("PORTS").check("sensor_in",Value("/uh/sensor:in")).asString();

		// input port for sensor data
		ConstString dataPortInName = config.findGroup("PORTS").check("data_port_in",Value("/uh/ComputDist/data:in")).asString();

		dataPortIn.setStrict();
		dataPortIn.open(dataPortInName.c_str());

		// output port for distance data
		ConstString outputName = config.findGroup("PORTS").check("compute_dist_out",Value("/uh/ComputDist/distance:out")).asString();

		BufferedPort<Bottle> output;
	    output.open(outputName);

		// input port of next stage (relax)
		ConstString relaxInputName = config.findGroup("PORTS").check("relax_in", Value("/uh/Relax/distance:in")).asString();


		ComputeDistance *cd = new ComputeDistance();

		if (!config.findGroup("GENERAL").check("num_sensors")) {
			fprintf(stderr,"Configuration file expects num_sensors\n");
			exit(-1);
		}
		cd->numSensors = config.findGroup("GENERAL").find("num_sensors").asInt();

		// read type of distance metric 1=ONENORM 2=INFORMATION DISTANCE (default)
		cd->distanceM = config.findGroup("COMPUTE_DIST").check("distance_type",Value(2)).asInt();

		// normalise switch
		int c = config.findGroup("COMPUTE_DIST").check("normalize",Value(0)).asInt();
		if (c)
			cd->normalise = true;
		else
			cd->normalise = false;


		if (!config.findGroup("COMPUTE_DIST").check("num_bins")) {
			fprintf(stderr,"Configuration file expects num_bins\n");
			exit(-1);
		}
		int numBins = config.findGroup("COMPUTE_DIST").find("num_bins").asInt();

		// start/stop (how much data to read)
		cd->start = config.findGroup("COMPUTE_DIST").check("start",Value(0)).asInt();
		cd->stop = config.findGroup("COMPUTE_DIST").check("stop",Value(100)).asInt();

		// for normalization
		cd->maxVal = config.findGroup("COMPUTE_DIST").check("max_val",Value(1)).asInt();

		// allocate the distance matrix memory
		cd->distance = (double**)malloc(cd->numSensors*sizeof(double*));
		for(i= 0; i < cd->numSensors; i++)
			cd->distance[i] = (double*)malloc(cd->numSensors*sizeof(double));

		// get config switches for adaptive binning
		adaptive_binning_switch = config.findGroup("COMPUTE_DIST").check("adaptive_binning",Value(0)).asInt();

		if (adaptive_binning_switch) {
			binning_window = config.findGroup("COMPUTE_DIST").check("binning_window_size",Value(32)).asInt();
			binning_histogram = config.findGroup("COMPUTE_DIST").check("binning_histogram_size",Value(256)).asInt();
		}

		Network::sync(sensorInputName.c_str());

		while( !Network::connect(sensorInputName.c_str(), dataPortInName.c_str(),"tcp", false)){
		 printf("connecting to %s ...\n",sensorInputName.c_str());
		 Time::delay(0.5);
		}

		switch(cd->distanceM) {
		case ONENORM_DISTANCE:
			cd->computeOneNorm(cd->numSensors);
			break;
		case INFORMATION_DISTANCE:
			cd->computeInformationMetric(cd->numSensors, numBins);
			break;
		default:
			printf("Unknown metric - bailing out!");
			exit(1);
		}


		Network::sync(relaxInputName.c_str());

		while( !Network::connect(outputName.c_str(),relaxInputName.c_str(), "tcp", false)){
			 printf("connecting to %s ...\n",relaxInputName.c_str());
			 Time::delay(0.5);
		}

		for(int y = 0; y < cd->numSensors; y++){
			Bottle& bot=output.prepare();
			bot.clear();
			for(int x = 0; x < cd->numSensors; x++){
				bot.addDouble(cd->distance[y][x]);
				}
			output.writeStrict();
			Time::delay(0.05);
		}

		dataPortIn.close();
		output.close();
		for (i=0;i<cd->numSensors;i++)
			free(cd->distance[i]);
		free(cd->distance);
		free(cd);

		Network::fini();
		exit(1);
	}


