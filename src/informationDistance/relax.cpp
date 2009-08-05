/*
 * Copyright (C) <2007> RobotCub Consortium, European Commission FP6 Project IST-004370
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

/*
 *  Implements the relaxation algorithm presented in:
 *
 * @article{Olsson2006c,
 *   Author="Olsson, L. and Nehaniv, C. L. and Polani, D.",
 *   title="From Unknown Sensors and Actuators to Actions Grounded in Sensorimotor Perceptions",
 *   journal="Connection Science",
 *   volume="18",number="2",year="2006"
 * }
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "DistanceMatrix.h"
#include <yarp/os/Property.h>
#include <yarp/os/all.h>
#include <yarp/os/BufferedPort.h>

#include <iostream>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>


using namespace std;
using namespace yarp::os;

#define EUCLIDIAN_DISTANCE(x1, y1, x2, y2) (sqrt(((x1 - x2) * (x1 - x2)) + \
                         ((y1 - y2) * (y1 - y2))))

#define RELAX_MAX 30

static int num_sensors = -1;

static double *diff = NULL;
static double *points = NULL;
BufferedPort<Bottle> dataPortIn;
BufferedPort<Bottle> dataPortOut;
BufferedPort<Bottle> dataPortOut1;

/**
 * For reading distance matrix data from a port
 * @param numSensors - number of sensors
 */
void getDiffMatrix(int numSensors)
{
		for (int i = 0; i < numSensors; i++){
			Bottle *bot = dataPortIn.read();
			for (int j = 0; j < numSensors; j++){
				diff[i * num_sensors + j] = bot->get(j).asDouble();
			}
		}
}

/**
 * For writing the result of the relaxation algorithm (points) to a port
 * @param numSensors - number of sensors
 */
void printResult(int numSensors){
		for (int i = 0; i < numSensors; i++){
			Bottle& bot=dataPortOut.prepare();
			bot.clear();
			bot.addDouble(points[i * 2 + 0]);
			bot.addDouble(points[i * 2 + 1]);
			dataPortOut.writeStrict();
			Time::delay(0.05);
		}
}

/**
 * For writing the map to a port
 * @param numSensors - number of sensors
 * @param map - map of the sensors
 */
void printMap(int numSensors, int **map){
		int npoints = sqrt(numSensors);
		for (int i = 0; i < npoints; i++){
			Bottle &bot = dataPortOut1.prepare();
			bot.clear();
			for (int j = 0; j < npoints; j++){
				bot.addInt(map[i][j]);
			}

			dataPortOut1.writeStrict();
			Time::delay(0.05);
		}
}

/**
 * Implements the relaxation algorithm
 */
static void relax()
{
	int i, j;
	double *forceX = (double *)malloc(num_sensors * sizeof(double));
	double *forceY = (double *)malloc(num_sensors * sizeof(double));
	double force = 0.0;
	double dist = 0.0;


	for(i = 0; i < num_sensors; i++){
		forceX[i] = 0.0;
		forceY[i] = 0.0;
		for(j = 0; j < num_sensors;j++){
			if(j != i){
				dist = EUCLIDIAN_DISTANCE(points[i * 2 + 0],
							  points[i * 2 + 1],
							  points[j * 2 + 0],
							  points[j * 2 + 1]);

				if(dist != 0.0)
					force =(dist - diff[i * num_sensors + j]) / dist;
				else {
					force = diff[i * num_sensors + j];
				}

				forceX[i] = forceX[i] + (force * (points[j * 2 + 0] -
								  points[i * 2 + 0]));
				forceY[i] = forceY[i] + (force * (points[j * 2 + 1] -
								  points[i * 2 + 1]));
			}
		}
	}

	for(i = 0; i < num_sensors; i++){
		points[i * 2 + 0] = points[i * 2 + 0] + 0.002 * forceX[i];
		points[i * 2 + 1] = points[i * 2 + 1] + 0.002 * forceY[i];
	}

	free(forceX);
	free(forceY);
}

/*main*/
int main(int argc, char * argv[])
{
	//int layout = 0
	int i, j, steps = -1;
//	char *di = NULL, *po = NULL;
	double min = 0.0;

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

		// input port of next stage (relax)
		ConstString relaxInputName = config.findGroup("PORTS").check("relax_in", Value("/uh/Relax/distance:in")).asString();
		dataPortIn.open(relaxInputName);

		//output port for points -- result from relax algorithm
		ConstString pointOutputPortName = config.findGroup("PORTS").check("points_out",Value("/uh/Relax/points:out")).asString();
		dataPortOut.open(pointOutputPortName);

		//output port for map
		ConstString mapOutputPortName = config.findGroup("PORTS").check("map_out",Value("/uh/Relax/map:out")).asString();
		dataPortOut1.open(mapOutputPortName);

		// input port for points
		ConstString pointsInName = config.findGroup("PORTS").check("points_in",Value("/uh/points:in")).asString();

		// input port for map
		ConstString mapInName = config.findGroup("PORTS").check("map_in",Value("/uh/map:in")).asString();

		if (!config.findGroup("GENERAL").check("num_sensors")) {
			fprintf(stderr,"Configuration file expects num_sensors\n");
			exit(-1);
		}
		num_sensors = config.findGroup("GENERAL").find("num_sensors").asInt();

		// number of steps (how many times to run the relax algorithm)
		steps = config.findGroup("RELAX").check("steps",Value(1)).asInt();

	srand((unsigned)time(NULL));


	while( !Network::connect(pointOutputPortName.c_str(),pointsInName.c_str(), "tcp", false)){
		 printf("connecting to %s ...\n",pointsInName.c_str());
		 Time::delay(0.5);
	}
	while( !Network::connect(mapOutputPortName.c_str(), mapInName.c_str(),"tcp", false)){
		 printf("connecting to %s ...\n",mapInName.c_str());
		 Time::delay(0.5);
	}

	diff = (double *)malloc(num_sensors * num_sensors * sizeof(double));
	points = (double *)malloc(num_sensors * 2 * sizeof(double));

	for(i = 0; i < num_sensors; i++){
		for(j = 0; j < num_sensors; j++){
			diff[i * num_sensors + j] = 0.0;
		}
	}

	getDiffMatrix(num_sensors);

	/* just put in random values */
	for(i = 0;i < num_sensors;i++)
	{
		points[i * 2 + 0] = rand() / (double)RAND_MAX;
		points[i * 2 + 1] = rand() / (double)RAND_MAX;
	}


	fprintf(stderr, "\nrelaxing...\n");

	i = 0;
	while(i++ < steps)
	{
		if((i % 100) == 0)
			fprintf(stderr, "%d\n", i);
		relax();
	}
	fprintf(stderr, "...done\n");

	printResult(num_sensors);

	for (i = 0; i < num_sensors; i++){
		printf("%lf %lf \n", points[i * 2 + 0], points[i * 2 + 1]);
	}

	DistanceMatrix *dm = new DistanceMatrix(num_sensors);
	printMap(num_sensors, dm->getMap(points));

	fprintf(stderr, "...done\n");

	dataPortOut.close();
	dataPortOut1.close();
	dataPortIn.close();
	free(dm);

	return 0;
}
