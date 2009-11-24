/*******************************************************************************
 * Copyright (C) 2009 Christian Wressnegger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *******************************************************************************/

#include "iCub/vislab/Eye2world.h"

using namespace std;
using namespace iKin;
using namespace ctrl;

using namespace vislab::util;
using namespace vislab::yarp::util;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

namespace vislab {
namespace math {

Eye2world::Eye2world() :
	AbstractRFModule("eye2world") {
}

bool Eye2world::configure(ResourceFinder &rf) {
	AbstractRFModule::configure(rf);
	ConstString str;

	str = "Adds a specific value to the z coordinate of the transformation to the projection plane.";
	addModuleOption(new Option("heightOffset", str, Option::NUMERIC, Value(0.0)));
	str
			= "This value is added to the z coordinate of the transformation to the projection plane but is again subtracted from the result afterwards.";
	addModuleOption(new Option("motor2eye", str, Option::NUMERIC, Value(0.06)));
	str
			= "A scale with is applied to the results. This might be used to compensate calibration issues.";
	addModuleOption(new Option("scale", str, Option::NUMERIC, Value(1.0)));

	Value* v = Value::makeList("1.0 2 3.3");
	addModuleOption(new Option("vector", "Test vector", Option::VECTOR, *v));
	delete v;

	ConstString robotPort = prefix + robotName;

	ConstString calibrationFilename = rf.check("eyeCalibration", Value("iCubEyes.ini"),
			"Name of the configuration file of the iCub's eye calibration (string)").asString();

	ConstString tableConfiguration = rf.check("tableConfiguration", Value("table.ini"),
			"Name of the configuration file of the experiments table (string)").asString();

	str
			= "Input port providing 2D coordinates as Bottle([ConstString:cam,] double:x, double:y[, double:height])";
	str = prefix + getName(rf.check("in", Value("/in"), str).asString());
	dataPorts.add(id.Input_Coordinates2d, str, new BufferedPort<Bottle> );

	str = prefix + getName(rf.check("headState", Value(robotPort + "/head/state:i"),
			"Input port providing the head's position").asString());
	dataPorts.add(id.Input_HeadPosition, str, new BufferedPort<Bottle> );

	str = prefix + getName(rf.check("torsoState", Value(robotPort + "/torso/state:i"),
			"Input port providing the torso's position").asString());
	dataPorts.add(id.Input_TorsoPosition, str, new BufferedPort<Bottle> );

	str = prefix + getName(rf.check("out", Value("/out"),
			"Output port providing 3D coordinates as Bottle(double:x, double:y, double:z)").asString());
	dataPorts.add(id.Output_Coordinates3d, str, new BufferedPort<Bottle> );

	ResourceFinder config;
	config.setDefaultConfigFile(calibrationFilename);
	config.setDefaultContext("");
	config.configure("ICUB_ROOT", 0, NULL, false);
	rightEyeCalibration.fromString(config.findGroup("CAMERA_CALIBRATION_RIGHT").toString());
	leftEyeCalibration.fromString(config.findGroup("CAMERA_CALIBRATION_LEFT").toString());

	cameras["right"] = &rightEyeCalibration;
	cameras["left"] = &leftEyeCalibration;

	ResourceFinder tableConfig;
	tableConfig.setDefaultConfigFile(tableConfiguration);
	tableConfig.setDefaultContext("");
	tableConfig.configure("ICUB_ROOT", 0, NULL, false);
	cout << tableConfig.findGroup("POSITION").toString() << endl;
	tabletopPosition.fromString(tableConfig.findGroup("POSITION").toString());

	vector<unsigned int> errorLog;
	if (!dataPorts.open(&errorLog)) {
		for (unsigned int i = 0; i < errorLog.size(); i++) {
			cout << getName() << ": unable to open port " << dataPorts.getName(i) << endl;
		}
		return false; // unable to open; let RFModule know so that it won't run
	}

	return startThread();
}

Thread* Eye2world::createWorkerThread() {
	workerThread = new WorkerThread(moduleOptions, dataPorts, id, cameras, tabletopPosition);
	return workerThread;
}

Eye2world::WorkerThread::WorkerThread(const OptionManager& moduleOptions,
		const Contactables& ports, struct PortIds ids, map<const string, Property*>& cams,
		Property& table) :
	AbstractWorkerThread(moduleOptions, ports) {

	id = ids;

	map<const string, Property*>::const_iterator itr;
	for (itr = cams.begin(); itr != cams.end(); ++itr) {
		const string key = (*itr).first;
		Property *p = (*itr).second;

		CameraCalibration c;
		c.fx = p->find("fx").asDouble();
		c.fy = p->find("fy").asDouble();
		c.cx = p->find("cx").asDouble();
		c.cy = p->find("cy").asDouble();
		c.fx = c.fx == 0.0 ? 1.0 : c.fx;
		c.fy = c.fy == 0.0 ? 1.0 : c.fy;

		cameras[key] = c;
	}
	tabletopPosition.resize(3);
	tabletopPosition[0] = table.find("x").asDouble();
	tabletopPosition[1] = table.find("y").asDouble();
	tabletopPosition[2] = table.find("z").asDouble();
}

bool Eye2world::WorkerThread::threadInit() {
	return true;
}

//#define DEBUG_OFFLINE

void Eye2world::WorkerThread::run() {
	VectorOf<double> head6d(6);
	VectorOf<double> torso3d(3);
	double prevZOffset = 0.0;

#ifndef DEBUG_OFFLINE
	bool positionUpdated = false;
	bool transformationAvailable = false;
	BufferedPort<Bottle>* in = (BufferedPort<Bottle>*) dataPorts[id.Input_Coordinates2d];
	BufferedPort<Bottle>* head = (BufferedPort<Bottle>*) dataPorts[id.Input_HeadPosition];
	BufferedPort<Bottle>* torso = (BufferedPort<Bottle>*) dataPorts[id.Input_TorsoPosition];
#endif
	BufferedPort<Bottle>* out = (BufferedPort<Bottle>*) dataPorts[id.Output_Coordinates3d];

	map<const string, iKinChain*> eyes;
	map<const string, Projection*> projections;

	map<const string, CameraCalibration>::const_iterator camItr;
	for (camItr = cameras.begin(); camItr != cameras.end(); ++camItr) {
		const string key = (*camItr).first;
		const CameraCalibration cam = (*camItr).second;

		iCubEye *eye = new iCubEye(key.c_str());
		for (int j = 0; j < 3; j++) {
			eye->releaseLink(j);
		}
		iKinChain *chain = eye->asChain();
		eyes[key] = chain;
		(*chain)[6].setMax(35 * M_PI / 180.0);

		projections[key] = new Projection(cam, &tabletopPosition);
	}

	while (!isStopping()) {

		string camera = "left";
		Vector object2d(2);
		double zOffset = 0;

#ifdef DEBUG_OFFLINE
		yarp::os::Time::delay(1);
#else
		do {
			inputCoordinates = in->read(true);
		} while (inputCoordinates == NULL && !isStopping());

		if (isStopping()) {
			break;
		}

		unsigned int coordinatesPos = 0;
		if (inputCoordinates->get(0).isString()) {
			camera = inputCoordinates->get(0).asString().c_str();
			// optional! default: 0.0
			coordinatesPos = 1;
		} else {
			// Assume (x,y) pair!
			// This is for testing with YARP viewer click port.
		}

		double motor2eye = moduleOptions["motor2eye"].getValue().asDouble();

		for (unsigned int i = 0; i < 2; i++) {
			object2d[i] = inputCoordinates->get(coordinatesPos + i).asDouble();
		}
		zOffset = moduleOptions["heightOffset"].getValue().asDouble();
		zOffset += motor2eye;
		zOffset += inputCoordinates->get(coordinatesPos + 2).asDouble();

		positionUpdated = zOffset != prevZOffset;

		Bottle* headPosition = head->read(!transformationAvailable);
		if (headPosition != NULL && headPosition->size() >= 6) {
			//get data and convert from degrees to radiant
			for (unsigned int i = 0; i < 6; i++) {
				head6d[i] = headPosition->get(i).asDouble() * M_PI / 180.0;
			}
			positionUpdated = true;
		}

		Bottle* torsoPosition = torso->read(!transformationAvailable);
		if (torsoPosition != NULL && torsoPosition->size() >= 3) {
			//get data and convert from degrees to radiant
			for (unsigned int i = 0; i < 3; i++) {
				torso3d[i] = torsoPosition->get(i).asDouble() * M_PI / 180.0;
			}
			positionUpdated = true;
		}

		if (positionUpdated || !transformationAvailable) { // The 2nd one is just to be sure ;)
			Vector v(8);
			// order according to http://eris.liralab.it/wiki/ICubHeadKinematics
			v[0] = torso3d[2]; // pitch
			v[1] = torso3d[1]; // roll
			v[2] = torso3d[0]; // yaw
			v[3] = head6d[0]; // neck pitch
			v[4] = head6d[1]; // neck roll
			v[5] = head6d[2]; // neck yaw
			v[6] = head6d[3]; // tilt of the eye's
			v[7] = camera == "left" ? head6d[4] + head6d[5] / 2 : head6d[4] - head6d[5] / 2;
			// angle of the eye according to http://eris.liralab.it/wiki/Vergence%2C_Version_and_Disparity
			// TODO: I don't like how I do the distinction at this point :/

			// eye -> robot
			Matrix T = SE3inv(eyes[camera]->getH(v));
			projections[camera]->setHeightOffset(zOffset, false);
			projections[camera]->setBaseTransformation(T);
			transformationAvailable = true;
		}
#endif

		Vector object3d(3);

		projections[camera]->project(object2d, object3d);

		// Add scalor to the result
		double scale = moduleOptions["scale"].getValue().asDouble();
		object3d[0] = object3d[0] * scale;
		object3d[1] = object3d[1] * scale;

		object3d[2] = object3d[2] - motor2eye;

		Bottle &outputCoordinates = out->prepare();
		outputCoordinates.clear();
		for (unsigned int i = 0; i < 3; i++) {
			outputCoordinates.addDouble(object3d[i]);
		}
		cout << outputCoordinates.toString() << endl;
		out->write();
	}

	map<const string, Projection*>::const_iterator itr;
	for (itr = projections.begin(); itr != projections.end(); ++itr) {
		const string key = (*itr).first;

		delete projections[key];
		delete eyes[key];
	}
}

void Eye2world::WorkerThread::threadRelease() {
}

}
}
