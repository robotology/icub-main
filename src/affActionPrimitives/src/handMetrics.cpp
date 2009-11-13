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

#include <iCub/handMetrics.h>

#include <yarp/os/Time.h>
#include <yarp/math/Math.h>


using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;

const double HandMetrics::pidLimit = 933.0;
const double HandMetrics::residualVoltageThres = 70.0;

const double FunctionSmoother::e=1.0;
const double FunctionSmoother::t=2.0;
const double FunctionSmoother::l=0.5;


double norm(const Vector &v)
{
    return sqrt(dot(v,v));
}


HandMetrics::HandMetrics(IEncoders* const encoders, IPidControl* const pidControl,
		                 IAmplifierControl* const ampControl, map<const string, Matrix>& constants) :
     	                 objectSensingConstants(constants)
{
    this->encoders  =encoders;
    this->pidControl=pidControl;
    this->ampControl=ampControl;

	for (int i=7; i<numAxes; i++)
    {   
        this->pidControl->disablePid(i); 
        this->ampControl->disableAmp(i);        
        this->pidControl->getPid(i,&prevPids[i]);
    }

    Pid pid1(-120,-1000,0,pidLimit,4,pidLimit);
    Pid pid2(100,1000,0,pidLimit,4,pidLimit);
    Pid pid3(-120,-1250,0,pidLimit,5,pidLimit);

	this->pidControl->setPid(8,pid1);
    this->pidControl->setPid(numAxes-1,pid3);

	for (int i=9; i<numAxes-1; i++) 
		this->pidControl->setPid(i,pid2);    

	for (int i=7; i<numAxes; i++)
    {    
        this->ampControl->enableAmp(i);
        this->pidControl->enablePid(i);
    }

	string neededConstants[]={"derivate_gain","offsets","springs"};
	for (int i=0; i<3; i++) 
    {
		map<const string, Matrix>::iterator itr=objectSensingConstants.find(neededConstants[i]);

		if (itr==objectSensingConstants.end() || itr->second.cols()!=numAxes)
        {
			ostringstream ss;
			ss << "Missing or incomplete constant `" << neededConstants[i] << "`";
			throw ss.str();
		}
	}

	prevVoltageOffset.resize(numAxes);
	prevSpringStiffness.resize(numAxes);
	error.resize(numAxes);
    position.resize(numAxes);

	prevTime=Time::now();

	snapshot();
}

HandMetrics::~HandMetrics()
{
    for (int i=7; i<numAxes; i++)
    {    
        pidControl->disablePid(i);
        ampControl->disableAmp(i);
        pidControl->setPid(i,prevPids[i]);
        ampControl->enableAmp(i);
        pidControl->enablePid(i);
    }
}

void HandMetrics::snapshot()
{
	prevPosition = position;

	// reset hand metrics (force re-computations)
	position.clear();
	velocity.clear();
	voltage.clear();
	error.clear();

	// positions
	position = getPosition();
	// voltages
	targetVoltage.resize(numAxes);
	pidControl->getOutputs(targetVoltage.data());

	prevTime = curTime;
	curTime = Time::now();
}

void HandMetrics::printSnapshotData(ostream& s)
{
	s << "delta t = "       << curTime - prevTime       << endl;
	s << "position = "      << position.toString()      << endl;
	s << "prePosition = "   << prevPosition.toString()  << endl;
	s << "targetVoltage = " << targetVoltage.toString() << endl;
}

Vector& HandMetrics::getPosition()
{
	if (position.size() <= 0)
    {
		position.resize(numAxes);
		Vector error(numAxes), output(numAxes);
		encoders->getEncoders(position.data());
		pidControl->getErrors(error.data());

#ifdef DEBUG
		cout << "original position = " << position.toString() << endl;
		cout << "error = "             << error.toString()    << endl;
#endif
		position = position + error;
	}

	return position;
}

Vector& HandMetrics::getVelocity()
{
	if (velocity.size() <= 0)
    {
		Vector deltaR = getPosition() - prevPosition;
		double deltaT = getTimeInterval();
		velocity = deltaR * (1.0/deltaT);

#ifdef DEBUG
		cout << "deltaR = " << deltaR.toString() << endl;
		cout << "deltaT = " << deltaT            << endl;
#endif
	}

	return velocity;
}

Vector& HandMetrics::getVoltage()
{
	if (voltage.size() <= 0)
    {
		voltage.resize(numAxes);
		Vector v = getVelocity();
		v = objectSensingConstants["derivate_gain"].getRow(0) * v;

		for (int i = 0; i < v.size(); i++)
        {
			// too many assignments! but hey -- it's readable ;)
			double offset = prevVoltageOffset[i];
			offset = v[i] < -residualVoltageThres ? objectSensingConstants["offsets"][0][i] : offset;
			offset = v[i] > residualVoltageThres ? -objectSensingConstants["offsets"][0][i] : offset;
			prevVoltageOffset[i] = offset;

			// dto
			double spring = prevSpringStiffness[i];
			spring = position[i] > 0 ? objectSensingConstants["springs"][0][i] : spring;
			spring = position[i] > 30 ? objectSensingConstants["springs"][1][i] : spring;
			spring = position[i] > 60 ? objectSensingConstants["springs"][2][i] : spring;
			prevSpringStiffness[i] = spring;

			voltage[i] = v[i] + offset + spring * position[i];

			voltage[i] = min(voltage[i], (double) pidLimit);
			voltage[i] = max(voltage[i], -(double) pidLimit);
		}

#ifdef DEBUG
	cout << "derivate = "     << v.toString()                   << endl;
	cout << "--- position = " << position.toString()            << endl;
	cout << "*offset = "      << prevVoltageOffset.toString()   << endl;
	cout << "*spring = "      << prevSpringStiffness.toString() << endl;
	cout << "voltage = "      << voltage.toString()             << endl;
#endif

	}

	return voltage;
}

Vector& HandMetrics::getError()
{
	if (error.size() <= 0)
		error = targetVoltage - getVoltage();

    return error;
}

double HandMetrics::getTimeInterval()
{
	return fabs(curTime - prevTime);
}


FunctionSmoother::FunctionSmoother(const Vector &residualVoltageThresholds)
{
	thresholds=residualVoltageThresholds;
    prevValues.resize(thresholds.length(),0.0);
}


Vector &FunctionSmoother::smooth(const Vector &v, Vector &smoothedValues, const double deltaT)
{
	if (v.length() != thresholds.length())
		throw "Values vector size mismatches the size of the thresholds.";

	smoothedValues.resize(v.length());

	for (int i = 0; i < v.length() && i < thresholds.length(); i++) 
    {
		double d = (prevValues[i] + v[i]) * l;

		if (deltaT > t) // reset integral
			smoothedValues[i] = 0;

        if (fabs(d - prevValues[i]) < e) 
			smoothedValues[i] = 0;
		else
        {
			bool isOpening = thresholds[i] > 0;

			if ((isOpening && d > 0) || (!isOpening && d < 0))
				smoothedValues[i] += (d / (700 * deltaT));

            // otherwise remain at 0
		}

		prevValues[i] = d;
	}

	return smoothedValues;
}



