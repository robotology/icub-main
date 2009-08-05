/*
 *
 * minimum jerk trajectory generation.
 * 	this code uses floating point emulation.
 *
 */
 
#include "controller.h"
#include "asc.h"

/* global variables */
//bool _actv[JN] = { false, false };
Int32 _x0[JN] = { 0, 0 };
Int32 _xf[JN] = { 0, 0 };
Int32 _distance[JN] = { 0, 0 };

float _tf[JN] = { 0., 0. };
float _curtf[JN] = { 0., 0. };
Int32 _curstepf[JN] = { 0, 0 };

Int16 _period = CONTROLLER_PERIOD;	/* in ms */
float _stepf[JN] = { 0., 0. };

bool _ended[JN] = { true, true };
//extern bool EnablePrintOnScreen;

/* local prototypes */
float p5f (float x);


/* (10 * (t/T)^3 - 15 * (t/T)^4 + 6 * (t/T)^5) * (x0-xf) + x0 */
float p5f (float x)
{
	float accum = 0.;
	float tmp = x * x * x;
	accum += (10 * tmp);
	tmp *= x;
	accum -= (15 * tmp);
	tmp *= x;
	accum += (6 * tmp);
	return accum;
}


Int16 init_trajectory (byte jj, Int32 current, Int32 final, Int16 speed)
{
	float currentf = current;
	float finalf = final;
	float speedf = __abs(speed);
	
	//if (!_ended[jj] || speed <= 0)
	if (speed <= 0)
		return -1;
	
	_x0[jj] = current;
	_xf[jj] = final;
	
	_distance[jj] = _xf[jj] - _x0[jj];
	_tf[jj] = 100 *__labs (_distance[jj]) / speedf;
	_tf[jj] /= (float)_period;
	_stepf[jj] = 1 / _tf[jj];
	
	if (_tf[jj] < 1)
	{
		abort_trajectory (jj, final);
		return -1;
	}
		
	_curtf[jj] = 0;
	_curstepf[jj] = 0;
	_ended[jj] = false;
	
	return 0;
}


Int16 abort_trajectory (byte jj, Int32 limit)
{
	if (!_ended[jj])
	{
		_ended[jj] = true;
		_curtf[jj] = 0;
		_curstepf[jj] = 0;
		_xf[jj] = limit;
	}
	else
	{
		_curtf[jj] = 0;
		_curstepf[jj] = 0;
		_xf[jj] = limit;
	}
	
	return 0;
}

/* calculate next step in trajectory generation (floating point version) */
Int32 step_trajectory (byte jj)
{
	float a;
	
	/* (10 * (t/T)^3 - 15 * (t/T)^4 + 6 * (t/T)^5) * (x0-xf) + x0 */
	if (_ended[jj])
		return _xf[jj];
		
	if (_curtf[jj] == 0)
	{
		_curtf[jj] += _stepf[jj];
		_curstepf[jj] ++;
		return _x0[jj];
	}
	else
	if (_curtf[jj] < 1.0 - _stepf[jj])
	{
		/* calculate the power factors */
		a = p5f (_curtf[jj]);
		a *= _distance[jj];
		a += _x0[jj];
		
		/* time */
		_curtf[jj] += _stepf[jj];
		_curstepf[jj] ++;

		return (Int32)a;
	}			

	_ended[jj] = true;
	return _xf[jj];
}