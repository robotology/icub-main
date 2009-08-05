/*
 *
 * minimum jerk trajectory generation.
 * 	this code uses floating point emulation.
 *
 */
 
#include "controller.h"
#include "asc.h"
#include "trajectory.h"

/******************************************************/
// global variables
/******************************************************/

//bool _actv[JN] = { false, false };

Int32 _x0[JN] = INIT_ARRAY (0);
Int32 _xf[JN] = INIT_ARRAY (0);
Int32 _distance[JN] = INIT_ARRAY (0);
Int32 _dx0[JN] = INIT_ARRAY (0);
Int32 _prev_a[JN] = INIT_ARRAY (0);

float _tf[JN] = INIT_ARRAY (0.);
float _curtf[JN] = INIT_ARRAY (0.);
Int32 _curstepf[JN] = INIT_ARRAY (0);


float _stepf[JN] = INIT_ARRAY (0.);

bool  _ended[JN]= INIT_ARRAY (true);

Int16 _period = CONTROLLER_PERIOD;	/* in ms */
//extern bool EnablePrintOnScreen;

/****************************************************************************/
// local prototypes 
/****************************************************************************/
float p5f (float x, byte jj);
float p5f_vel (float x, byte jj);
Int32 compute_current_vel(byte jj);

/****************************************************************************/
/* (10 * (t/T)^3 - 15 * (t/T)^4 + 6 * (t/T)^5) * (x0-xf) + x0 */
float p5f (float t, byte jj)
{
	float xfx0 = _xf[jj] - _x0[jj];
	float dx0  = _dx0[jj];

	float accum = dx0*t;
	float tmp = t * t * t;
	accum = accum + tmp * (10*xfx0 - 6*dx0);
	tmp *= t;
	accum = accum - tmp * (15*xfx0 - 8*dx0);
	tmp *= t;
	accum = accum + tmp * (6*xfx0 - 3*dx0);
	return accum;
}

float p5f_vel (float t, byte jj)
{
	float x0 = _x0[jj];
	float xf = _xf[jj];
	float dx0  = _dx0[jj];
	
	float accum = -2*dx0*t-dx0;
	float tmp = t * t;
	accum = accum + 30*tmp*x0+15*tmp*dx0-30*tmp*xf;
	tmp = (t-1)*(t-1);
	accum = -accum/_tf[jj] * tmp;
	return accum;
}

Int32 compute_current_vel(byte jj)
{
	float a;
	
	/* (10 * (t/T)^3 - 15 * (t/T)^4 + 6 * (t/T)^5) * (x0-xf) + x0 */
	if (_ended[jj])
		return 0;
		
	if (_curtf[jj] == 0)
	{
		return 0;
	}
	else
	if (_curtf[jj] < 1.0 - _stepf[jj])
	{
		/* calculate the velocity */
		a = p5f_vel (_curtf[jj], jj);
		
		return (Int32)a;
	}			
	
	return 0;
}

/****************************************************************************/
// Init trajectory
/****************************************************************************/
Int16 init_trajectory (byte jj, Int32 current, Int32 final, Int16 speed)
{

	float speedf = __abs(speed);

	
	//if (!_ended[jj] || speed <= 0)
	if (speed <= 0)
		return -1;
	
	_dx0[jj] = compute_current_vel(jj);
	_x0[jj] = current;
	_prev_a[jj] = current;
	_xf[jj] = final;
	
	_distance[jj] = _xf[jj] - _x0[jj];
	_tf[jj] = 100 *__labs (_distance[jj]) / speedf;
	_tf[jj] /= (float)_period;
	_dx0[jj] = _dx0[jj] * _tf[jj];

	if (_tf[jj] < 1 || _tf[jj] == 0)
	{
		abort_trajectory (jj, final);
		_stepf[jj]=0;
		return -1;
	
	}
	else
	{
		_stepf[jj] = 1 / _tf[jj];
	}
		
	_curtf[jj] = 0;
	_curstepf[jj] = 0;
	_ended[jj] = false;
	
	return 0;
}



/****************************************************************************/
// Abort Trajectory
/****************************************************************************/
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

/****************************************************************************/
// Step trajectory
/****************************************************************************/
/* calculate next step in trajectory generation (floating point version) */
Int32 step_trajectory (byte jj)
{
	Int32 a;
	Int32 delta_a;
	
	/* (10 * (t/T)^3 - 15 * (t/T)^4 + 6 * (t/T)^5) * (x0-xf) + x0 */
	if (_ended[jj])
	{
		a = _xf[jj];
		delta_a = a - _prev_a[jj];
		_prev_a[jj] = a;
		return a;
	}
		
	if (_curtf[jj] == 0)
	{
		_curtf[jj] += _stepf[jj];
		_curstepf[jj] ++;
		
		a = _x0[jj];
		delta_a = a - _prev_a[jj];
		_prev_a[jj] = a;
		return a;

	}
	else
	if (_curtf[jj] < 1.0 - _stepf[jj])
	{
		/* calculate the power factors */
		a = p5f (_curtf[jj], jj);
		a += _x0[jj];
		
		/* time */
		_curtf[jj] += _stepf[jj];
		_curstepf[jj] ++;

		delta_a = a - _prev_a[jj];
		_prev_a[jj] = a;
		return a;
	}			

	_ended[jj] = true;
	return _xf[jj];
}

/****************************************************************************/
// Step trajectory
/****************************************************************************/
/* calculate next step in trajectory generation (floating point version) */
Int32 step_trajectory_delta (byte jj)
{
	Int32 a;
	Int32 delta_a;
	
	/* (10 * (t/T)^3 - 15 * (t/T)^4 + 6 * (t/T)^5) * (x0-xf) + x0 */
	if (_ended[jj])
	{
		a = _xf[jj];
		delta_a = a - _prev_a[jj];
		_prev_a[jj] = a;
		return delta_a;
	}
		
	if (_curtf[jj] == 0)
	{
		_curtf[jj] += _stepf[jj];
		_curstepf[jj] ++;
		
		a = _x0[jj];
		delta_a = a - _prev_a[jj];
		_prev_a[jj] = a;
		return delta_a;

	}
	else
	if (_curtf[jj] < 1.0 - _stepf[jj])
	{
		/* calculate the power factors */
		a = p5f (_curtf[jj], jj);
		a += _x0[jj];
		
		/* time */
		_curtf[jj] += _stepf[jj];
		_curstepf[jj] ++;

		delta_a = a - _prev_a[jj];
		_prev_a[jj] = a;
		return delta_a;
	}			

	_ended[jj] = true;
	return 0;
}