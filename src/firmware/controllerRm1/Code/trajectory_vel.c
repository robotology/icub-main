/*
 *
 * minimum jerk trajectory generation.
 * 	this code uses floating point emulation.
 *
 */
 
#include "controller.h"
#include "asc.h"

Int32 _dx0[JN] = { 0, 0 };
Int32 _prev_a[JN] = { 0, 0 };

/* global variables */
extern Int32 _x0[];
extern Int32 _xf[];
extern Int32 _distance[];


extern float _tf[];
extern float _curtf[];
extern Int32 _curstepf[];

extern Int16 _period;	/* in ms */
extern float _stepf[];

extern bool _ended[];


/* local prototypes */
float p5f_vel (float t, byte jj);
float p5f_vel_vel (float t, byte jj);
Int32 compute_current_vel(byte jj);


/* x0 + dx0 t + (-6 dx0 - 10 x0 + 10 xf) t^3

                                          
         + (8 dx0 + 15 x0 - 15 xf) t^4

                                         
         + (-6 x0 - 3 dx0 + 6 xf) t^5 */
         
float p5f_vel (float t, byte jj)
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

float p5f_vel_vel (float t, byte jj)
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
		a = p5f_vel_vel (_curtf[jj], jj);
		
		return (Int32)a;
	}			
	
	return 0;
}

Int16 init_trajectory_vel (byte jj, Int32 current, Int32 final, Int16 speed)
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


/* calculate next step in trajectory generation (floating point version) */
Int32 step_trajectory_vel (byte jj)
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
		a = p5f_vel (_curtf[jj], jj);
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