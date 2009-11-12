#include <iCub/yarpTimer.h>
using yarp::os::Time;

YarpTimer::YarpTimer()
{
  reset();
};

YarpTimer::~YarpTimer()
{
};

	
void YarpTimer::reset()
{
  lap_counter = 1;
  last_lap = 0;
  global_acc = 0;
  lap_acc = 0;
  global_start = Time::now();
};
		
double YarpTimer::now()
{
  return Time::now() - global_start;
};


void YarpTimer::endlap()
{
  lap_counter++;
  global_acc += lap_acc;
  last_lap = lap_acc;
  lap_acc = 0;
  start();
}

void YarpTimer::start()
{
  lap_partial = Time::now();
}

void YarpTimer::stop()
{
  double delta = Time::now() - lap_partial;	
  lap_acc += delta;
}
		
int YarpTimer::cyc()
{
  return lap_counter;
}

double YarpTimer::lap()
{
  return lap_acc;
};

double YarpTimer::lastlap()
{
  return last_lap;
};

double YarpTimer::tot()
{
  return lap_acc;
}

double YarpTimer::avg()
{
  if(lap_counter)
    return (global_acc/lap_counter);
  else
    return 0;
}
