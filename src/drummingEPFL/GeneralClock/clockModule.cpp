
#include "clockModule.h"

#include <ace/OS.h>

clockThread::clockThread(int period) : RateThread(period)
{
  this->period = ((double)period)/1000.0;
}

clockThread::~clockThread()
{
}

//read the parameters coming from the manager and update the cpg myManager
void clockThread::getParameters()
{
    Bottle *bot = manager_port.read(false);
    if(bot!=NULL)
    {
        double freq = bot->get(0).asDouble();
        if(freq < MAX_FREQUENCY)
        nu = freq;
    }
}

void clockThread::run()
{
    double time_now = Time::now();
    double time_residue = time_now - original_time - theoretical_time;
    getParameters();
      
    theoretical_time += period + time_residue;

      //check if we must stop
    if(nu<0)
    {
        ACE_OS::printf("Task is finished\n");
        this->stop();
        return;
    }

    ///check the current beat
    if(time_now-lastBeat_time > 1.0/nu)
    {
        ACE_OS::printf("current beat: %d\n",beat);
        beat++;
        lastBeat_time = time_now;
          
        ///send the beat
        Bottle& output = beat_port.prepare();
        output.clear();
        output.addInt(beat);
        beat_port.write();
    }

    ///integrate the system
    double pi = 3.1415;
    int inner_steps = (int)((period+time_residue)/dt);
    for(int i=0;i<inner_steps;i++)
    {
        double r2 = current_state[0]*current_state[0] + current_state[1]*current_state[1];
        double dx = a*(m_on-r2)*current_state[0] - 2*pi*nu * current_state[1];
        double dy = a*(m_on-r2)*current_state[1] + 2*pi*nu * current_state[0];
        current_state[0] += dx*dt;
        current_state[1] += dy*dt;
    }

    fprintf(debug_file,"%f %f %f\n",current_state[0],current_state[1],Time::now());

    ////send the command
    Bottle &bot = clock_port.prepare();
    bot.clear();
    bot.addDouble(current_state[0]);
    bot.addDouble(current_state[1]);
    clock_port.write();
    
}

bool clockThread::threadInit()
{
  return true;
}

void clockThread::threadRelease()
{
    clock_port.close();
    manager_port.close();
    beat_port.close();

    fclose(debug_file);
}


bool clockThread::init(Searchable &s)
{
  Time::turboBoost();

  debug_file = fopen("debug.dat","w");
  
  ///default frequency
  nu = 0.5; //Hz
  a = 15.0;
  m_on = 1.0;
  dt = 0.001;

  current_state[0] = 0.0;
  current_state[1] = 1.0;

  ///We open the output port
  bool ok = clock_port.open("/clock/out");
  if(!ok)
    {
      ACE_OS::printf("cannot open clock port\n");
      return false;
    }
  ///we open the parameter port
  ok = manager_port.open("/clock/parameters/in");
  if(!ok)
    {
      ACE_OS::printf("cannot open manager port\n");
      return false;
    }

  ///we open the beat port
  ok = beat_port.open("/clock/check_motion/out");
  if(!ok)
    {
      ACE_OS::printf("failed to open beat port\n");
      return false;
    }

  beat = 0;

  theoretical_time = 0.0;
  original_time = Time::now();
  lastBeat_time = Time::now();

  return true;
}

////////////DRUM CLOCK MODULE/////////////////

double clockModule::getPeriod()
{
  ACE_OS::printf(" Clock Module is running\n");
  return 1.0;
}

bool clockModule::updateModule()
{
  return true;
}

bool clockModule::close()
{
  theThread->stop();
  delete theThread;
  return true;
}


bool clockModule::open(yarp::os::Searchable &s)
{
  Property options(s.toString());
  int period = 50; // in ms

  if(options.check("period"))
    {
      period = options.find("period").asInt();
    }
  
  theThread = new clockThread(period);
  if(!theThread->init(s))
    return false;

  theThread->start();
  return true;
}
