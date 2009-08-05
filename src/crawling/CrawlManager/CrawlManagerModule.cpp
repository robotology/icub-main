// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CrawlManagerModule.h"
#include <ace/OS.h>

#define DEBUG 1


managerThread::managerThread(int period) : RateThread(period)
{
    this->period = ((double)period)/1000.0;
}

managerThread::~managerThread()
{}

void managerThread::run()
{
  for(int i=0;i<4;i++)
    {
      if(connected_part[i])
	{
	  Bottle& paramBot = parts_port[i].prepare();
	  paramBot.clear();
	  for(int j=0;j<8;j++)
	    paramBot.addDouble(parameters[i][j]);
	  paramBot.addDouble(om_stance);
	  paramBot.addDouble(om_swing);
	  parts_port[i].write();
	}
    }
}



bool managerThread::threadInit()
{
    fprintf(stderr, "crawl manager thread init\n");
    return true;
}



void managerThread::threadRelease()
{
    fprintf(stderr, "crawl manager thread releasing\n");
    ///we stop the vcControl

}

bool managerThread::init(Searchable &s)
{
    Property options(s.toString());
    Time::turboBoost();

    char *part_names[4] = {"left_arm","right_arm","left_leg","right_leg"};

    for(int i=0;i<4;i++)
      {
	char tmp1[255],tmp2[255];
	sprintf(tmp1,"/%s/parameters/in",part_names[i]);
	Contact query = Network::queryName(tmp1);
	if(query.isValid())
	  {
	    sprintf(tmp2,"/manager/%s/out",part_names[i]);
	    bool ok = parts_port[i].open(tmp2);
	    if(!ok)
	      {
		ACE_OS::printf("unable to open %s\n",tmp2);
		return false;
	      }
	    ok = Network::connect(tmp2,tmp1);
	    if(!ok)
	      {
		ACE_OS::printf("unable to connect %s with %s\n",tmp2,tmp1);
		return false;
	      }
	    connected_part[i] = true;
	  }
	else
	  connected_part[i] = false;

	if(options.check(part_names[i]))
	  {
	    Bottle &bot = options.findGroup(part_names[i]);
	    Bottle &bot2 = bot.findGroup("mu");
	    Bottle &bot3 = bot.findGroup("setPoints");
	    if(bot2.isNull() || bot3.isNull() || bot2.size()<5 || bot3.size()<5)
	      {
		ACE_OS::printf("please specify config for %s\n",part_names[i]);
		return false;
	      }
	    for(int j=0;j<4;j++)
	      {
		parameters[i][2*j] = bot2.get(j+1).asDouble();
		parameters[i][2*j+1] = bot3.get(j+1).asDouble();
	      }
	  }
	else
	  {
	    ACE_OS::printf("please specify config for %s\n",part_names[i]);
	    return false;
	  }
      }

    ////we get the frequencies
    if(options.check("omStance"))
      {
	Bottle& tmpBot = options.findGroup("omStance");
	if(tmpBot.size()==2)
	  {
	    om_stance = tmpBot.get(1).asDouble();
	  }
	else
	  {
	    ACE_OS::printf("Please specify omStance for manager\n");
	    return false;
	  }
      }
    else
      {
	ACE_OS::printf("Please specify omStance for manager\n");
	return false;
      }
    if(options.check("omSwing"))
      {
	Bottle& tmpBot = options.findGroup("omSwing");
	if(tmpBot.size()==2)
	  {
	    om_swing = tmpBot.get(1).asDouble();
	  }
	else
	  {
	    ACE_OS::printf("Please specify omSwing for manager\n");
	    return false;
	  }
      }
    else
      {
	ACE_OS::printf("Please specify omSwing for manager\n");
	return false;
      }
    
    return true;
}



////////////DRUM MANAGER MODULE/////////////////



double CrawlManagerModule::getPeriod()

{

    //    ACE_OS::printf("Crawl Manager Module is running\n");

    return 1.0;

}



bool CrawlManagerModule::updateModule()
{
    return true;

}



bool CrawlManagerModule::close()
{
    fprintf(stderr, "crawl manager module closing\n");
    theThread->stop();
    delete theThread;
    fprintf(stderr, "crawl manager module closed\n");
    return true;
}


bool CrawlManagerModule::open(yarp::os::Searchable &s)
{
    Property options(s.toString());
    int period = 500; // in ms

    if(options.check("period"))
        {
            period = options.find("period").asInt();
        }

    theThread = new managerThread(period);
    if(!theThread->init(s))
      return false;
    
    theThread->start();
    return true;

}

