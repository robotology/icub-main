// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CrawlManagerModule.h"
#include <ace/OS.h>

#define DEBUG 1


bool CrawlManagerModule::updateModule()
{
    char key;
    
    cout << "Please specify the task" << endl;
    cout << "1 - Go to init pos" << endl;
    cout << "2 - Crawl" << endl;
    cout << "9 - stop" << endl;
    //cerr << key;
    
    if(com==9)
        return false;
    
    return true;
}

bool CrawlManagerModule::respond(const Bottle &command, Bottle &reply)
{
  com = command.get(0).asInt();
  cout << "soos " << com << endl;
  
   switch(com)
        {
        case 1:
            for(int i=0;i<4;i++)
                {
                    if(connected_part[i])
                        {
                            Bottle& paramBot = parts_port[i].prepare();
                            paramBot.clear();
                            for(int j=0;j<8;j++)
                                paramBot.addDouble(init_parameters[i][j]);
                            paramBot.addDouble(om_stance);
                            paramBot.addDouble(om_swing);
                            parts_port[i].write();
                            printf("SENDING COMMAND INIT TO PART %d\n", i);
                        }
                }
            reply.addString("going to init pos\n");
            break;
            
        case 2:
            for(int i=0;i<4;i++)
                {
                    if(connected_part[i])
                        {
                            Bottle& paramBot = parts_port[i].prepare();
                            paramBot.clear();
                            for(int j=0;j<8;j++)
                                paramBot.addDouble(crawl_parameters[i][j]);
                            paramBot.addDouble(om_stance);
                            paramBot.addDouble(om_swing);
                            parts_port[i].write();
                            printf("SENDING COMMAND CRAWL TO PART %d\n", i);
                        }
                }
            reply.addString("crawling!!\n");
            break;
        case 9:
            reply.addString("stopping\n");
            for(int i=0;i<4;i++)
                {
                    if(connected_part[i])
                        {
                            Bottle& paramBot = parts_port[i].prepare();
                            paramBot.clear();
                            for(int j=0;j<8;j++)
                                paramBot.addDouble(init_parameters[i][j]);
                            paramBot.addDouble(om_stance);
                            paramBot.addDouble(om_swing);
                            parts_port[i].write();
                        }
                }
            //reply.addString("going to stopping pos\n");
            break;
        default:
            break;
        }
    return true;
       
}
    


double CrawlManagerModule::getPeriod()

{

    //    ACE_OS::printf("Crawl Manager Module is running\n");

    return 2.0;

}




bool CrawlManagerModule::close()
{
    fprintf(stderr, "crawl manager module closing\n");
    //theThread->stop();
    //delete theThread;
    fprintf(stderr, "crawl manager module closed\n");
    return true;
}


bool CrawlManagerModule::open(yarp::os::Searchable &s)
{
    Property options(s.toString());
    Time::turboBoost();
    
    ConstString part_names[4] = {"left_arm","right_arm","left_leg","right_leg"};
    
    for(int i=0;i<4;i++)
        {
            char tmp1[255],tmp2[255];
            sprintf(tmp1,"/%s/parameters/in",part_names[i].c_str());

            /////////////////////////////////////////////////////
            ///check if the part is active and connect///////////
            /////////////////////////////////////////////////////
            Contact query = Network::queryName(tmp1);
            if(query.isValid())
                {
                    cout << "found part " << tmp2 << " ... connecting" << endl;
                    sprintf(tmp2,"/manager/%s/out",part_names[i].c_str());
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
            
            ///////we get
            if(options.check(part_names[i].c_str()))
                {
                    Bottle &bot = options.findGroup(part_names[i].c_str());
                    Bottle &bot2 = bot.findGroup("mu_crawl");
                    Bottle &bot3 = bot.findGroup("setPoints_crawl");
                    if(bot2.isNull() || bot3.isNull() || bot2.size()<5 || bot3.size()<5)
                        {
                            ACE_OS::printf("please specify crawl config for %s\n",part_names[i].c_str());
                            return false;
                        }
                    for(int j=0;j<4;j++)
                        {
                            crawl_parameters[i][2*j] = bot2.get(j+1).asDouble();
                            crawl_parameters[i][2*j+1] = bot3.get(j+1).asDouble()/180.0*3.1415;
                        }

                    Bottle &bot4 = bot.findGroup("mu_init");
                    Bottle &bot5 = bot.findGroup("setPoints_init");
                    if(bot4.isNull() || bot5.isNull() || bot4.size()<5 || bot5.size()<5)
                        {
                            ACE_OS::printf("please specify init config for %s\n",part_names[i].c_str());
                            return false;
                        }
                    for(int j=0;j<4;j++)
                        {
                            init_parameters[i][2*j] = bot4.get(j+1).asDouble();
                            init_parameters[i][2*j+1] = bot5.get(j+1).asDouble()/180.0*3.1415;
                        }
                }
            else
                {
                    ACE_OS::printf("please specify config for %s\n",part_names[i].c_str());
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

